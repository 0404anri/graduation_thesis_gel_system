import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import re

from gel_interfaces.action import VisionCommand
from gel_interfaces.srv import StimulationCommand

"""
概要:
    システム統合制御用タスクマネージャ

説明:
    Visionサーバー（Action）と電圧印加サービス（Service）のクライアントとして機能する。
"""

class TaskManager(Node):
    """
    画像認識結果に基づき、物体の配置領域を判定し、指令するノード。
    """
    def __init__(self):
        super().__init__('task_manager')

        # Visionサーバーへのアクションクライアント作成
        self._vision_client = ActionClient(self, VisionCommand, 'vision/command')

        # 電気刺激ノードへのサービスクライアント作成
        self._stim_client = self.create_client(StimulationCommand, 'stimulation/command')

        self.get_logger().info('クライアント起動: "capture orange" シーケンスを開始します')
        
        # 処理開始: 画像認識をリクエスト
        self.send_vision_goal("capture orange")

    def send_vision_goal(self, command_text):
        """
        画像認識サーバーへ撮影・解析リクエストを送信する。
        """
        # サーバー待機 (5秒でタイムアウト)
        if not self._vision_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Visionサーバー(Action)が見つかりません')
            self.shutdown_node()
            return

        goal_msg = VisionCommand.Goal()
        goal_msg.command = command_text

        self.get_logger().info(f'Visionリクエスト送信: "{command_text}"')
        
        send_future = self._vision_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """アクションサーバーからのリクエスト受諾確認"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Visionリクエストが拒否されました')
            self.shutdown_node()
            return

        self.get_logger().info('Visionリクエスト受諾。解析中...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Visionサーバーからの解析結果を受け取り、座標パースを行う。
        形式: "OK x=0.123 y=-0.456 z=0.789"
        """
        result = future.result().result
        ans_str = result.answer
        
        self.get_logger().info(f'Vision結果受信: {ans_str}')

        if "OK" in ans_str:
            try:
                # 正規表現を用いてレスポンス文字列から数値(x, y)を抽出
                match = re.search(r'x=([-\d\.]+).*y=([-\d\.]+)', ans_str)
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    # 座標に基づく刺激パターンの決定処理へ
                    self.process_coordinates(x, y)
                else:
                    self.get_logger().error('座標フォーマットの解析に失敗しました')
                    self.shutdown_node()
            except ValueError as e:
                self.get_logger().error(f'数値変換エラー: {e}')
                self.shutdown_node()
        else:
            self.get_logger().warn('ターゲットが見つかりませんでした')
            self.shutdown_node()

    def process_coordinates(self, x, y):
        """
        座標(x, y)から該当する領域を判定し、刺激サービスリクエストを送信する。
        Y軸方向の座標に応じて5つの領域に分割されている。
        """
        position_str = "領域外"
        stim_command = "0"

        # --- 領域判定ロジック ---
        # Region 1: y = -20cm ~ -15cm
        if 0 <= x <= 0.1 and -0.20 <= y <= -0.15:
            position_str = "領域1"
            stim_command = "1"
        
        # Region 2: y = -15cm ~ -10cm
        elif 0 <= x <= 0.1 and -0.15 <= y <= -0.10:
            position_str = "領域2"
            stim_command = "2"

        # Region 3: y = -10cm ~ -5cm
        elif 0 <= x <= 0.1 and -0.10 <= y <= -0.05:
            position_str = "領域3"
            stim_command = "3"

        # Region 4: y = -5cm ~ 0cm
        elif 0 <= x <= 0.1 and -0.05 <= y <= 0.00:
            position_str = "領域4"
            stim_command = "4"

        # Region 5: y = 0cm ~ +5cm
        elif 0 <= x <= 0.1 and 0.00 <= y <= 0.05:
            position_str = "領域5"
            stim_command = "5"

        self.get_logger().info(f'判定結果: {position_str} (x={x:.3f}, y={y:.3f}) -> コマンド: {stim_command}')

        # "0" (領域外) ではない場合のみサービスを呼び出す
        if stim_command != "0":
            self.call_stimulation_service(stim_command)
        else:
            self.get_logger().info("領域外のため、刺激は行わずに終了します。")
            self.shutdown_node()

    def call_stimulation_service(self, pattern_str):
        """
        決定されたパターンIDを電極制御ノードへ送信する。
        """
        # サービスサーバーの待機
        if not self._stim_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('電極制御サービス(Service)が見つかりません')
            self.shutdown_node()
            return

        # StimulationCommand の Request を作成
        req = StimulationCommand.Request()
        req.command = pattern_str

        self.get_logger().info(f'サービス送信中... ({pattern_str})')
        
        future = self._stim_client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """サービスの完了を受け取ってノードを終了する"""
        try:
            response = future.result()
            self.get_logger().info(f'電極制御完了: {response.answer}')
        except Exception as e:
            self.get_logger().error(f'サービス呼び出し失敗: {e}')

        self.shutdown_node()

    def shutdown_node(self):
        """ノードの終了処理"""
        self.get_logger().info('全処理完了。終了します。')
        raise SystemExit

def main():
    rclpy.init()
    node = TaskManager()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()