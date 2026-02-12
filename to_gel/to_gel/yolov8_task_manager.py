import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import threading
import sys
from std_msgs.msg import String

# ユーザー定義のアクションインターフェース
from gel_interfaces.action import StringCommand

"""
概要:
    YOLOv8追跡連動型タスクマネージャ

説明:
    ユーザーからのコマンド（例: "track cup"）を受け付けてVisionサーバーへ送信すると同時に、
    TFツリーを監視して対象物の座標をリアルタイムに取得する。
    座標が特定の領域に入った場合、対応する電気刺激パターンをパブリッシュする。
"""

class Yolov8TaskManager(Node):
    """
    TFリスナーを用いて物体の位置を常時監視し、刺激フィードバックを行うノード。
    """
    def __init__(self):
        super().__init__('yolov8_task_manager')

        # Visionサーバー制御用のアクションクライアント
        self._action_client = ActionClient(self, StringCommand, 'vision/command')

        # 電気刺激制御用のパブリッシャ (トピック通信により連続的な指令が可能)
        self.stimulation_publisher = self.create_publisher(String, 'stimulation_command', 10)

        # TFリスナーの定義 (座標変換の受信バッファ)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ターゲット（物体）のフレーム名（YOLOノードが配信するフレーム名と一致させる）
        self.target_frame = 'target'
        
        # 基準とするフレーム名（カメラ座標系）
        self.source_frame = 'camera_color_optical_frame'

        # 座標監視・判定用タイマー (0.5秒周期で実行)
        self.timer = self.create_timer(0.5, self.on_tf_timer)
        
        self.get_logger().info('クライアント起動完了')
        self.get_logger().info('コマンド入力待機中... (例: "track cup", "find bottle", "stop")')

    def send_goal(self, command_text):
        """
        Visionサーバーへ追跡開始や停止のコマンドを送信する。
        """
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('アクションサーバーが見つかりません')
            return

        goal_msg = StringCommand.Goal()
        goal_msg.command = command_text

        self.get_logger().info(f'コマンド送信: "{command_text}"')
        
        # 非同期でゴールを送信
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """サーバーがゴールを受け付けたかどうかのコールバック"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('ゴールが拒否されました')
            return

        self.get_logger().info('ゴールが受け付けられました')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """アクションの結果（終了時）のコールバック"""
        result = future.result().result
        self.get_logger().info(f'アクション完了 結果: {result.answer}')

    def on_tf_timer(self):
        """
        タイマー割り込み関数。
        最新のTF座標を取得し、領域判定を行って刺激コマンドを送信する。
        """
        stim_command = "0,0,0"  # デフォルト値（検出なし/停止）
        
        try:
            # 最新のTFを取得 (source_frame -> target_frame)
            t = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y
            
            position_str = "領域外"

            # --- 領域判定ロジック ---
            # Region 1: y = -20cm ~ -15cm
            if 0 <= x <= 0.1 and -0.20 <= y <= -0.15:
                position_str = "領域1"
                stim_command = "1,0,0"
        
            # Region 2: y = -15cm ~ -10cm
            elif 0 <= x <= 0.1 and -0.15 <= y <= -0.10:
                position_str = "領域2"
                stim_command = "1,1,0"

            # Region 3: y = -10cm ~ -5cm
            elif 0 <= x <= 0.1 and -0.10 <= y <= -0.05:
                position_str = "領域3"
                stim_command = "0,1,0"

            # Region 4: y = -5cm ~ 0cm
            elif 0 <= x <= 0.1 and -0.05 <= y <= 0.00:
                position_str = "領域4"
                stim_command = "0,1,1"

            # Region 5: y = 0cm ~ +5cm
            elif 0 <= x <= 0.1 and 0.00 <= y <= 0.05:
                position_str = "領域5"
                stim_command = "0,0,1"
            
            # コンソールへのデバッグ表示（現在位置と判定領域）
            # \r を使用して同じ行を上書き更新する
            if stim_command != "0,0,0":
                print(f'\r[検出中] {position_str} | x={x:.3f}, y={y:.3f} -> Cmd: {stim_command}   ', end='')
            
        except TransformException:
            # 物体が見つかっていない、またはTFが配信されていない場合
            pass

        # 判定された刺激コマンドをパブリッシュ（検出なしの場合は停止コマンド）
        self.publish_stimulation(stim_command)

    def publish_stimulation(self, command_str):
        """
        指定された文字列を stimulation_command トピックへ送信する。
        
        """
        msg = String()
        msg.data = command_str
        self.stimulation_publisher.publish(msg)

def main():
    """
    メイン関数: ノードの初期化と入力待ちループ
    """
    rclpy.init()
    client = Yolov8TaskManager()

    # ROSのコールバック処理（TF受信など）をバックグラウンドスレッドで実行
    executor = rclpy.executors.MultiThreadedExecutor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(client, executor), daemon=True)
    spin_thread.start()

    try:
        while True:
            # ユーザーからの入力を受け付ける（ブロッキング）
            user_input = input("\nコマンドを入力してください (qで終了) > ")
            
            if user_input.lower() == 'q':
                break
            
            if user_input:
                # 入力された文字列をそのままアクションサーバーへ送信
                # 例: "track cup", "find refrigerator", "stop"
                client.send_goal(user_input)
            
            # CPU負荷低減のための短いスリープ
            threading.Event().wait(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        # 終了時に必ず刺激を停止する
        client.publish_stimulation("0,0,0")
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()