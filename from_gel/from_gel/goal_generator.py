import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray
from gel_interfaces.action import StringCommand
import numpy as np
import scipy.optimize as opt
import time
import csv
import os
from datetime import datetime
from pathlib import Path

"""
概要:
    電流センサ値に基づく目標座標決定およびアクション制御ノード

説明:
    本プログラムは、INA219電流センサから取得した電流値（3点）を用いて二次多項式フィッティングを行い、
    ピーク値を算出する。
    データ入力が途切れたタイミングで最終的な判定を行い、有効なピーク（上に凸な放物線）
    である場合にロボットのPick & Place動作を指令する。
"""

# --- 電流値の正規化のための設定値 ---
# 実験環境におけるセンサ配置と正規化パラメータ
SCREEN_WIDTH = 10         # 操作対象の全幅（座標単位）
ORIG_TOP = -7.3           # 上部センサの基準電流値（無負荷時）
ORIG_MID = -8.1           # 中部センサの基準電流値
ORIG_BOT = -9.5           # 下部センサの基準電流値
UP_RANGE_TOP = 15.5       # 上部センサの正規化上限レンジ
UP_RANGE_MID = 12.0       # 中部センサの正規化上限レンジ
UP_RANGE_BOT = 18.0       # 下部センサの正規化上限レンジ
LOW_RANGE_TOP = 1.5       # 正規化下限レンジ
LOW_RANGE_MID = 1.5
LOW_RANGE_BOT = 1.5

RAW_THRESHOLD = -12       # ノイズ判定閾値（これ以下の値は無視する）
ACTION_NAME = 'manipulation/command'
TOPIC_NAME = 'ina219_current_data'

# --- 保存先設定 ---
HOME_DIR = '/home/neya/gel_ws'
CSV_DIR_NAME = os.path.join(HOME_DIR, 'peak_history')
CSV_FILE_NAME = 'peak_data.csv'

# --- 関数定義 ---
def map_current(current, max_c, min_c):
    """
    電流値を指定されたレンジに基づいて 0.0 ~ 1.0 に正規化する関数。

    Args:
        current (float): 現在の生電流値
        max_c (float): 正規化の最大値（1.0に対応）
        min_c (float): 正規化の最小値（0.0に対応）

    Returns:
        float: 0.0 から 1.0 の範囲にクリップされた正規化値
    """
    if max_c - min_c == 0: return 0
    temp = (current - min_c) / (max_c - min_c)
    return np.clip(temp, 0, 1) 

def quadratic_func(x, a, b, c):
    """
    カーブフィッティング用の二次関数モデル。
    f(x) = ax^2 + bx + c
    """
    return a * np.power(x, 2) + b * x + c

class GoalGenerator(Node):
    """
    センサデータから目標座標を生成し、ロボットアクションを実行するROS 2ノード。
    """
    def __init__(self):
        super().__init__('goal_generator')
        self._action_client = ActionClient(self, StringCommand, ACTION_NAME)
        self.subscription = self.create_subscription(
            Float32MultiArray, TOPIC_NAME, self.listener_callback, 10
        )
        
        # センサの配置座標（3点）
        self.sensor_x = np.array([
            int((9 * SCREEN_WIDTH) / 10),        
            int((5 * SCREEN_WIDTH) / 10), 
            int(SCREEN_WIDTH / 10)
        ])
        
        # 最新の計算結果を保持する変数
        self.latest_valid_peak = None
        self.latest_a = None
        
        self.last_msg_time = 0.0
        self.is_moving = False
        
        # データ途切れ監視用タイマー (0.5秒周期)
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
        
        # ログ保存用ディレクトリの作成
        if not os.path.exists(CSV_DIR_NAME):
            os.makedirs(CSV_DIR_NAME)
            self.get_logger().info(f'Created directory: {CSV_DIR_NAME}')
            
        self.csv_path = os.path.join(CSV_DIR_NAME, CSV_FILE_NAME)
        self.init_csv()
        
        self.get_logger().info(f'Client Started. Logging to: {self.csv_path}')

    def init_csv(self):
        """CSVファイルの初期化（ヘッダー書き込み）を行う"""
        if not os.path.exists(self.csv_path):
            try:
                with open(self.csv_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        'datetime', 'timestamp', 'peak_x', 
                        'raw_top', 'raw_mid', 'raw_bot',
                        'norm_top', 'norm_mid', 'norm_bot',
                        'a_val'
                    ])
            except Exception as e:
                self.get_logger().error(f"Failed to create CSV: {e}")

    def save_to_csv(self, peak_x, raw_vals, norm_vals, a_val):
        """
        計算結果とセンサ生データをCSVに追記保存する。
        """
        try:
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                now = datetime.now()
                writer.writerow([
                    now.strftime("%Y-%m-%d %H:%M:%S.%f"), 
                    time.time(),            
                    f"{peak_x:.4f}",        
                    f"{raw_vals[0]:.2f}",   
                    f"{raw_vals[1]:.2f}",   
                    f"{raw_vals[2]:.2f}",
                    f"{norm_vals[0]:.4f}", 
                    f"{norm_vals[1]:.4f}", 
                    f"{norm_vals[2]:.4f}",
                    f"{a_val:.6f}"
                ])
        except Exception as e:
            self.get_logger().error(f"CSV Write Error: {e}")

    def listener_callback(self, msg):
        """
        トピック購読コールバック関数。
        受信した電流データを正規化し、二次多項式フィッティングによってピーク値を算出する。
        """
        if self.is_moving:
            return

        raw_vals = msg.data
        if len(raw_vals) < 3: return
        
        # 閾値チェック (ノイズ対策)
        if max(raw_vals) < RAW_THRESHOLD:
            return

        # データを検知した時刻を更新
        self.last_msg_time = time.time()

        # --- 毎回計算を実行 ---
        val_top, val_mid, val_bot = raw_vals[0], raw_vals[1], raw_vals[2]
        
        # 電流値の正規化
        c_top = map_current(val_top, ORIG_TOP + UP_RANGE_TOP, ORIG_TOP - LOW_RANGE_TOP)
        c_mid = map_current(val_mid, ORIG_MID + UP_RANGE_MID, ORIG_MID - LOW_RANGE_MID)
        c_bot = map_current(val_bot, ORIG_BOT + UP_RANGE_BOT, ORIG_BOT - LOW_RANGE_BOT)
        
        norm_vals = [c_top, c_mid, c_bot]
        y_data = np.array(norm_vals)

        try:
            # カーブフィッティング (scipy.optimize.curve_fit)
            params, _ = opt.curve_fit(quadratic_func, self.sensor_x, y_data)
            a, b, c = params
            
            # 頂点のx座標 (-b/2a) を計算
            if a != 0:
                calc_x = -b / (2 * a)
            else:
                # 直線の場合は暫定的に端を設定
                calc_x = 9.0 if b > 0 else 1.0

            peak_x = np.clip(calc_x, 0, SCREEN_WIDTH)
            
            # 最新の値を常に更新して保持する
            self.latest_valid_peak = peak_x
            self.latest_a = a
            
            # すべての計算結果をCSVに保存する
            self.save_to_csv(peak_x, raw_vals, norm_vals, a)
            self.get_logger().info(f'Updated: x={peak_x:.2f}, a={a:.4f}')

        except Exception as e:
            self.get_logger().warn(f'Fitting failed: {e}')
            
    def watchdog_callback(self):
        """
        データ途絶監視コールバック関数。
        データの受信が一定時間（1.0秒）途絶えた場合、動作完了とみなし、
        保持している最新のピーク情報に基づいてアクション送信の可否を決定する。
        """
        # 移動中、または有効なデータがない場合は何もしない
        if self.latest_valid_peak is None or self.is_moving:
            return

        # データが途切れてから 1.0秒経過したら「終了」とみなして判定を行う
        if time.time() - self.last_msg_time > 1.0:
            
            final_peak = self.latest_valid_peak
            final_a = self.latest_a

            # 判定条件: フィッティング結果が上に凸 (a < 0) であること
            if final_a is not None and final_a < 0:
                self.get_logger().info(f"Stream ended. Decision: MOVE (a={final_a:.4f} < 0). Target: {final_peak:.2f}")
                self.send_goal(final_peak)
            
            # a > 0 (下に凸) または a=0 の場合は誤検知または不適切な入力として無視
            else:
                self.get_logger().info(f"Stream ended. Decision: STAY (a={final_a:.4f} >= 0). No request sent.")
            
            # 次回のためにリセット
            self.latest_valid_peak = None
            self.latest_a = None

    def send_goal(self, target_x):
        """
        決定された目標座標を用いてAction Goalを送信する。
        """
        self.is_moving = True 
        goal_msg = StringCommand.Goal()
        goal_msg.command = f'pick_and_place_x {target_x:.2f}'
        self.get_logger().info(f'Sending goal: "{goal_msg.command}"')
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            self.is_moving = False
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_moving = False
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.answer}')
        self.is_moving = False

def main(args=None):
    rclpy.init(args=args)
    node = GoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()