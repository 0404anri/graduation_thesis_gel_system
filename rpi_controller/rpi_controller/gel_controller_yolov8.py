import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import sys

# --- ROSメッセージ用インポート ---
from std_msgs.msg import Float32MultiArray, String

# --- INA219用インポート ---
import board
import busio
from adafruit_ina219 import INA219

# --- 設定項目 (GPIO) ---
ACTIVE_STATE = GPIO.LOW
INACTIVE_STATE = GPIO.HIGH

# ★変更点: パルス幅ではなく「タイムアウト時間」を定義
# 送信側が0.5秒周期なので、1.0秒間コマンドが来なければ通信断絶とみなして停止する
WATCHDOG_TIMEOUT = 1.0 

ELECTRODE_PAIRS = {
    "blue":   {"pog_pol": 17, "pog_enb":27, "neg_pol": 22, "neg_enb":10},
    "green":  {"pog_pol": 9, "pog_enb":11, "neg_pol": 20, "neg_enb":21},
    "orange": {"pog_pol": 14, "pog_enb":15, "neg_pol": 23, "neg_enb":24},
}

SORTED_COLORS = sorted(ELECTRODE_PAIRS.keys())

# --- 設定項目 (INA219) ---
INA219_ADDRESSES = [0x40, 0x41, 0x44]

class GelController(Node):
    """
    Topic 'stimulation_command' を受信している間、刺激を継続するノード。
    コマンドが途切れて WATCHDOG_TIMEOUT 秒経過すると自動停止する。
    """
    def __init__(self):
        super().__init__('gel_controller')

        # タイマーハンドルの初期化
        self.watchdog_timer = None   # ★安全停止用タイマー
        self.data_timer = None       # 電流計測ループ用

        # ==========================================
        # 1. 電流センサ (Publisher) の準備
        # ==========================================
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ina219_current_data', 10)
        
        self.get_logger().info("I2Cセンサ(INA219)の接続確認中...")
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.sensors = []
            for addr in INA219_ADDRESSES:
                try:
                    sensor = INA219(self.i2c, addr)
                    self.sensors.append(sensor)
                except ValueError:
                    self.get_logger().warn(f'センサーが見つかりません: {hex(addr)}')
                    self.sensors.append(None)
        except Exception as e:
            self.get_logger().error(f'I2Cバスのエラー: {e}')
            self.sensors = [None, None, None]

        # ==========================================
        # 2. 電極制御 (Subscriber) の初期化
        # ==========================================
        self.subscription = self.create_subscription(
            String,
            'stimulation_command',
            self.command_callback,
            10
        )
        
        self.get_logger().info("GPIOピンの初期化を開始します...")
        try:
            GPIO.setmode(GPIO.BCM)
            for color, pins in ELECTRODE_PAIRS.items():
                GPIO.setup(pins['pog_pol'], GPIO.OUT)
                GPIO.setup(pins['pog_enb'], GPIO.OUT)
                GPIO.setup(pins['neg_pol'], GPIO.OUT)
                GPIO.setup(pins['neg_enb'], GPIO.OUT)
            
            self.set_all_electrodes_inactive()
            self.get_logger().info(f"待機中: 'stimulation_command'を受信し続ける限りONになります")
            self.get_logger().info(f"安全装置: {WATCHDOG_TIMEOUT}秒間信号がないと停止します")

        except Exception as e:
            self.get_logger().fatal(f"GPIOの初期化中に致命的なエラー: {e}")
            if rclpy.ok():
                rclpy.shutdown()

    # --- INA219 計測制御 ---

    def start_publishing(self):
        """電流計測タイマーを開始する（既に動いていれば何もしない）"""
        if self.data_timer is None:
            self.data_timer = self.create_timer(0.1, self.timer_callback)
            # self.get_logger().info("計測開始")

    def stop_publishing(self):
        """電流計測タイマーを停止・破棄する"""
        if self.data_timer is not None:
            self.data_timer.cancel()
            self.data_timer.destroy()
            self.data_timer = None
            # self.get_logger().info("計測停止")

    def timer_callback(self):
        """定期的にセンサを読み取りPublishする"""
        readings = []
        for sensor in self.sensors:
            if sensor:
                try:
                    readings.append(sensor.current)
                except:
                    readings.append(0.0)
            else:
                readings.append(0.0)

        msg = Float32MultiArray()
        msg.data = readings 
        self.publisher_.publish(msg)

    # --- 電極制御 関連メソッド ---

    def command_callback(self, msg):
        """
        トピックを受信したときのコールバック
        継続的に受信することで、watchdog_timerがリセットされ、ON状態が維持される
        """
        command_str = msg.data.strip()
        
        try:
            # カンマ区切りで分解
            parts = command_str.split(',')
            
            # 要素数が足りない場合は無視
            if len(parts) != len(SORTED_COLORS):
                return

            # 全て "0" ("0,0,0") なら即時停止
            if all(p == '0' for p in parts):
                self.force_stop()
                return

            # --- ここから「刺激ON」の処理 ---

            # 1. 指定された電極パターンをONにする
            target_list = []
            for i, val in enumerate(parts):
                if val == '1':
                    target_list.append(SORTED_COLORS[i])
            
            self.activate_pattern(target_list)
            self.start_publishing() # 計測も開始

            # 2. ウォッチドッグタイマーをリセット（延長）する
            # 既存の停止タイマーがあればキャンセルして、新しくセットし直す
            self.reset_watchdog()

        except Exception as e:
            self.get_logger().error(f"コマンド処理エラー: {e}")

    def reset_watchdog(self):
        """安全停止タイマーをリセット（時間を巻き戻す）"""
        if self.watchdog_timer:
            self.watchdog_timer.cancel()
            self.watchdog_timer.destroy()
        
        # WATCHDOG_TIMEOUT秒後に force_stop を呼ぶタイマーをセット
        self.watchdog_timer = self.create_timer(WATCHDOG_TIMEOUT, self.on_watchdog_timeout)

    def on_watchdog_timeout(self):
        """[安全装置] コマンドが一定時間来なかった場合に呼ばれる"""
        self.get_logger().warn(f"通信断絶: {WATCHDOG_TIMEOUT}秒間コマンドなし。緊急停止します。")
        self.force_stop()

    def force_stop(self):
        """刺激と計測を直ちに停止する"""
        self.set_all_electrodes_inactive()
        self.stop_publishing()
        
        # タイマーもクリア
        if self.watchdog_timer:
            self.watchdog_timer.cancel()
            self.watchdog_timer.destroy()
            self.watchdog_timer = None

    def activate_pattern(self, target_list):
        """指定パターンをONにする（状態維持）"""
        for color in SORTED_COLORS:
            pins = ELECTRODE_PAIRS[color]
            
            if color in target_list:
                GPIO.output(pins['pog_pol'], ACTIVE_STATE)
                GPIO.output(pins['pog_enb'], ACTIVE_STATE)
                GPIO.output(pins['neg_pol'], INACTIVE_STATE)
                GPIO.output(pins['neg_enb'], ACTIVE_STATE)
            else:
                GPIO.output(pins['pog_pol'], INACTIVE_STATE)
                GPIO.output(pins['pog_enb'], INACTIVE_STATE)
                GPIO.output(pins['neg_pol'], INACTIVE_STATE)
                GPIO.output(pins['neg_enb'], INACTIVE_STATE)

    def set_all_electrodes_inactive(self):
        try:
            for pins in ELECTRODE_PAIRS.values():
                GPIO.output(pins['pog_pol'], INACTIVE_STATE)
                GPIO.output(pins['pog_enb'], INACTIVE_STATE)
                GPIO.output(pins['neg_pol'], INACTIVE_STATE)
                GPIO.output(pins['neg_enb'], INACTIVE_STATE)
        except Exception as e:
            self.get_logger().error(f"停止処理中にエラー: {e}")

    def destroy_node(self):
        self.get_logger().info("ノード終了処理...")
        self.force_stop()
        try:
            GPIO.cleanup()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GelController()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        print("\nCtrl+C 検出。停止処理...")
    except Exception as e:
        print(f"予期せぬエラー: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("終了しました。")

if __name__ == '__main__':
    main()