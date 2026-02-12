import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import sys # 終了用にインポート

# --- INA219用インポート ---
from std_msgs.msg import Float32MultiArray
import board
import busio
from adafruit_ina219 import INA219

# --- インターフェース ---
from gel_interfaces.srv import StimulationCommand

# --- 設定項目 (GPIO) ---
ACTIVE_STATE = GPIO.LOW
INACTIVE_STATE = GPIO.HIGH

# 刺激を行う時間 (秒)
ACTIVE_INTERVAL = 0.15
# Publishを停止する時間 (刺激開始からの経過時間)
# "ACTIVE_INTERVAL - 1秒" で停止します (5.0 - 1.0 = 4.0秒)
PUBLISH_DURATION = max(0.0, ACTIVE_INTERVAL)

ELECTRODE_PAIRS = {
    "blue":   {"pog_pol": 17, "pog_enb":27, "neg_pol": 22, "neg_enb":10},
    "green":  {"pog_pol": 9, "pog_enb":11, "neg_pol": 20, "neg_enb":21},
    "orange": {"pog_pol": 14, "pog_enb":15, "neg_pol": 23, "neg_enb":24},
}

TARGET_MAPPING = {
    1: ["blue"],
    2: ["blue", "green"], 
    3: ["green"],
    4: ["green", "orange"],
    5: ["orange"]
}

SORTED_COLORS = sorted(ELECTRODE_PAIRS.keys())

# --- 設定項目 (INA219) ---
INA219_ADDRESSES = [0x40, 0x41, 0x44]

class GelController(Node):
    """
    起動時から電流値をPublishし、コマンド受信で刺激開始。
    刺激より1秒早くPublishを停止し、刺激終了後に自動シャットダウンするノード。
    """
    def __init__(self):
        super().__init__('gel_controller')

        # タイマーハンドルの初期化
        self.stim_off_timer = None   # 電極OFF用
        self.pub_off_timer = None    # Publish停止用
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

        # ★プログラム実行開始直後から電流計測をスタート
        self.start_publishing()

        # ==========================================
        # 2. 電極制御 (Service) の初期化
        # ==========================================
        self.srv = self.create_service(
            StimulationCommand,
            'stimulation/command',
            self.service_callback
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
            self.get_logger().info(f"初期化完了: ACTIVE={ACTIVE_INTERVAL}s, PUB_DUR={PUBLISH_DURATION}s")

        except Exception as e:
            self.get_logger().fatal(f"GPIOの初期化中に致命的なエラー: {e}")
            if rclpy.ok():
                rclpy.shutdown()

    # --- INA219 計測制御 ---

    def start_publishing(self):
        """電流計測タイマーを開始する"""
        if self.data_timer is None:
            # 0.1秒ごとに timer_callback を呼ぶ
            self.data_timer = self.create_timer(0.1, self.timer_callback)
            self.get_logger().info("計測開始 (Publish Start)")

    def stop_publishing(self):
        """電流計測タイマーを停止・破棄する"""
        if self.data_timer is not None:
            self.data_timer.cancel()
            self.data_timer.destroy()
            self.data_timer = None
            self.get_logger().info("計測終了 (Publish Stop)")

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

    def service_callback(self, request, response):
        command_str = request.command
        self.get_logger().info(f'サービスリクエスト受信: "{command_str}"')
        
        try:
            cmd_id = int(command_str.strip())
            
            if cmd_id in TARGET_MAPPING:
                # -----------------------------------------------
                # 1. 計測(Publish)を確実にONにする
                # -----------------------------------------------
                self.start_publishing()

                # -----------------------------------------------
                # 2. 指定された電極をONにする
                # -----------------------------------------------
                target_colors = TARGET_MAPPING[cmd_id]
                self.activate_pattern(target_colors)
                
                # -----------------------------------------------
                # 3. タイマーセット
                # -----------------------------------------------
                self.cancel_timers()

                # A: Publish停止 (4.0秒後)
                self.pub_off_timer = self.create_timer(PUBLISH_DURATION, self.stop_publishing_callback)
                
                # B: 電極OFF (5.0秒後) -> ここでシャットダウンも行います
                self.stim_off_timer = self.create_timer(ACTIVE_INTERVAL, self.stop_stimulation_callback)
                
                response.answer = f"OK: Pattern {cmd_id} (Stim:{ACTIVE_INTERVAL}s, Pub:{PUBLISH_DURATION}s)"
            else:
                self.get_logger().warn(f"未定義のコマンドID: {cmd_id}")
                response.answer = f"Error: Undefined command {cmd_id}"

        except ValueError:
            self.get_logger().error("数値変換エラー")
            response.answer = "Error: Command must be integer string"
            
        return response

    def activate_pattern(self, target_list):
        """指定パターンをONにする"""
        target_str = ",".join(target_list)
        for color in SORTED_COLORS:
            pins = ELECTRODE_PAIRS[color]
            # ※注: GPIOロジックはご提示いただいた内容に従っています
            if color in target_str:
                GPIO.output(pins['pog_pol'], ACTIVE_STATE)
                GPIO.output(pins['pog_enb'], ACTIVE_STATE)
                GPIO.output(pins['neg_pol'], INACTIVE_STATE)
                GPIO.output(pins['neg_enb'], ACTIVE_STATE)
                self.get_logger().info(f" -> {color}: ON")
            else:
                GPIO.output(pins['pog_pol'], INACTIVE_STATE)
                GPIO.output(pins['pog_enb'], INACTIVE_STATE)
                GPIO.output(pins['neg_pol'], INACTIVE_STATE)
                GPIO.output(pins['neg_enb'], INACTIVE_STATE)

    def stop_publishing_callback(self):
        """[タイマーA] 時間経過(4s)でPublish停止"""
        self.get_logger().info(f"計測時間終了({PUBLISH_DURATION}s): Publish停止")
        self.stop_publishing()
        # タイマー破棄
        if self.pub_off_timer:
            self.pub_off_timer.cancel()
            self.pub_off_timer.destroy()
            self.pub_off_timer = None

    def stop_stimulation_callback(self):
        """[タイマーB] 時間経過(5s)で電極OFF し、ノードを終了する"""
        self.get_logger().info(f"刺激時間終了({ACTIVE_INTERVAL}s): 電極OFF")
        self.set_all_electrodes_inactive()
        
        # タイマー破棄
        if self.stim_off_timer:
            self.stim_off_timer.cancel()
            self.stim_off_timer.destroy()
            self.stim_off_timer = None
            
        # --- 自動シャットダウン処理 ---
        self.get_logger().info("全プロセス完了: 自動シャットダウンを実行します。")
        raise SystemExit # ここで例外を発生させてmainループを抜ける

    def cancel_timers(self):
        """待機中のタイマーを安全にキャンセル"""
        if self.stim_off_timer:
            self.stim_off_timer.cancel()
            self.stim_off_timer.destroy()
            self.stim_off_timer = None
        if self.pub_off_timer:
            self.pub_off_timer.cancel()
            self.pub_off_timer.destroy()
            self.pub_off_timer = None

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
        self.set_all_electrodes_inactive()
        self.stop_publishing()
        self.cancel_timers()
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
    
    except SystemExit:
        # stop_stimulation_callback で raise SystemExit された場合にここに来ます
        # 正常終了として扱います
        pass

    except KeyboardInterrupt:
        print("\nCtrl+C 検出。停止処理...")
        if node:
            node.set_all_electrodes_inactive()
            node.stop_publishing()
    except Exception as e:
        print(f"予期せぬエラー: {e}")
        if node:
            node.set_all_electrodes_inactive()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("終了しました。")

if __name__ == '__main__':
    main()
