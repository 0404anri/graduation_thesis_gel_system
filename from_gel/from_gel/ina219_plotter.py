import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import time
from collections import deque
import datetime 
import os  # 追加: ディレクトリ操作用

# グラフに表示するデータ点数を制限
MAX_DATA_POINTS = 200

class INA219Plotter(Node):
    """
    '/ina219_current_data'トピックをサブスクライブし、
    データをリアルタイムでグラフにプロット、ファイルに保存するクラス。
    """
    def __init__(self):
        super().__init__('ina219_plotter')

        # Publisherに合わせてQoSをシンプルに「10」に設定
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ina219_current_data',
            self.listener_callback,
            10)

        # 固定長のキューを使ってデータを保存
        self.time_data = deque(maxlen=MAX_DATA_POINTS)
        self.current_data_top = deque(maxlen=MAX_DATA_POINTS)
        self.current_data_middle = deque(maxlen=MAX_DATA_POINTS)
        self.current_data_bottom = deque(maxlen=MAX_DATA_POINTS)
        
        self.start_time = time.time()

        # --- ログファイルの設定 ---
        
        # 保存先ディレクトリ名
        data_dir = 'raw_current_data'
        
        # ディレクトリが存在しない場合は作成する
        if not os.path.exists(data_dir):
            try:
                os.makedirs(data_dir)
                self.get_logger().info(f"ディレクトリを作成しました: {data_dir}")
            except OSError as e:
                self.get_logger().error(f"ディレクトリ作成失敗: {e}")

        # ファイル名の生成
        now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_filename = f'current_log_{now_str}.csv'
        
        # ディレクトリとファイル名を結合してフルパスを作成
        full_path = os.path.join(data_dir, log_filename)
        
        try:
            # 結合したパスでファイルを開く
            self.log_file = open(full_path, 'w')
            
            # ヘッダー書き込み
            self.log_file.write('elapsed_time,top_mA,middle_mA,bottom_mA\n')
            self.get_logger().info(f"Logging data to {full_path}")
            
        except IOError as e:
            self.get_logger().error(f"Failed to open log file: {e}")
            self.log_file = None

        # --- Matplotlibの設定 ---
        plt.ion() # インタラクティブモード有効化
        self.fig, self.ax = plt.subplots(figsize=(10, 6))

        # プロットの初期化
        self.line1, = self.ax.plot([], [], color='black', label='Top (0x40)')
        self.line2, = self.ax.plot([], [], color='brown', label='Middle (0x41)')
        self.line3, = self.ax.plot([], [], color='red', label='Bottom (0x44)')

        self.ax.set_title('Real-time INA219 Current Readings')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Current (mA)')
        self.ax.legend(loc='upper right')
        self.ax.grid(True)
        
        self.get_logger().info('Real-time plotter node has started.')

    def listener_callback(self, msg):
        """メッセージ受信時の処理"""
        elapsed_time = time.time() - self.start_time
        data = msg.data
        
        # データが足りない場合の安全策
        val_top = data[0] if len(data) > 0 else 0.0
        val_mid = data[1] if len(data) > 1 else 0.0
        val_bot = data[2] if len(data) > 2 else 0.0

        # --- ファイル保存 ---
        if self.log_file:
            log_line = f"{elapsed_time:.3f},{val_top},{val_mid},{val_bot}\n"
            self.log_file.write(log_line)
            self.log_file.flush() # 即時書き込み

        # --- グラフ用データ更新 ---
        self.time_data.append(elapsed_time)
        self.current_data_top.append(val_top)
        self.current_data_middle.append(val_mid)
        self.current_data_bottom.append(val_bot)
        
        # グラフデータのセット
        self.line1.set_data(self.time_data, self.current_data_top)
        self.line2.set_data(self.time_data, self.current_data_middle)
        self.line3.set_data(self.time_data, self.current_data_bottom)

        self.ax.relim()
        self.ax.autoscale_view()

    def is_plot_closed(self):
        """プロットウィンドウが閉じられたかチェック"""
        return not plt.fignum_exists(self.fig.number)

    def cleanup(self):
        """終了処理"""
        if self.log_file and not self.log_file.closed:
            self.log_file.close()
            self.get_logger().info('Log file closed correctly.')
        plt.close('all')

def main(args=None):
    rclpy.init(args=args)
    plotter_node = INA219Plotter()

    try:
        while rclpy.ok():
            # GUI更新のためにタイムアウト付きでspin
            rclpy.spin_once(plotter_node, timeout_sec=0.01)
            
            if plotter_node.is_plot_closed():
                break
            
            # GUIイベント処理（フリーズ防止）
            plt.pause(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        plotter_node.cleanup()
        plotter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()