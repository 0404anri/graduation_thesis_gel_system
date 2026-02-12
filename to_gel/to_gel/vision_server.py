import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import threading
import time

from gel_interfaces.action import VisionCommand

"""
概要:
    RealSenseカメラを用いた物体検出・位置計測ノード

説明:
    Actionリクエスト受信時にのみ画像処理を有効化し、
    HSV色抽出と深度情報の統合によって対象物の3次元座標を特定する。
"""

# --- 画像処理パラメータ ---
# 検出対象（オレンジ色）のHSV閾値
HSV_LOWER = np.array([5, 120, 100]) 
HSV_UPPER = np.array([25, 255, 255])

# ノイズ除去設定
MIN_CONTOUR_AREA = 100  # 最小輪郭面積（これ以下のノイズは無視）
ROI_MARGIN = 0.2        # 画像端の除外割合（上下左右20%を無視して中央のみ処理）

class VisionServer(Node):
    """
    画像認識を行い、結果をアクションとして返すサーバーノード。
    """
    def __init__(self):
        super().__init__('vision_server')
        self.get_logger().info('ワンショット色認識サーバー起動: リクエスト受信後に撮影・終了します')

        self.target_name = 'orange'
        self.frame_id = 'target_obj'
        
        # --- 制御用の変数 ---
        self.capture_event = threading.Event() # 撮影完了を待機するイベント
        self.capture_requested = False         # 撮影要求フラグ
        self.detected_coords = None            # 検出結果 (x, y, z)
        self.latest_debug_img = None           # デバッグ表示用画像
        self.should_shutdown = False           # ノード終了フラグ

        self.lock = threading.Lock()

        # マルチスレッド実行のためのコールバックグループ
        self.callback_group = ReentrantCallbackGroup()
        
        # RealSenseトピックの購読設定
        # カラー画像、深度画像、カメラ情報を同時に扱うため message_filters を使用
        self.sub_info = Subscriber(
            self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
            callback_group=self.callback_group)
        self.sub_color = Subscriber(
            self, Image, '/camera/camera/color/image_raw',
            callback_group=self.callback_group)
        self.sub_depth = Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            callback_group=self.callback_group)
        
        # タイムスタンプの同期（許容誤差 0.1秒）
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)

        self.broadcaster = TransformBroadcaster(self)

        self.action_server = ActionServer(
            self,
            VisionCommand,
            'vision/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

    def cancel_callback(self, goal_handle):
        """アクションのキャンセル要求を受け付ける"""
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        アクション実行コールバック。
        画像処理スレッドに対して撮影要求を出し、結果が得られるまで待機する。
        """
        self.get_logger().info('リクエスト受信...')

        request = goal_handle.request
        result = VisionCommand.Result()

        cmd_parts = request.command.split(' ')
        command_type = cmd_parts[0]
        
        if command_type == 'capture':
            target_color = cmd_parts[1] if len(cmd_parts) > 1 else 'orange'
            self.get_logger().info(f'ターゲット: {target_color} を撮影して解析します')

            # 状態のリセット
            self.capture_event.clear()
            self.detected_coords = None
            self.target_name = target_color
            
            # 画像処理スレッドへ撮影を依頼
            with self.lock:
                self.capture_requested = True
            
            # 画像取得を待機 (最大3.0秒)
            is_set = self.capture_event.wait(timeout=3.0)

            if is_set and self.detected_coords is not None:
                x, y, z = self.detected_coords
                result.answer = f'OK x={x:.4f} y={y:.4f} z={z:.4f}'
                self.get_logger().info(f'成功: {result.answer}')
                goal_handle.succeed()
            else:
                msg = 'NG Timeout' if not is_set else 'NG Not Found'
                result.answer = msg
                self.get_logger().warn(msg)
                goal_handle.succeed() # 失敗時もActionとしてはSucceedで返し、メッセージで判定

            # ワンショット動作のため、処理完了後にノード終了フラグを立てる
            self.get_logger().info('処理完了。1秒後にシャットダウンします。')
            time.sleep(1.0) 
            self.should_shutdown = True

        else:
            result.answer = f'NG Unknown command: {command_type}'
            goal_handle.abort()

        return result

    def images_callback(self, msg_info, msg_color, msg_depth):
        """
        同期されたカメラデータを受信し、画像処理を行うコールバック。
        フラグが立っている場合のみ高負荷な画像処理を実行する。
        """
        with self.lock:
            if not self.capture_requested:
                return
            self.capture_requested = False

        try:
            bridge = CvBridge()
            img_color = bridge.imgmsg_to_cv2(msg_color, 'bgr8')
            img_depth = bridge.imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            self.capture_event.set()
            return

        # --- 画像処理（ROI切り出し） ---
        h_img, w_img = img_color.shape[:2]
        roi_x1 = int(w_img * ROI_MARGIN)
        roi_x2 = int(w_img * (1 - ROI_MARGIN))
        roi_y1 = int(h_img * ROI_MARGIN)
        roi_y2 = int(h_img * (1 - ROI_MARGIN))
        img_crop = img_color[roi_y1:roi_y2, roi_x1:roi_x2]
        
        # --- 色抽出 (HSV) ---
        hsv = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        
        # モルフォロジー演算（オープニング）によるノイズ除去
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 輪郭抽出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # 最大面積の輪郭を対象物とみなす
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                
                # ROI座標を元画像全体の座標に戻す
                bu1, bu2 = x + roi_x1, x + w + roi_x1
                bv1, bv2 = y + roi_y1, y + h + roi_y1
                u, v = int((bu1 + bu2) / 2), int((bv1 + bv2) / 2) # 重心画素 (u, v)
                
                # 深度値の取得（スケール補正含む）
                if img_depth.dtype == np.uint16:
                    depth_scale = 0.001 # mm -> meter conversion
                else:
                    depth_scale = 1.0
                
                # 中心付近5x5ピクセルの中央値を取得（欠損対策）
                d_roi = img_depth[v:v+5, u:u+5]
                depth_val = np.median(d_roi) if d_roi.size > 0 else 0
                
                if depth_val > 0:
                    z = depth_val * depth_scale
                    
                    # --- 座標変換 (Pixel -> Camera Coordinate) ---
                    # ピンホールカメラモデル: X = Z * (u - cx) / fx
                    fx = msg_info.k[0] # 焦点距離 x
                    fy = msg_info.k[4] # 焦点距離 y
                    cx = msg_info.k[2] # 光学中心 x
                    cy = msg_info.k[5] # 光学中心 y
                    
                    real_x = z * (u - cx) / fx
                    real_y = z * (v - cy) / fy
                    real_z = z
                    
                    self.detected_coords = (real_x, real_y, real_z)
                    
                    # デバッグ描画
                    cv2.rectangle(img_color, (bu1, bv1), (bu2, bv2), (0, 255, 0), 2)
                    cv2.putText(img_color, f'{real_z:.2f}m', (bu1, bv1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        self.latest_debug_img = img_color
        self.capture_event.set() # 処理完了通知

def main():
    """
    メイン関数: ノードの実行とOpenCVウィンドウの制御
    """
    rclpy.init()
    node = VisionServer()
    executor = MultiThreadedExecutor()
    
    # ROS 2の通信処理を別スレッドで実行（GUI描画をメインスレッドで行うため）
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if node.should_shutdown:
                break
            # デバッグ画像があれば表示
            if node.latest_debug_img is not None:
                cv2.imshow('OneShot Detection', node.latest_debug_img)
                cv2.waitKey(100)
                time.sleep(0.1)
            else:
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        pass
    
    cv2.destroyAllWindows()
    rclpy.try_shutdown()
    print("Program finished.")

if __name__ == '__main__':
    main()