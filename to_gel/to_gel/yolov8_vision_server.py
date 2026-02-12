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
from ultralytics import YOLO
import threading
import queue
import time

from gel_interfaces.action import StringCommand

"""
概要:
    YOLOv8とRealSenseを用いた物体位置推定ノード

説明:
    RGB画像に対してYOLOv8による物体検出を行い、検出されたバウンディングボックスに対応する
    深度画像の領域から対象物の3次元位置を特定する。
    高負荷な推論処理による遅延を防ぐため、フレーム間引き処理を導入している。
"""

class Yolov8VisionServer(Node):
    """
    YOLOv8推論エンジンを搭載したROS 2アクションサーバー。
    """
    def __init__(self):
        super().__init__('yolov8_vision_server')
        self.get_logger().info('画像認識サーバーを起動開始')

        # 状態管理フラグと変数
        self.running = False
        self.target_name = 'orange'
        self.frame_id = 'target'
        
        # 検出成功率判定用のカウンタ
        self.counter_total = 0
        self.counter_detect = 0
        
        # 画像表示用のキュー
        self.q_color = queue.Queue(maxsize=1)
        self.q_depth = queue.Queue(maxsize=1)
        
        # 排他制御用ロック
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.target_detection_lock = threading.Lock()
        
        # フレーム間引き用のカウンタ
        self.frame_count = 0 

        # マルチスレッド実行用コールバックグループ
        self.callback_group = ReentrantCallbackGroup()
        
        # RealSenseトピックの購読 (RGB, Depth, CameraInfo)
        self.sub_info = Subscriber(
            self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
            callback_group=self.callback_group)
        self.sub_color = Subscriber(
            self, Image, '/camera/camera/color/image_raw',
            callback_group=self.callback_group)
        self.sub_depth = Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            callback_group=self.callback_group)
        
        # タイムスタンプ同期 (許容誤差 0.1秒)
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)

        # TFブロードキャスター (RViz表示用)
        self.broadcaster = TransformBroadcaster(self)

        # アクションサーバー設定
        self.action_server = ActionServer(
            self,
            StringCommand,
            'vision/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group
        )

        # YOLOモデルのロード (OpenVINO最適化モデル)
        # 初回推論は時間がかかるため、ダミー画像でウォームアップを行う
        self.model = YOLO('/home/neya/yolov8n_openvino_model/', task='detect')
        dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
        results = self.model(dummy_image, verbose=False, imgsz=640)
        self.names = results[0].names.values()
        
        self.get_logger().info('画像認識サーバーを起動完了')
        
        # スレッド例外のフック（エラー落ち防止）
        threading.excepthook = lambda x: ()

    def handle_accepted_callback(self, goal_handle):
        """新しいゴールが来た場合、前のゴールを中止して新しいゴールを受け入れる"""
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前のゴールを中止')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        アクション実行コールバック。
        'find [name]', 'track [name]', 'stop' コマンドを処理する。
        """
        with self.execute_lock:
            self.get_logger().info('実行...')
            request: StringCommand.Goal = goal_handle.request
            result = StringCommand.Result()
            result.answer = 'NG'
            
            # --- 物体探索コマンド (find) ---
            if request.command.startswith('find'):
                name = request.command[4:].strip()
                if len(name) == 0:
                    result.answer = 'NG name required'
                    goal_handle.abort()
                elif name not in self.names:
                    result.answer = 'NG unknown name'
                    goal_handle.abort()
                else:
                    # 探索開始
                    with self.target_detection_lock:
                        self.target_name = name
                        self.running = True
                        self.counter_total = 0
                        self.counter_detect = 0
                    
                    # 3秒間計測を行う
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        if not goal_handle.is_active:
                            with self.target_detection_lock:
                                self.running = False
                            break
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            with self.target_detection_lock:
                                self.running = False
                            break
                        time.sleep(0.1)
                    
                    # 計測終了後の判定
                    if self.running:
                        with self.target_detection_lock:
                            counter_total = self.counter_total
                            counter_detect = self.counter_detect
                            self.running = False
                        
                        # 検出率が50%以上なら成功とみなす
                        if counter_total > 0 and counter_detect / counter_total >= 0.5:
                            result.answer = 'OK'
                            goal_handle.succeed()
                        else:
                            result.answer = 'NG not found'
                            goal_handle.succeed()
            
            # --- 物体追跡コマンド (track) ---
            elif request.command.startswith('track'):
                name = request.command[5:].strip()
                if len(name) == 0:
                    result.answer = 'NG name required'
                    goal_handle.abort()
                elif name not in self.names:
                    result.answer = 'NG unknown name'
                    goal_handle.abort()
                else:
                    with self.target_detection_lock:
                        self.target_name = name
                        self.running = True
                    result.answer = 'OK'
                    goal_handle.succeed()
            
            # --- 停止コマンド (stop) ---
            elif request.command.startswith('stop'):
                with self.target_detection_lock:
                    self.running = False
                result.answer = 'OK'
                goal_handle.succeed()
            else:
                result.answer = f'NG {request.command} not supported'
                goal_handle.abort()

            self.get_logger().info(f'answer: {result.answer}')
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT

    def images_callback(self, msg_info, msg_color, msg_depth):
        """
        画像処理コールバック。
        YOLO推論と深度情報の統合を行う。負荷分散のためフレーム間引きを実施。
        """
        try:
            img_color = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
            img_depth = CvBridge().imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        if img_color.shape[0:2] != img_depth.shape[0:2]:
            self.get_logger().warn('カラーと深度の画像サイズが異なる')
            return

        # 深度画像のスケール調整
        if img_depth.dtype == np.uint16:
            depth_scale = 1e-3
            img_depth_conversion = True
        elif img_depth.dtype == np.float32:
            depth_scale = 1
            img_depth_conversion = False
        else:
            self.get_logger().warn('深度画像の型に対応していない')
            return

        with self.target_detection_lock:
            target_name = self.target_name
            running = self.running

        boxes = []
        classes = []
        names = {}
        
        # 関心領域 (ROI) の設定
        h_img, w_img = img_color.shape[:2]
        roi_x1 = int(w_img * 0.3)
        roi_x2 = int(w_img * 0.8)
        roi_y1 = int(h_img * 0.0)
        roi_y2 = int(h_img * 0.8)

        # 画像のクロッピング（推論範囲の限定）
        img_crop = img_color[roi_y1:roi_y2, roi_x1:roi_x2]
        cv2.rectangle(img_color, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 0), 3)

        # --- フレーム間引き処理 ---
        self.frame_count += 1
        process_this_frame = (self.frame_count % 3 == 0) # 3フレームに1回だけ処理を実行

        if running and process_this_frame:
            # YOLO推論の実行 (信頼度閾値 40%)
            results = self.model(img_crop, verbose=False, conf=0.20)
            names = results[0].names
            boxes = results[0].boxes
            classes = results[0].boxes.cls
            
            # デバッグ用描画
            img_debug_crop = results[0].plot()
            img_color[roi_y1:roi_y2, roi_x1:roi_x2] = img_debug_crop
            
            with self.target_detection_lock:
                self.counter_total += 1

        if not self.q_color.full():
            self.q_color.put(img_color)

        # ターゲット物体の探索
        target_box = None
        for b, c in zip(boxes, classes):
            if names[int(c)] == target_name:
                target_box = b
                break
        
        depth = 0
        (bu1, bu2, bv1, bv2) = (0, 0, 0, 0)
        
        # 物体が見つかった場合の深度計算
        if target_box is not None:
            crop_x1, crop_y1, crop_x2, crop_y2 = [int(i) for i in target_box.xyxy.cpu().numpy()[0]]
            
            # ROI座標を全体座標に戻す
            bu1 = crop_x1 + roi_x1
            bu2 = crop_x2 + roi_x1
            bv1 = crop_y1 + roi_y1
            bv2 = crop_y2 + roi_y1
            
            # バウンディングボックス中央付近の領域(50%)のみを使って深度を計算
            a = 0.5
            u1 = round((bu1 + bu2) / 2 - (bu2 - bu1) * a / 2)
            u2 = round((bu1 + bu2) / 2 + (bu2 - bu1) * a / 2)
            v1 = round((bv1 + bv2) / 2 - (bv2 - bv1) * a / 2)
            v2 = round((bv1 + bv2) / 2 + (bv2 - bv1) * a / 2)
            
            u1 = max(0, u1); v1 = max(0, v1)
            u2 = min(w_img, u2); v2 = min(h_img, v2)

            u = round((bu1 + bu2) / 2)
            v = round((bv1 + bv2) / 2)
            
            if v2 > v1 and u2 > u1:
                depth = np.median(img_depth[v1:v2+1, u1:u2+1])
            
            # 3次元座標変換 (Pinhole Camera Model)
            if depth != 0:
                z = float(depth) * depth_scale
                fx = msg_info.k[0]
                fy = msg_info.k[4]
                cx = msg_info.k[2]
                cy = msg_info.k[5]
                x = z / fx * (u - cx)
                y = z / fy * (v - cy)
                
                self.get_logger().info(f'{target_name} ({x:.3f}, {y:.3f}, {z:.3f})')
                
                # TF配信
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                self.broadcaster.sendTransform(ts)
                
                with self.target_detection_lock:
                    self.counter_detect += 1

        # デバッグ表示用（Depth画像に矩形を描画）
        if img_depth_conversion:
            img_depth *= 16
        if depth != 0:
            pt1 = (int(bu1), int(bv1))
            pt2 = (int(bu2), int(bv2))
            cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)
        
        if not self.q_depth.full():
            self.q_depth.put(img_depth)


def main():
    """
    メイン関数: ノード実行およびOpenCVデバッグウィンドウの表示
    """
    rclpy.init()
    node = Yolov8VisionServer()
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    try:
        while True:
            if not node.q_color.empty():
                cv2.imshow('object_detection color', node.q_color.get())
            if not node.q_depth.empty():
                cv2.imshow('object_detection depth', node.q_depth.get())
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
