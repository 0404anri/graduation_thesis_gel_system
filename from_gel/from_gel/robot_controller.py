import sys
import os
import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from gel_interfaces.action import StringCommand
from pymoveit2 import MoveIt2

"""
概要:
    xArm6 ロボットアーム制御用アクションサーバー

説明:
    受信した目標位置（X座標）に基づき、MoveIt 2を用いたアーム移動と
    SDKを用いたグリッパー制御を組み合わせて、Pick-and-Place動作を実行する。
"""

# Studioと同じ通信経路(SDK)を直接使うためのライブラリ
# ROS 2トピック経由ではなく、TCP/IPソケットで直接コマンドを送るために使用
try:
    from xarm.wrapper import XArmAPI
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("xArm-Python-SDK is not installed. Please run: pip install xarm-python-sdk")

# --- ロボット設定パラメータ ---
ROBOT_IP = '192.168.1.242'  # ロボットコントローラのIPアドレス
GRIPPER_SPEED = 3000        # グリッパーの開閉速度
GRIPPER_OPEN_POS = 850      # グリッパー全開位置
GRIPPER_CLOSE_POS = 0       # グリッパー全閉位置

# 動作範囲の定義 (メートル)
WORKSPACE_MIN_X = 0.282     # 手前側の把持限界
WORKSPACE_MAX_X = 0.507     # 奥側の把持限界

class RobotController(Node):
    """
    Pick-and-Place動作を管理・実行するROS 2ノード。
    MoveIt 2による軌道計画と、SDKによるエンドエフェクタ制御を統合する。
    """
    def __init__(self):
        super().__init__('robot_controller')
        
        # アクション実行中に別のコールバック（MoveItのフィードバック等）を並行処理するためにReentrantCallbackGroupを使用
        self.cb_group_main = ReentrantCallbackGroup()
        
        # MoveItの初期化 (アームの軌道計画用)
        self.moveit2 = MoveIt2(
            node=self,
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            base_link_name = 'link_base', 
            end_effector_name = 'link_tcp', 
            group_name = 'xarm6',
            callback_group = self.cb_group_main, 
        )

        # SDKを直接初期化 (グリッパーの開閉制御用)
        self.arm_ip = ROBOT_IP
        if SDK_AVAILABLE:
            self.get_logger().info(f'Connecting directly to xArm at {self.arm_ip} for Gripper control...')
            try:
                self.arm = XArmAPI(self.arm_ip)
                self.arm.connect()
                self.arm.set_gripper_enable(True)
                self.arm.set_gripper_mode(0) # 0: 位置制御モード
                self.arm.set_gripper_speed(GRIPPER_SPEED)
                self.get_logger().info('Direct Gripper Control: READY')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to SDK: {e}')
        
        # 規定の姿勢定義 [x, y, z, roll, pitch, yaw]
        self.poses = {
            'home_point': [0.300, 0.255, 0.050, 3.14, 0.0, 0.0], # 待機位置
            'drop_point': [0.300, 0.000, 0.052, 3.14, 0.0, 0.0]  # 配置位置
        }
       
        # アクションサーバーの起動
        self.action_server = ActionServer(
            self, StringCommand, 'manipulation/command',
            execute_callback = self.execute_callback, 
            callback_group = self.cb_group_main 
        )

    def control_gripper(self, open_gripper: bool):
        """
        xArm SDKを用いてグリッパーを直接制御する。
        
        Args:
            open_gripper (bool): Trueなら開く、Falseなら閉じる
        """
        if not SDK_AVAILABLE:
            return
        
        pos = GRIPPER_OPEN_POS if open_gripper else GRIPPER_CLOSE_POS
        self.get_logger().info(f'SDK Command: Gripper -> {pos}')
        
        # wait=Falseにすることで、ROS側の処理ブロックを防ぐ
        self.arm.set_gripper_position(pos, wait=False)
        time.sleep(0.1) # 通信安定のための短い待機

    def execute_callback(self, goal_handle):
        """
        アクションリクエスト受信時に実行されるコールバック関数。
        コマンドを解析し、一連のPick-and-Place動作を開始する。
        """
        command_str = goal_handle.request.command
        self.get_logger().info(f'Executing command: {command_str}')
        
        words = command_str.split()
        if words[0] == 'pick_and_place_x':
            # 引数がある場合は取得、なければデフォルト値を使用
            val = float(words[1]) if len(words) > 1 else 5.0
            self.pick_and_place_sequence(val)
        
        goal_handle.succeed()
        result = StringCommand.Result()
        result.answer = 'OK'
        return result

    def move_ee(self, p):
        """
        指定された姿勢（位置・オイラー角）へアームを移動させる。
        
        Args:
            p (list): [x, y, z, roll, pitch, yaw] 形式の目標姿勢
        """
        self.moveit2.move_to_pose(
            position=p[:3], 
            quat_xyzw=self.euler_to_quat(p[3:]), 
            target_link='link_tcp', 
            frame_id='link_base'
        )
        # 移動完了を待機
        success = self.moveit2.wait_until_executed()
        time.sleep(0.05)
        return success

    def euler_to_quat(self, rpy):
        """オイラー角(Roll, Pitch, Yaw)をクォータニオン(x, y, z, w)に変換する"""
        roll, pitch, yaw = rpy
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]

    def pick_and_place_sequence(self, val):
        """
        入力値に基づき、対象物の把持・搬送・配置を行う一連のシーケンス。
        
        Args:
            val (float): 0.0〜10.0の範囲で指定される目標位置の指標
        """
        # 入力値を0.0-1.0に正規化
        ratio = max(0.0, min(val / 10.0, 1.0))
        
        # 線形補間により物理座標(X軸)を計算
        tx = WORKSPACE_MIN_X + (WORKSPACE_MAX_X - WORKSPACE_MIN_X) * ratio
        
        self.get_logger().info(f'Starting sequence at X={tx:.3f} (Input: {val})')

        # 1. 準備 (開く & ホームへ移動)
        self.control_gripper(True)
        self.move_ee(self.poses['home_point'])
        
        # 2. ピック位置の計算 (アプローチ位置と把持位置)
        pick = [tx, 0.0, 0.002, 3.14, 0.0, 0.0]     # 把持高さ
        app_pick = [tx, 0.0, 0.052, 3.14, 0.0, 0.0] # アプローチ高さ (+5cm)
        
        # 3. ピック動作 (アプローチ -> 降下)
        self.move_ee(app_pick)
        self.move_ee(pick)
        
        # 4. 掴む (SDK直接制御)
        self.control_gripper(False)
        time.sleep(1.2) # グリッパーが閉じる物理的な時間待機
        
        # 5. 配置場所へ移動 (上昇 -> ドロップ位置へ)
        self.move_ee(app_pick)
        self.move_ee(self.poses['drop_point'])
        
        # 6. 離す
        self.control_gripper(True)
        time.sleep(0.8)
        
        # 7. 完了 (アプローチ位置へ退避 -> ホームへ戻る)
        app_drop = list(self.poses['drop_point']); app_drop[2] += 0.05
        self.move_ee(app_drop)
        self.move_ee(self.poses['home_point'])

def main():
    """
    メイン関数: ノードの初期化とマルチスレッドExecutorの実行
    """
    rclpy.init()
    node = RobotController()
    
    # MoveItのアクションと自身のActionServerを並列処理するためにMultiThreadedExecutorを使用
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()