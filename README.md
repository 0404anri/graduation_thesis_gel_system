# EAP Gel Robot Manipulation System

**卒業論文：EAPゲルのセンシング機能を用いたロボットマニピュレーションシステム**

本リポジトリは、2025年度卒業論文において開発した、EAP（Electro-Active Polymer）ゲルをセンサとして用いたロボットマニピュレーションシステムの制御プログラム一式である。

## 概要
本システムは、**Host PC**（画像処理・ロボット制御）と **Raspberry Pi 4**（ゲルへの電圧印加制御・ゲルの電流値取得）の2つの計算機が連携して動作する。
EAPゲルへの電圧印加によって生じるイオン分布変化を電流値として計測し、二次多項式フィッティングを用いて物体の把持位置を推定することで、EAPゲルを用いたロボットマニピュレーションを実現している。

## システム構成
本システムは以下のROS 2ノードによって構成され、各通信にはトピック・サービス・アクションを適切に使い分けている。

### 1. Host PC側 (Ubuntu 22.04)
- **`Task Manager`**:
  - ゲルへの電圧印加までのシステム全体の指揮を行うノード。
  - `Vision Server` から得た座標に基づき、物体が存在する領域（Place 1〜5）を判定し、`Gel Controller` へ指令を送る。
- **`Vision Server`**:
  - **OpenCVによるHSV色抽出**（オレンジ色）を行い、対象物体の3次元座標を特定する。
  - 誤作動防止のため、リクエスト時のみ撮影を行うアクション通信を採用している。
- **`Goal Generator`**:
  - EAPゲルの電流値データを解析し、ロボットアームの目標手先座標を決定する。
  - 正規化した電流値に対して**二次多項式フィッティング ($y=ax^2+bx+c$)** を行い、ピーク位置を算出する。
- **`Robot Controller`**:
  - `Goal Generator` からの指令に基づき、実機の制御を行う。
  - 軌道計画には **MoveIt 2**、グリッパ制御には **xArm Python SDK** を併用している。
- **`INA219 Plotter`**:
  - EAPゲルから取得した電流値をリアルタイムでプロットするノード。
- **`YOLOv8 Task Manager`** (YOLO版):
  - システム全体の指揮を行うノードのYOLOv8対応版。
- **`YOLOv8 Vision Server`** (YOLO版):
  - **YOLOv8による物体認識**を行い、対象物体の3次元座標を特定する。
  - 物体を追跡して常に3次元座標を更新し、トピック通信を用いてリアルタイムに配信する。

### 2. Raspberry Pi側 (Ubuntu 22.04)
- **`Gel Controller`**:
  - GPIO制御によるEAPゲルへの電圧印加（Stimulation）および I2C通信による電流値取得（Sensing）を担う。
  - 物体の位置（Place 1〜5）に応じて、Top / Mid / Bot の電極への印加パターンを動的に切り替える。
- **`Gel Controller YOLOv8`** (YOLO版):
  - YOLOv8システム用のGPIO制御およびセンシングノード。
 
## 動作環境

| 構成要素 | 使用デバイス |
| :--- | :--- |
| **Robot Arm** | UFACTORY xArm 6 |
| **Camera** | Intel RealSense Depth Camera D435i |
| **Sensor** | INA219 Current Sensor Module |
| **Computer** | Host PC, Raspberry Pi 4 Model B |

## ソフトウェア環境
- **OS**: Ubuntu 22.04 LTS
- **Middleware**: ROS 2 Humble Hawksbill
- **Language**: Python 3.10
- **Main Libraries**:
  - Realsense ROS Wrapper, MoveIt 2, OpenCV, xArm-Python-SDK, RPi.GPIO

## アルゴリズムの特徴

### 物体認識
深層学習モデルであるYOLOv8による物体認識と比較検討した結果、安全性とリアルタイム性を重視し、**OpenCVを用いたHSV色抽出手法**を採用した。特定色の領域抽出と深度情報の統合により、低計算コストで確実な座標特定を実現している。

### 目標座標決定
EAPゲルから得られる電流値は個体差や環境要因の影響を受けるため、以下の処理を行っている。
1. **正規化処理**: 予備実験で得た最大・最小値を用いて $0 \sim 1$ に正規化。
2. **フィッティング**: 電極位置（Top, Mid, Bot）を仮想座標系 ($x=9, 5, 1$) に配置し、二次多項式フィッティングを行うことで、物理的な電極が存在しない中間地点の座標も高精度に推定する。

## インストールと実行

### 1. ビルド
```bash
cd ~/ros2_ws/src
git clone https://github.com/0404anri/EAP_gel_robot_manipulation_system.git
rosdep install -i --from-path src --rosdistro humble -y
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. ネットワーク設定
Host PCとRaspberry Piの両方で以下を実行してください。

```bash
export ROS_DOMAIN_ID=0
```

### 3. 実行手順 (Execution Steps)
システムは以下の順序で起動する。

#### Step 1: Raspberry Pi側の起動
Raspberry Pi のターミナルで以下を実行し、待機状態にする。

```bash
ros2 run rpi_controller gel_controller
```

#### Step 2: Host PC側の起動
各ノードを別々のターミナルで起動する。

**Terminal 1: ロボットとMoveItの起動**
（xArmのIPアドレスは環境に合わせて変更してください）

```bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.xxx
```

**Terminal 2: ロボットコントローラの実行**

```bash
ros2 run from_gel robot_controller
```

**Terminal 3: 目標座標生成ノードの実行**

```bash
ros2 run from_gel goal_generator
```

**Terminal 4: カメラの起動**

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_gyro:=false enable_accel:=false
```

**Terminal 5: 色抽出・領域判定サーバの実行**

```bash
ros2 run to_gel vision_server
```

**Terminal 6: タスクマネージャの実行（メイン）**

```bash
ros2 run to_gel task_manager
```

**Terminal 7: 電流値プロット（オプション）**

```bash
ros2 run from_gel ina219_plotter
```

#### (オプション) YOLOv8版の実行
物体認識にYOLOv8を使用する場合は、`gel_controller`, `vision_server`, `task_manager` の代わりに以下を使用する。

**Raspberry Pi:**

```bash
ros2 run rpi_controller gel_controller_yolov8
```

**Host PC:**

```bash
# YOLOv8画像処理サーバ
ros2 run from_gel yolov8_vision_server

# YOLOv8対応タスクマネージャ
ros2 run from_gel yolov8_task_manager
```

## 著者 (Author)
- **袮屋 安里**
- 立命館大学 理工学部 ロボティクス学科

## License
This project is licensed under the MIT License.
