# 多传感器融合定位系统 - 方法指导文档

## 目录
1. [系统概述](#1-系统概述)
2. [架构设计](#2-架构设计)
3. [环境配置](#3-环境配置)
4. [模块说明](#4-模块说明)
5. [使用指南](#5-使用指南)
6. [ROS robot_localization方案](#6-ros-robot_localization方案) ⭐推荐
7. [算法原理](#7-算法原理)
8. [Python EKF vs ROS robot_localization对比](#8-python-ekf-vs-ros-robot_localization对比)
9. [常见问题](#9-常见问题)

---

## 1. 系统概述

### 1.1 任务描述

本系统实现了一个完整的**多传感器融合定位**方案，专为月球车仿真环境设计：

- **输入传感器**：
  - 双目导航相机（高处，广视野）
  - 双目避障相机（低处，窄视野）
  - IMU（惯性测量单元）
  - 模拟星敏感器（姿态测量）
  - 模拟轮速计（速度测量）

- **输出**：融合后的高精度位姿估计（位置 + 姿态）

### 1.2 技术路线

```
┌─────────────────────────────────────────────────────────────────────┐
│                        数据采集层 (UE4 + AirSim)                     │
├─────────────────────────────────────────────────────────────────────┤
│  导航相机 │ 避障相机 │ IMU（0.02°） │ 星敏感器(模拟) │ 圈数计(模拟) │真值│ gyro 0.0001°
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      数据存储层 (dataset_saver.py)                   │
├─────────────────────────────────────────────────────────────────────┤
│            EuRoC格式 (CSV + 图像)  │  ROS Bag格式                    │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
┌──────────────────────────——————──┐     ┌────────────────────────────┐          模拟器：太阳高度角6.5°，AirSim截图图像增强
│   VINS-Fusion 调datarate         │     │       EKF 融合器            │
│   -------------------------      │ →→  │   -------------------------│   星敏（10Hz）+ 圈数计（准确）（10Hz）  global map with label/local map匹配
│   输入: 避障相机 + IMU            │     │   输入: VINS输出            │              ↓                                ↓
│   输出: 视觉惯性里程计             │     │   输出: 最终融合位姿        │ ←← dead reckoning （14% drift）              避障
└─────────────────────────────—————┘     └────────────────────────────┘
                                                     │
                                 ┬───────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                          评估与可视化                                │
├─────────────────────────────────────────────────────────────────────┤
│      ATE (绝对轨迹误差) │ RPE (相对位姿误差) │ 轨迹可视化            │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. 架构设计

### 2.1 模块结构

```
LunarSimulator/
├── ros_fusion_pipeline.py      # ROS robot_localization方案 ★推荐
├── fusion_pipeline.py          # Python EKF方案（备选）
├── ekf_fusion.py               # Python EKF实现（备选）
├── vins_docker_interface.py    # VINS-Docker自动化接口
├── dataset_saver.py            # 数据集保存工具
├── config.py                   # 配置参数
├── bag_to_csv.py               # ROS bag转CSV工具
└── Documents/
    └── Multi_Sensor_Fusion_Guide.md  # 本文档
```

### 2.2 数据流

```
            ┌──────────────────────────────────────────────────────────┐
            │                    fusion_pipeline.py                     │
            │  ┌─────────────────────────────────────────────────────┐ │
            │  │                                                     │ │
Input ──────┼──►  RosbagDataLoader / EurocDataLoader                 │ │
(bag/euroc) │  │         │                                           │ │
            │  │         ▼                                           │ │
            │  │  ┌─────────────────┐                                │ │
            │  │  │ VINSDockerInterface ─────► VINS轨迹              │ │
            │  │  └─────────────────┘              │                 │ │
            │  │                                   │                 │ │
            │  │  IMU + 星敏 + 轮速计              │                 │ │
            │  │         │                         │                 │ │
            │  │         ▼                         ▼                 │ │
            │  │  ┌─────────────────────────────────────────┐        │ │
            │  │  │            MultiSensorEKF               │        │ │
            │  │  │  ┌─────────┐ ┌───────────┐ ┌─────────┐ │        │ │
            │  │  │  │ predict │ │update_vins│ │update_* │ │        │ │
            │  │  │  │  (IMU)  │ │           │ │         │ │        │ │
            │  │  │  └─────────┘ └───────────┘ └─────────┘ │        │ │
            │  │  └─────────────────────────────────────────┘        │ │
            │  │                      │                              │ │
            │  │                      ▼                              │ │
            │  │              融合位姿输出 ──────► 评估 & 可视化      │ │
            │  └─────────────────────────────────────────────────────┘ │
            └──────────────────────────────────────────────────────────┘
```

---

## 3. 环境配置

### 3.1 Python环境

```bash
# 创建虚拟环境（推荐）
python -m venv venv
source venv/bin/activate  # Linux/Mac
# 或 venv\Scripts\activate  # Windows

# 安装依赖
pip install numpy scipy matplotlib
pip install rosbags  # 用于读写ROS bag
pip install pyyaml opencv-python
```

### 3.2 WSL2配置（Windows用户）

由于VINS-Fusion运行在Ubuntu Docker中，Windows用户需要配置WSL2：

```powershell
# 1. 启用WSL2
wsl --install -d Ubuntu-24.04

# 2. 在WSL2中安装Docker
# 进入WSL2
wsl -d Ubuntu-24.04

# 安装Docker
sudo apt update
sudo apt install docker.io
sudo systemctl start docker
sudo usermod -aG docker $USER
```

### 3.3 VINS-Fusion Docker

确保Docker镜像已构建（参考 VNS_Fusion_docker.md）：

```bash
# 检查镜像是否存在
docker images | grep vins_fusion

# 如果不存在，需要构建或拉取镜像
```

### 3.4 数据目录配置

```bash
# 创建数据目录（WSL2中）
mkdir -p ~/data

# 确保Windows可以访问
# Windows路径: \\wsl$\Ubuntu-24.04\home\<username>\data
```

---

## 4. 模块说明

### 4.1 EKF融合器 (ekf_fusion.py)

#### 状态向量定义

```python
# 13维状态向量
x = [
    p_x, p_y, p_z,           # 位置 (3)
    v_x, v_y, v_z,           # 速度 (3)
    q_w, q_x, q_y, q_z,      # 姿态四元数 (4)
    b_gx, b_gy, b_gz         # 陀螺仪偏置 (3)
]
```

#### 主要方法

| 方法 | 功能 | 输入 |
|------|------|------|
| `initialize()` | 初始化EKF状态 | 位置、速度、姿态、时间戳 |
| `predict()` | IMU预测步骤 | IMU数据（加速度、角速度） |
| `update_vins()` | VINS位姿更新 | VINS位姿（位置+姿态） |
| `update_star_tracker()` | 星敏感器更新 | 姿态四元数 |
| `update_wheel_odometry()` | 轮速计更新 | 速度（线速度+角速度） |

#### 使用示例

```python
from ekf_fusion import MultiSensorEKF

# 创建EKF
ekf = MultiSensorEKF()

# 初始化
ekf.initialize(
    position=[0, 0, 0],
    velocity=[0, 0, 0],
    quaternion=[1, 0, 0, 0],
    timestamp=0.0
)

# IMU预测
ekf.predict({
    'timestamp': 0.01,
    'linear_acceleration': [0.0, 0.0, 1.62],
    'angular_velocity': [0.0, 0.0, 0.0]
})

# VINS更新
ekf.update_vins({
    'timestamp': 0.1,
    'position': [0.1, 0.0, 0.0],
    'quaternion': [1, 0, 0, 0]
})

# 获取状态
state = ekf.get_state()
```

### 4.2 VINS接口 (vins_docker_interface.py)

#### 执行模式

| 模式 | 描述 | 适用场景 |
|------|------|---------|
| `wsl` | 通过WSL2调用Docker | Windows + WSL2 |
| `ssh` | 通过SSH连接远程Ubuntu | 远程服务器 |
| `native` | 直接在Linux上运行 | 原生Linux |

#### 主要方法

```python
from vins_docker_interface import VINSDockerInterface, VINSConfig

# 创建配置
config = VINSConfig()
config.container_name = "vins_fusion"
config.image_name = "vins_fusion:melodic"

# 创建接口
vins = VINSDockerInterface(config)

# 检查Docker状态
status = vins.check_docker_status()
print(f"Docker状态: {status}")

# 启动容器
vins.start_container(with_gui=False)

# 处理数据集
result = vins.process_bag_offline(
    bag_path="~/data/dataset.bag",
    output_csv="~/data/vins_output.csv"
)

# 获取轨迹
trajectory = result['trajectory']
```

### 4.3 融合管道 (fusion_pipeline.py)

#### 命令行参数

```bash
python fusion_pipeline.py --help

# 必需参数
--dataset, -d     数据集路径

# 可选参数
--output, -o      输出目录 (默认: ./fusion_output)
--no-vins         禁用VINS-Fusion
--no-star-tracker 禁用星敏感器融合
--no-wheel-odom   禁用轮速计融合
--no-viz          禁用可视化
--vins-config     VINS配置文件路径
```

---

## 5. 使用指南

### 5.1 完整工作流程

#### 步骤1：采集数据

```python
# 使用仿真器采集数据（参考main.py）
# 数据将保存为EuRoC格式 + ROS bag格式
```

#### 步骤2：准备数据

确保数据集目录结构正确：

```
dataset/20260113_164908/
├── mav0/
│   ├── cam0/data/          # 导航相机左
│   ├── cam1/data/          # 导航相机右
│   ├── cam2/data/          # 避障相机左
│   ├── cam3/data/          # 避障相机右
│   ├── imu0/data.csv       # IMU数据
│   └── state_groundtruth_estimate0/data.csv  # 真值
├── dataset.bag             # ROS bag
└── collection_metadata.json
```

#### 步骤3：复制数据到WSL2（Windows用户）

```powershell
# 复制到WSL2数据目录
cp -r .\dataset\20260113_164908 \\wsl$\Ubuntu-24.04\home\<user>\data\
```

#### 步骤4：运行融合

```bash
# 完整融合（VINS + EKF）
python fusion_pipeline.py -d ./dataset/20260113_164908

# 仅EKF融合（不使用VINS）
python fusion_pipeline.py -d ./dataset/20260113_164908 --no-vins

# 指定输出目录
python fusion_pipeline.py -d ./dataset/20260113_164908 -o ./my_output
```

### 5.2 仅运行EKF融合（无VINS）

如果Docker/VINS配置困难，可以使用纯Python方案：

```python
from fusion_pipeline import MultiSensorFusionPipeline, FusionConfig

# 创建配置（禁用VINS）
config = FusionConfig(
    dataset_path="./dataset/20260113_164908",
    use_vins=False,           # 不使用VINS
    use_star_tracker=True,    # 使用星敏感器
    use_wheel_odometry=True   # 使用轮速计
)

# 运行融合
pipeline = MultiSensorFusionPipeline(config)
pipeline.load_dataset(config.dataset_path)
pipeline.run_fusion()
pipeline.save_results()
pipeline.visualize()
```

### 5.3 自定义传感器噪声

```python
from ekf_fusion import MultiSensorEKF

# 自定义EKF配置
ekf_config = {
    'gravity': 1.62,  # 月球重力
    
    'process_noise': {
        'position': 0.01,
        'velocity': 0.1,
        'attitude': 0.001,
        'gyro_bias': 0.0001,
    },
    
    'vins_noise': {
        'position': 0.05,   # VINS位置噪声 (m)
        'attitude': 0.01,   # VINS姿态噪声 (rad)
    },
    
    'star_tracker_noise': {
        'attitude': 1.7e-5,  # 星敏感器噪声 (~0.001度)
    },
    
    'wheel_odom_noise': {
        'velocity': 0.05,    # 轮速计噪声 (m/s)
    }
}

ekf = MultiSensorEKF(ekf_config)
```

---

## 6. ROS robot_localization方案

> ⭐ **推荐方案**：使用ROS的robot_localization包，无需在Docker外额外安装ROS

### 6.1 方案优势

| 特性 | robot_localization | Python EKF |
|------|-------------------|------------|
| 成熟度 | 生产级别，广泛测试 | 原型实现 |
| 传感器支持 | 自动处理多种ROS消息 | 手动实现 |
| 时间同步 | 完善的时间戳处理 | 简单排序 |
| 协方差处理 | 完整诊断 | 基础实现 |
| 可配置性 | YAML配置文件 | 代码级别 |

### 6.2 架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Docker Container (ROS Melodic)                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   rosbag play ──► /obstacle_camera/* ──► VINS-Fusion                │
│              ──► /imu0                        │                      │
│                                               ▼                      │
│                                    /vins_estimator/odometry          │
│                                               │                      │
│   rosbag play ──► /star_tracker/attitude ─────┼──► robot_localization│
│              ──► /wheel_odometry/twist ───────┘      (EKF)          │
│                                                        │             │
│                                               /odometry/filtered     │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.3 快速开始

#### 方式1：自动化运行

```bash
# 生成配置文件
python ros_fusion_pipeline.py --generate-config -o ./ros_config

# 完整融合（VINS + EKF）
python ros_fusion_pipeline.py --bag ./dataset/20260113_164908/dataset.bag

# 仅EKF融合
python ros_fusion_pipeline.py --bag ./dataset/dataset.bag --ekf-only
```

#### 方式2：手动运行（推荐用于调试）

```bash
# 查看详细手动步骤
python ros_fusion_pipeline.py --manual
```

### 6.4 手动运行详细步骤

#### 步骤1：准备工作

```bash
# 在Windows上，先生成配置文件
python ros_fusion_pipeline.py --generate-config -o ./ros_config

# 将配置文件复制到WSL2数据目录
cp ./ros_config/*.yaml \\wsl$\Ubuntu-24.04\home\<user>\data\
cp ./ros_config/*.launch \\wsl$\Ubuntu-24.04\home\<user>\data\
```

#### 步骤2：进入WSL2/Ubuntu

```bash
# Windows用户
wsl -d Ubuntu-24.04

# 或SSH到远程Ubuntu
ssh user@ubuntu-host
```

#### 步骤3：启动Docker容器

```bash
# 终端1：启动容器和roscore
docker start vins_fusion
docker exec -it vins_fusion bash

# 在容器内
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash
roscore
```

#### 步骤4：安装robot_localization（首次运行）

```bash
# 终端2：安装依赖
docker exec -it vins_fusion bash
apt-get update
apt-get install -y ros-melodic-robot-localization ros-melodic-topic-tools
```

#### 步骤5：启动EKF节点

```bash
# 终端2（继续）
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash

# 启动robot_localization EKF
roslaunch /root/data/ekf_only.launch
```

#### 步骤6：启动VINS-Fusion（可选）

```bash
# 终端3
docker exec -it vins_fusion bash
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash

rosrun vins vins_node /root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
```

#### 步骤7：录制输出

```bash
# 终端4
docker exec -it vins_fusion bash
source /opt/ros/melodic/setup.bash

rosbag record -O /root/data/fusion_output.bag \
    /ekf/odometry \
    /vins_estimator/odometry \
    /vins_estimator/path \
    /ground_truth/pose
```

#### 步骤8：播放数据集

```bash
# 终端5
docker exec -it vins_fusion bash
source /opt/ros/melodic/setup.bash

rosbag play /root/data/dataset.bag --clock
```

#### 步骤9：获取结果

```bash
# 等待播放完成后，按Ctrl+C停止录制

# 复制输出到主机
docker cp vins_fusion:/root/data/fusion_output.bag ~/data/

# 查看输出信息
rosbag info ~/data/fusion_output.bag
```

### 6.5 EKF配置说明

生成的`ekf_config.yaml`关键配置：

```yaml
# 传感器配置矩阵说明
# [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]

# VINS输出：使用位置和姿态
odom0: /vins_estimator/odometry
odom0_config: [true,  true,  true,   # 位置
               true,  true,  true,   # 姿态
               false, false, false,  # 速度（不使用，可能有延迟）
               false, false, false,  # 角速度
               false, false, false]  # 加速度

# IMU：使用姿态、角速度、加速度
imu0: /imu0
imu0_config: [false, false, false,  # 位置
              true,  true,  true,   # 姿态
              false, false, false,  # 速度
              true,  true,  true,   # 角速度
              true,  true,  true]   # 加速度

# 轮速计：使用速度
twist0: /wheel_odometry/twist
twist0_config: [false, false, false,
                false, false, false,
                true,  true,  true,   # 线速度
                true,  true,  true,   # 角速度
                false, false, false]

# 星敏感器：仅姿态（需要消息转换）
pose0: /star_tracker/pose
pose0_config: [false, false, false,
               true,  true,  true,   # 姿态
               false, false, false,
               false, false, false,
               false, false, false]
```

### 6.6 调整噪声参数

如果融合效果不理想，可以调整`ekf_config.yaml`中的噪声参数：

```yaml
# 过程噪声（对角线元素）
# 位置噪声大 → 更信任传感器测量
# 位置噪声小 → 更信任模型预测
process_noise_covariance: [
    0.05,  # x
    0.05,  # y
    0.06,  # z
    0.03,  # roll
    0.03,  # pitch
    0.06,  # yaw
    # ... 速度、角速度、加速度噪声
]
```

---

## 7. 算法原理

### 6.1 扩展卡尔曼滤波器 (EKF)

#### 状态方程

$$
\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k) + \mathbf{w}_k
$$

其中：
- $\mathbf{x}_k$: 状态向量
- $\mathbf{u}_k$: 控制输入（IMU测量）
- $\mathbf{w}_k$: 过程噪声

#### 预测步骤（IMU驱动）

**位置更新**：
$$
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2}\mathbf{a}_k \Delta t^2
$$

**速度更新**：
$$
\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_k \Delta t
$$

**姿态更新**（四元数积分）：
$$
\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \Delta\mathbf{q}
$$

其中 $\Delta\mathbf{q}$ 由角速度 $\boldsymbol{\omega}$ 和时间间隔计算：
$$
\Delta\mathbf{q} = \begin{bmatrix} \cos(\frac{\|\boldsymbol{\omega}\|\Delta t}{2}) \\ \frac{\boldsymbol{\omega}}{\|\boldsymbol{\omega}\|}\sin(\frac{\|\boldsymbol{\omega}\|\Delta t}{2}) \end{bmatrix}
$$

#### 更新步骤

**卡尔曼增益**：
$$
\mathbf{K} = \mathbf{P}_{k|k-1}\mathbf{H}^T(\mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^T + \mathbf{R})^{-1}
$$

**状态更新**：
$$
\mathbf{x}_{k|k} = \mathbf{x}_{k|k-1} + \mathbf{K}(\mathbf{z}_k - h(\mathbf{x}_{k|k-1}))
$$

**协方差更新**：
$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}_{k|k-1}
$$

### 6.2 传感器融合策略

| 传感器 | 测量量 | 更新频率 | 噪声特性 | 融合权重 |
|--------|--------|----------|----------|----------|
| IMU | 加速度、角速度 | 100Hz | 高频噪声、偏置漂移 | 预测驱动 |
| VINS | 位置、姿态 | 20Hz | 中等噪声、长期稳定 | 主要更新 |
| 星敏感器 | 姿态 | 10Hz | 极低噪声、高精度 | 姿态校正 |
| 轮速计 | 速度 | 50Hz | 打滑影响、中等噪声 | 速度约束 |

### 6.3 四元数运算

**四元数乘法**（Hamilton约定）：
$$
\mathbf{q}_1 \otimes \mathbf{q}_2 = \begin{bmatrix}
w_1w_2 - x_1x_2 - y_1y_2 - z_1z_2 \\
w_1x_2 + x_1w_2 + y_1z_2 - z_1y_2 \\
w_1y_2 - x_1z_2 + y_1w_2 + z_1x_2 \\
w_1z_2 + x_1y_2 - y_1x_2 + z_1w_2
\end{bmatrix}
$$

**四元数球面插值 (SLERP)**：
$$
\text{slerp}(\mathbf{q}_1, \mathbf{q}_2, t) = \frac{\sin((1-t)\theta)}{\sin\theta}\mathbf{q}_1 + \frac{\sin(t\theta)}{\sin\theta}\mathbf{q}_2
$$

其中 $\theta = \arccos(\mathbf{q}_1 \cdot \mathbf{q}_2)$

---

## 8. Python EKF vs ROS robot_localization对比

### 8.1 功能对比

| 功能 | Python EKF (ekf_fusion.py) | ROS robot_localization |
|------|---------------------------|------------------------|
| **状态维度** | 13维 | 15维 |
| **滤波器类型** | EKF | EKF + UKF |
| **传感器数量** | 有限 | 理论无限 |
| **消息类型** | 手动解析 | 自动处理 nav_msgs, sensor_msgs |
| **坐标变换** | 手动处理 | 自动TF变换 |
| **外推预测** | 无 | 支持 |
| **诊断输出** | 基础 | 完整ROS诊断 |
| **实时性能** | 依赖Python | C++优化 |

### 8.2 选择建议

**选择 robot_localization 的情况：**
- 已有ROS环境（Docker中已有）
- 需要与其他ROS节点集成
- 需要成熟稳定的解决方案
- 需要详细的诊断信息

**选择 Python EKF 的情况：**
- 无法运行ROS（备选方案）
- 需要深度定制滤波器
- 学习和研究目的
- 快速原型验证

### 8.3 两种方案的文件

```
ros_fusion_pipeline.py   → ROS robot_localization方案（推荐）
fusion_pipeline.py       → Python EKF方案（备选）
ekf_fusion.py            → Python EKF实现（备选）
```

---

## 9. 常见问题

### 9.1 robot_localization相关

**Q: 需要在Docker外安装ROS吗？**

A: **不需要**。robot_localization可以直接安装在现有的VINS Docker容器中：
```bash
docker exec -it vins_fusion bash
apt-get install -y ros-melodic-robot-localization
```

**Q: 星敏感器消息格式不兼容？**

A: robot_localization不直接支持`QuaternionStamped`，需要转换为`PoseStamped`：
```bash
# 使用topic_tools进行消息转换
rosrun topic_tools transform /star_tracker/attitude /star_tracker/pose \
    geometry_msgs/PoseStamped \
    'geometry_msgs.msg.PoseStamped(header=m.header, 
     pose=geometry_msgs.msg.Pose(orientation=m.quaternion))'
```

**Q: EKF输出频率太低？**

A: 调整配置文件中的`frequency`参数：
```yaml
frequency: 100  # 提高到100Hz
```

**Q: 融合结果有明显跳变？**

A: 可能原因和解决方案：
1. 传感器噪声参数设置不当 → 调整`process_noise_covariance`
2. 异常值未被拒绝 → 调整`rejection_threshold`参数
3. 时间戳不同步 → 检查传感器时间戳

### 9.2 VINS相关

**Q: VINS处理时间很长？**

A: 可以尝试：
1. 降低图像分辨率（在config中调整）
2. 减少特征点数量（修改VINS配置的`max_cnt`参数）
3. 使用更强的CPU

**Q: Docker无法连接？**

A: 检查：
1. WSL2是否正常运行：`wsl --status`
2. Docker是否启动：`docker info`
3. 网络是否通畅

### 9.3 Python EKF相关

**Q: 融合结果发散？**

A: 可能原因：
1. 初始状态设置错误
2. 过程噪声参数过小
3. 传感器时间戳不同步

解决方案：
```python
# 增大过程噪声
ekf_config = {
    'process_noise': {
        'position': 0.1,    # 增大
        'velocity': 0.5,    # 增大
        'attitude': 0.01,
        'gyro_bias': 0.001,
    }
}
```

**Q: 姿态估计不准确？**

A: 确保：
1. 四元数格式一致（本系统使用 [w, x, y, z]）
2. 星敏感器数据正确加载
3. IMU偏置估计正常

### 9.4 数据相关

**Q: rosbag无法读取？**

A: 
1. 确保安装了rosbags库：`pip install rosbags`
2. 检查bag文件完整性：`rosbag info xxx.bag`

**Q: 时间戳单位问题？**

A: 本系统统一使用**秒**为时间单位：
- 纳秒 → 秒：`timestamp_s = timestamp_ns / 1e9`
- CSV文件中存储的是纳秒

---

## 附录

### A. 坐标系定义

- **世界坐标系**：NED（北-东-地）或ENU（东-北-上）
- **车体坐标系**：前-左-上 (FLU)
- **相机坐标系**：右-下-前 (RDF)

### B. 参考文献

1. VINS-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
2. EuRoC MAV Dataset: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
3. Quaternion kinematics for the error-state Kalman filter (Joan Solà)

### C. 更新日志

- **2026-01-13**: 初始版本，实现EKF融合器和VINS接口
