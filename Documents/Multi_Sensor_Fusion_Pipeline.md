# 多传感器融合数据处理流程

## 数据采集架构

```
AirSim仿真环境
    ↓
数据采集 (run_data_collection.py)
    ↓
同时输出两种格式：
├─ EuRoC格式 (mav0/)          → 供VINS-Fusion使用
└─ ROS bag (dataset.bag)      → 供EKF融合使用
```

## ROS bag中的Topic列表

### 输入到VINS的数据
- `/obstacle_camera/left/image_raw` - 避障相机左图（供VINS）
- `/obstacle_camera/right/image_raw` - 避障相机右图（供VINS）
- `/imu0` - IMU数据（供VINS和EKF）

### 输入到EKF的传感器数据
- `/vins/odometry` - VINS输出的位姿估计（在线或离线生成）
- `/star_tracker/attitude` - 星敏感器姿态测量（真值+高精度噪声）
- `/wheel_odometry/twist` - 轮速计速度测量（真值+低精度噪声）

### 可选输入
- `/nav_camera/left/image_raw` - 导航相机左图
- `/nav_camera/right/image_raw` - 导航相机右图

### 评估用真值
- `/ground_truth/pose` - 位姿真值
- `/ground_truth/twist` - 速度真值

## 处理流程

### 步骤1：运行数据采集
```bash
python run_data_collection.py
```

生成的数据：
- `dataset/20260113_XXXXXX/mav0/` - EuRoC格式
- `dataset/20260113_XXXXXX/dataset.bag` - ROS bag

### 步骤2：运行VINS-Fusion

**方案A：使用EuRoC格式（离线）**
```bash
roslaunch vins_estimator euroc.launch \
  data_path:=/path/to/dataset/20260113_XXXXXX/mav0 \
  config_path:=/path/to/lunar_rover_vins_config.yaml
```

**方案B：使用ROS bag（在线）**
```bash
# Terminal 1: 启动VINS节点
roslaunch vins_estimator vins_rviz.launch \
  config_path:=/path/to/lunar_rover_vins_config.yaml

# Terminal 2: 播放bag文件
rosbag play dataset.bag \
  /obstacle_camera/left/image_raw:=/cam0/image_raw \
  /obstacle_camera/right/image_raw:=/cam1/image_raw
```

VINS会发布位姿到 `/vins/odometry`

### 步骤3：运行EKF融合节点

使用`robot_localization`包的EKF节点：

```bash
roslaunch lunar_rover_fusion ekf_fusion.launch
```

**ekf_fusion.launch 配置示例：**
```xml
<launch>
  <!-- EKF节点 -->
  <node pkg="robot_localization" type="ekf_localization_node" 
        name="ekf_fusion" clear_params="true">
    
    <rosparam command="load" file="$(find lunar_rover_fusion)/config/ekf_config.yaml" />
    
    <!-- 频率 -->
    <param name="frequency" value="50"/>
    <param name="sensor_timeout" value="0.1"/>
    
    <!-- 输出frame -->
    <param name="map_frame" value="map"/>
    <param name="odom_frame" value="odom"/>
    <param name="base_link_frame" value="base_link"/>
    <param name="world_frame" value="map"/>
    
    <!-- 订阅的topic -->
    <remap from="odometry/filtered" to="/ekf/odometry"/>
  </node>
  
  <!-- 播放bag文件 -->
  <node pkg="rosbag" type="play" name="player" 
        args="--clock /path/to/dataset.bag"/>
  
  <!-- 可视化 -->
  <node pkg="rviz" type="rviz" name="rviz" 
        args="-d $(find lunar_rover_fusion)/rviz/fusion.rviz"/>
</launch>
```

**ekf_config.yaml 配置示例：**
```yaml
# EKF融合配置
frequency: 50
sensor_timeout: 0.1
two_d_mode: false
transform_time_offset: 0.0
transform_timeout: 0.0
print_diagnostics: true
debug: false

# 输入的传感器
# 格式: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]

# VINS位姿输入 (高精度位置和姿态)
odom0: /vins/odometry
odom0_config: [true,  true,  true,   # x, y, z
               true,  true,  true,   # roll, pitch, yaw
               false, false, false,  # vx, vy, vz (VINS不提供)
               false, false, false,  # vroll, vpitch, vyaw
               false, false, false]  # ax, ay, az
odom0_differential: false
odom0_relative: false
odom0_queue_size: 10

# 星敏感器姿态输入 (高精度姿态)
pose0: /star_tracker/attitude
pose0_config: [false, false, false,  # x, y, z (不提供位置)
               true,  true,  true,   # roll, pitch, yaw (高精度)
               false, false, false,  # vx, vy, vz
               false, false, false,  # vroll, vpitch, vyaw
               false, false, false]  # ax, ay, az
pose0_differential: false
pose0_relative: false

# 轮速计速度输入 (低精度速度)
twist0: /wheel_odometry/twist
twist0_config: [false, false, false,  # x, y, z
                false, false, false,  # roll, pitch, yaw
                true,  true,  true,   # vx, vy, vz (低精度)
                false, false, true]   # vroll, vpitch, vyaw (yaw rate)
twist0_differential: false
twist0_relative: false

# IMU加速度输入（来自VINS中的IMU）
imu0: /imu0
imu0_config: [false, false, false,   # x, y, z
              false, false, false,   # roll, pitch, yaw
              false, false, false,   # vx, vy, vz
              false, false, false,   # vroll, vpitch, vyaw
              true,  true,  true]    # ax, ay, az
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true

# 过程噪声协方差（根据实际调整）
process_noise_covariance: [0.05, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.05,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.06,   0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.03,   0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.03,   0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.06,   0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.025,   0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.025,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.04,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.01,   0.0,    0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.02,   0.0,    0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,
                           0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015]

# 初始状态协方差
initial_estimate_covariance: [1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
                              0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9]
```

## 传感器特性

| 传感器 | Topic | 精度 | 更新频率 | 提供信息 |
|--------|-------|------|----------|----------|
| VINS | `/vins/odometry` | 位置: 厘米级<br>姿态: 度级 | 20Hz | 位置、姿态 |
| 星敏感器 | `/star_tracker/attitude` | 0.001° | 与state同步 | 姿态（四元数） |
| 轮速计 | `/wheel_odometry/twist` | 2% 速度误差 | 与state同步 | 线速度、角速度 |
| IMU | `/imu0` | 加速度: 0.02m/s²<br>角速度: 0.01rad/s | 100Hz | 加速度、角速度 |

## 评估指标

运行完EKF后，对比融合结果与真值：

```python
# 评估脚本示例
import rosbag
import numpy as np
from pyquaternion import Quaternion

def evaluate_fusion(bag_file):
    gt_poses = []
    ekf_poses = []
    
    bag = rosbag.Bag(bag_file)
    
    # 提取真值和EKF结果
    for topic, msg, t in bag.read_messages(['/ground_truth/pose', '/ekf/odometry']):
        if topic == '/ground_truth/pose':
            gt_poses.append((t, msg.pose.position, msg.pose.orientation))
        elif topic == '/ekf/odometry':
            ekf_poses.append((t, msg.pose.pose.position, msg.pose.pose.orientation))
    
    # 计算误差
    position_errors = []
    rotation_errors = []
    
    # ... 计算ATE, RPE等指标
    
    return metrics
```

## 可视化

在RViz中显示：
- 真值轨迹（绿色）
- VINS轨迹（蓝色）
- EKF融合轨迹（红色）
- 各传感器坐标系

## 注意事项

1. **VINS需要先标定** - 使用Kalibr标定相机和IMU
2. **传感器噪声需要调整** - 根据实际情况调整`dataset_saver.py`中的噪声参数
3. **EKF参数需要调优** - 根据传感器特性调整协方差矩阵
4. **时间同步很重要** - 确保所有传感器时间戳准确
5. **坐标系对齐** - 确保所有传感器在同一坐标系下
