# 月球车VINS数据采集系统

基于UE4+AirSim的月球车视觉惯性导航（VINS）数据集采集系统。

## 系统概述

本系统配置了一台装备双目相机和IMU的轮式月球车，用于采集VINS-Fusion、hdl_localization等算法所需的数据集。

### 传感器配置

1. **导航双目相机**（车体上方）
   - 分辨率：752×480
   - 视场角：90°
   - 基线距离：0.12m
   - 采样频率：20Hz
   - 用途：导航、全局/局部地图生成

2. **避障双目相机**（车体下方）
   - 分辨率：640×480
   - 视场角：60°
   - 基线距离：0.10m
   - 采样频率：20Hz
   - 用途：近距离避障、局部地图生成

3. **IMU**
   - 采样频率：100Hz
   - 输出：加速度、角速度、姿态

4. **轮速计**
   - 采样频率：50Hz
   - 四轮编码器，包含噪声和打滑模拟

## 文件说明

- `settings_lunar_rover.json` - AirSim配置文件（月球车、传感器）
- `config.py` - 传感器参数配置（基线、高度、FOV等）
- `lunar_rover_env.py` - 月球车环境和数据采集类
- `dataset_saver.py` - 数据集保存工具（支持EuRoC、自定义、ROS bag格式）
- `run_data_collection.py` - 主运行脚本

## 快速开始

### 1. 环境准备

```bash
# 安装依赖
pip install numpy opencv-python pillow pyyaml airsim

# 可选：安装ROS bag支持
pip install rosbag rospkg
```

### 2. 配置AirSim

将 `settings_lunar_rover.json` 复制到AirSim配置目录：

**Windows:**
```
C:\Users\<用户名>\Documents\AirSim\settings.json
```

**Linux:**
```
~/Documents/AirSim/settings.json
```

### 3. 启动UE4环境

启动UE4月球表面仿真环境，确保月球车模型正确加载。

### 4. 运行数据采集

```bash
# 基础运行（生成随机游走轨迹，保存为EuRoC格式）
python run_data_collection.py

# 指定轨迹类型
python run_data_collection.py --trajectory_type figure_eight

# 指定保存格式
python run_data_collection.py --save_format custom

# 采集多组数据
python run_data_collection.py --num_runs 5 --seed 42
```

### 5. 可用参数

- `--seed` - 随机种子（默认：42）
- `--trajectory_type` - 轨迹类型：`random_walk`, `figure_eight`, `spiral`, `grid`
- `--save_format` - 保存格式：`euroc`, `custom`, `rosbag`
- `--num_runs` - 运行次数（默认：1）

## 轨迹类型

1. **random_walk** - 随机游走（10×10m区域内随机移动）
2. **figure_eight** - 8字形轨迹（平滑曲线）
3. **spiral** - 螺旋轨迹（从中心向外）
4. **grid** - 网格轨迹（规律扫描）

## 数据集格式

### EuRoC MAV格式（推荐用于VINS-Fusion）

```
dataset/
└── mav0/
    ├── cam0/              # 导航相机左
    │   ├── data/
    │   │   ├── 1234567890.png
    │   │   └── ...
    │   └── data.csv
    ├── cam1/              # 导航相机右
    ├── cam2/              # 避障相机左
    ├── cam3/              # 避障相机右
    ├── imu0/
    │   └── data.csv       # IMU数据
    ├── state_groundtruth_estimate0/
    │   └── data.csv       # 真值位姿
    └── sensor.yaml        # 传感器标定参数
```

### 自定义格式

```
dataset/
├── nav_camera_left/       # 导航相机图像
├── nav_camera_right/
├── obstacle_camera_left/  # 避障相机图像
├── obstacle_camera_right/
├── depth/                 # 深度图
└── metadata.json          # 所有数据的元信息
```

### ROS bag格式

- Topic: `/nav_camera/left/image_raw`
- Topic: `/nav_camera/right/image_raw`
- Topic: `/obstacle_camera/left/image_raw`
- Topic: `/obstacle_camera/right/image_raw`
- Topic: `/imu0`
- Topic: `/ground_truth/pose`

## 参数调整

在 `config.py` 中可以调整以下参数：

### 相机参数
```python
nav_camera_params = {
    'baseline': 0.12,              # 基线距离
    'height_above_ground': 0.5,    # 离地高度
    'fov': 90,                     # 视场角
    'fps': 20,                     # 帧率
    # ...
}
```

### IMU参数
```python
imu_params = {
    'sampling_rate': 100,          # 采样率
    'accel_noise_sigma': 0.02,     # 加速度噪声
    'gyro_noise_sigma': 0.001,     # 陀螺仪噪声
    # ...
}
```

### 轨迹参数
```python
trajectory_params = {
    'area_size': 10.0,             # 活动区域大小
    'waypoint_spacing': 2.0,       # 航点间隔
    'speed': 0.5,                  # 巡航速度
    # ...
}
```

## 数据用途

### VINS-Fusion
使用EuRoC格式数据集，包含：
- 双目相机图像（用于视觉里程计）
- IMU数据（用于IMU预积分）
- 真值位姿（用于精度评估）

### hdl_localization
使用点云数据进行定位：
1. 初始时刻导航相机生成局部地图
2. 实时图像生成点云进行匹配
3. 需要将深度图转换为点云格式

### 数据处理示例

```python
from dataset_saver import DatasetSaver
from lunar_rover_env import LunarRoverEnv

# 加载数据集
import json
with open('dataset/metadata.json', 'r') as f:
    metadata = json.load(f)

# 处理相机图像
for frame in metadata['frames']:
    if 'nav_left' in frame:
        img_path = f"dataset/{frame['nav_left']}"
        # 处理图像...
    
    # IMU数据
    if 'imu' in frame:
        accel = frame['imu']['linear_acceleration']
        gyro = frame['imu']['angular_velocity']
        # 处理IMU数据...
```

## 注意事项

1. **确保AirSim正确连接**：运行前检查UE4环境是否启动
2. **磁盘空间**：每次采集约需要1-2GB空间（取决于轨迹长度）
3. **性能要求**：建议使用GPU加速，CPU占用率较高时可降低采样频率
4. **传感器同步**：系统已实现软件时间戳同步，但存在少量延迟

## 故障排除

### 连接失败
```
错误：无法连接到AirSim
解决：确保UE4环境已启动，检查settings.json配置
```

### 图像采集失败
```
错误：图像数据为空
解决：检查相机命名是否与settings.json一致
```

### 内存不足
```
错误：内存溢出
解决：减少num_runs或降低图像分辨率
```

## 扩展功能

### 添加新传感器

在 `settings_lunar_rover.json` 中添加传感器：
```json
"new_sensor": {
    "SensorType": 1,  // 1=Camera, 2=IMU, 3=GPS, etc.
    "Enabled": true,
    // 传感器参数...
}
```

### 自定义轨迹

在 `lunar_rover_env.py` 的 `generate_trajectory()` 方法中添加新轨迹类型。

## 参考文档

- [AirSim文档](https://microsoft.github.io/AirSim/)
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [EuRoC数据集格式](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

## 许可证

本项目基于原始AirSim穿越项目改编。

## 联系方式

如有问题或建议，请提交Issue。
