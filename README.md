# LunarSimulator

基于 AirSim + UE4 的月球车多传感器数据采集与融合定位仿真平台。

## 数据流

```
AirSim UE4 仿真
      │
      ▼
   main.py ──► lunar_rover_env.py（采集）──► dataset_saver.py（保存）
      │                                          │
      │                                    ┌─────┴──────┐
      │                               EuRoC mav0/   dataset.bag
      │                                    │            │
      ▼                                    │     rosbag_process.py
   config.py                               │       (后处理/插值)
                                           ▼
                       generate_vins_config.py ──► VINS-Fusion YAML
                       generate_ekf_config.py  ──► robot_localization YAML
                       estimate_imu_noise.py   ──► IMU 噪声参数
                                           │
                           ┌───────────────┤
                           ▼               ▼
                     WSL2/ROS 管线    Lunar-Perception-in-Python/
                     (VINS+EKF)        (纯 Python 离线验证)
```

## 核心脚本

| 脚本 | 作用 | 输入 | 输出 |
|------|------|------|------|
| `main.py` | 主入口：连接 AirSim，驾驶月球车采集传感器数据 | `config.py`；运行中的 AirSim 仿真 | EuRoC 数据集 + ROS bag → `./dataset/<timestamp>/` |
| `config.py` | 全局参数配置（车辆、相机、IMU、轮速计、轨迹等） | — | 供其他脚本 `import` |
| `lunar_rover_env.py` | AirSim 环境封装：采集双目图像、IMU、轮速计 | `config.py`；AirSim API | 内存中的传感器数据 dict |
| `dataset_saver.py` | 保存采集数据为 EuRoC 格式 + ROS bag | 传感器数据 dict；`config.py` | `mav0/`（cam0-3, imu0, groundtruth）、`dataset.bag`、`sensor.yaml`、`collection_metadata.json` |

## 配置生成

| 脚本 | 作用 | 输入 | 输出 |
|------|------|------|------|
| `generate_vins_config.py` | 从 AirSim settings.json 生成 VINS-Fusion 配置 | `settings.json`；可选数据集路径（自动估噪声） | `cam0.yaml`、`cam1.yaml`、`VIO.yaml` |
| `generate_ekf_config.py` | 生成 robot_localization EKF 配置 | 硬编码融合参数 | EKF `.yaml` |
| `estimate_imu_noise.py` | 从数据集 IMU 数据估算 VINS 四个噪声参数 | 数据集路径（含 `mav0/imu0/data.csv`） | 控制台输出 acc_n/gyr_n/acc_w/gyr_w；可选 YAML 文件 |

## 数据后处理

| 脚本 | 作用 | 输入 | 输出 |
|------|------|------|------|
| `rosbag_process.py` | ROS bag 后处理：检查/验证/IMU 插值/降采样/导出 | `dataset.bag` 路径 + 任务类型 | 处理后的 bag / 导出 CSV |
| `verify_dataset.py` | 验证数据集时间戳质量（IMU 间隔异常检测） | EuRoC 数据集路径 | 统计报告 + 图表 → `verification_output/` |
| `verify_extrinsics.py` | 验证相机外参一致性（settings.json vs sensor.yaml vs VINS config） | `settings.json`、`sensor.yaml`、VINS YAML | 控制台对比结果 |

## 性能调优（诊断/实验用）

| 脚本 | 作用 | 输入 | 输出 |
|------|------|------|------|
| `async_capture_fix.py` | 对比采集策略（纯 IMU / RGB-only / 异步深度等） | 运行中的 AirSim；命令行选择场景 | 各场景帧率统计 |
| `fov_test_manual.py` | 不同 FOV 的图像质量对比 | AirSim + 手动改 FOV | 测试图像 + 对比图表 |

## 数据集格式

输出采用 [EuRoC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) 格式：

```
dataset/<name>/
├── collection_metadata.json
├── dataset.bag
└── mav0/
    ├── sensor.yaml
    ├── cam0/          # 避障左目（光学坐标系）
    │   ├── data.csv   # timestamp, filename
    │   └── data/      # PNG 图像
    ├── cam1/          # 避障右目
    ├── cam2/          # 导航左目
    ├── cam3/          # 导航右目
    ├── imu0/
    │   └── data.csv   # timestamp, gx, gy, gz, ax, ay, az
    └── state_groundtruth_estimate0/
        └── data.csv   # timestamp, px, py, pz, qw, qx, qy, qz, vx, vy, vz, bwx, bwy, bwz, bax, bay, baz
```
