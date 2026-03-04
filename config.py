
# ============================================
# 月球车传感器配置参数
# ============================================
# 原则：外参(位置/姿态)和内参(FOV/分辨率)从 settings.json 读取
# 此文件仅保留：裁剪目标、采集控制(fps)、噪声参数、以及 fallback 参数

# 月球车物理参数
rover_params = {
    'track_width': 0.6,                    # 轮距 (m)
}

# 导航相机参数（上方，视野较广）
# baseline/offset/pitch 仅作为 settings.json 不可用时的 fallback
nav_camera_params = {
    'baseline': 0.12,                      # 双目基线距离 (m)
    'x_offset': 0.3,                       # 相机距车体中心前向偏移 (m)
    'z_offset': -0.5,                      # 相机距车体中心垂向偏移 (m, NED负值=向上)
    'pitch_angle': 0,                      # 俯仰角 (度)
    'resolution': (752, 480),              # 图像分辨率 (width, height)
    'fov': 90,                             # 视场角 (度)
    'fps': 2,                              # 采样频率 (Hz)
}

# 避障相机参数（下方，视野较窄）
# resolution/fov 从 settings.json 读取，此处 target_fov 控制裁剪
obstacle_camera_params = {
    'baseline': 0.10,                      # 双目基线距离 (m), fallback
    'x_offset': 0.075,                       # 前向偏移 (m), fallback
    'z_offset': -0.5,                       # 垂向偏移 (m), fallback
    'pitch_angle': 45,                     # 俯仰角 (度), fallback
    'resolution': (840, 840),              # AirSim采集分辨率 (width, height)
    'fov': 70,                             # AirSim FOV (度)
    'target_fov_h': 70,                    # 目标横向视场角 (度)
    'target_fov_v': 49,                    # 目标纵向视场角 (度)
    'fps': 2,                              # 采样频率 (Hz)
}

# IMU参数
imu_params = {
    'sampling_rate': 120,                   # 采集频率 (Hz)
    # 噪声参数（AirSim内置噪声，此处设为0；实际标定值见注释）
    'accel_noise_sigma': 0.0,       # 实际: 0.003 m/s²
    'accel_bias_sigma': 0.0,        # 实际: 0.0001 m/s³
    'gyro_noise_sigma': 0.0,        # 实际: 0.000045 rad/s
    'gyro_bias_sigma': 0.0,         # 实际: 0.000005 rad/s²
}

# 轮速计参数
wheel_encoder_params = {
    'sampling_rate': 100,                   # 采样频率 (Hz)
    'noise_std': 0.01,                     # 轮速噪声标准差（比例）
    'slip_probability': 0.05,              # 打滑概率
    'slip_factor_range': (0.8, 1.2),       # 打滑时速度因子范围
}

# 星敏感器参数
star_tracker_params = {
    # 精度参数 (对标实际硬件：0.1°/s 动态条件下)
    'xy_precision_sigma': 4.85e-6,         # X/Y 轴指向精度 (rad), 3"(3σ)
    'z_precision_sigma': 4.85e-5,          # Z 轴(滚转)精度 (rad), 30"(3σ)
}

# 轨迹生成参数（自动导航模式使用）
trajectory_params = {
    'area_size': 10.0,                     # 活动区域边长 (m)
    'area_range': (-5.0, 5.0),             # 活动区域范围 (m)
    'waypoint_spacing': 2,                 # 航点间隔距离 (m)
    'min_waypoints': 10,                    # 最少航点数量
    'max_waypoints': 30,                   # 最多航点数量
    'speed': 1.0,                          # 巡航速度 (m/s)
    'dt': 0.005,                           # 数据采集步长 (s)
}

# 数据集保存参数
dataset_params = {
    'save_path': './dataset',              # 数据集保存路径
    'save_format': 'both',                 # 保存格式: 'rosbag', 'custom', 'euroc', 'both'
    'metadata': {
        'description': 'Lunar rover multi-sensor fusion dataset',
        'environment': 'UE4+AirSim Lunar Surface',
        'sensors': ['stereo_nav', 'stereo_obstacle', 'imu', 'wheel_encoder', 'star_tracker(simulated)', 'wheel_odom(simulated)'],
        'fusion_pipeline': 'VINS (obstacle_camera+IMU) -> EKF (VINS+star_tracker+wheel_odom)'
    }
}
