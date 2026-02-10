
# ============================================
# 月球车传感器配置参数
# ============================================

# 月球车物理参数
rover_params = {
    'mass': 50.0,                          # 月球车质量 (kg)
    'wheel_radius': 0.15,                  # 车轮半径 (m)
    'wheel_base': 0.5,                     # 轴距 (m)
    'track_width': 0.6,                    # 轮距 (m)
    'max_speed': 0.5,                      # 最大速度 (m/s)
    'max_steering_angle': 45.0,            # 最大转向角度 (度)
}

# 导航相机参数（上方，视野较广）
nav_camera_params = {
    'baseline': 0.12,                      # 双目基线距离 (m)，实际基线 = baseline * 2
    'height_above_ground': 0.5,            # 相机距地高度 (m)
    'x_offset': 0.3,                       # 相机距车体中心前向偏移 (m)
    'z_offset': -0.5,                      # 相机距车体中心垂向偏移 (m, 负值表示向上)
    'pitch_angle': 0,                      # 俯仰角 (度)
    'resolution': (752, 480),              # 图像分辨率 (width, height)
    'fov': 90,                             # 视场角 (度)
    'fps': 1,                             # 采样频率 (Hz)，修改为2Hz左右
    'image_types': ['Scene', 'DepthPlanar', 'Segmentation'],  # 采集的图像类型
}

# 避障相机参数（下方，视野较窄）
obstacle_camera_params = {
    'baseline': 0.10,                      # 双目基线距离 (m)，实际基线 = baseline * 2
    'height_above_ground': 0.3,            # 相机距地高度 (m)
    'x_offset': 0.2,                       # 相机距车体中心前向偏移 (m)
    'z_offset': 0.3,                       # 相机距车体中心垂向偏移 (m)
    'pitch_angle': 15,                     # 俯仰角 (度，向下倾斜)
    'resolution': (840, 840),              # AirSim采集分辨率 (width, height)，正方形FOV 70°×70°
    'fov': 70,                             # AirSim FOV (度)，横向
    'target_fov_h': 70,                    # 目标横向视场角 (度)
    'target_fov_v': 49,                    # 目标纵向视场角 (度)
    'fps': 2,                              # 采样频率 (Hz)
    'image_types': ['Scene', 'DepthPlanar'],  # 采集的图像类型
}

# IMU参数
imu_params = {
    'position': (0.0, 0.0, 0.0),           # IMU位置（车体中心）
    'sampling_rate': 60,                   # 修改为实际采集频率 60Hz
    # 噪声参数（对标真实硬件：1°/h 零偏，0.02°/√h 随机游走）
    'accel_noise_sigma': 0.0,       # 实际性能：0.003，但airsim比这个高；降低加速度计噪声（对标高精度 MEMS）
    'accel_bias_sigma': 0.0,        # 实际性能：0.0001 降低加速度计偏置影响 (约 10ug)
    'gyro_noise_sigma': 0.0,        # 实际性能：0.000045 陀螺仪白噪声 (对应 0.02 deg/sqrt(h) @ 60Hz)
    'gyro_bias_sigma': 0.0,         # 实际性能：0.000005 陀螺仪零偏 (对应 1.03 deg/h，接近硬件图表)
}

# 轮速计参数
wheel_encoder_params = {
    'sampling_rate': 100,                   # 采样频率 (Hz)
    'resolution': 1024,                    # 编码器分辨率 (脉冲/圈)
    'noise_std': 0.01,                     # 轮速噪声标准差（比例）
    'slip_probability': 0.05,              # 打滑概率
    'slip_factor_range': (0.8, 1.2),       # 打滑时速度因子范围
}

# 星敏感器参数
star_tracker_params = {
    'sampling_rate': 10,                   # 采样频率 (Hz)
    # 精度参数 (对标实际硬件：0.1°/s 动态条件下)
    # X/Y 精度 3" (3-sigma) -> 1" (1-sigma) -> 4.85e-6 rad
    'xy_precision_sigma': 4.85e-6,         # X/Y 轴指向精度 (rad)
    # Z 轴测角精度 30" (3-sigma) -> 10" (1-sigma) -> 4.85e-5 rad
    'z_precision_sigma': 4.85e-5,          # Z 轴(滚转)精度 (rad)
}

# 轨迹生成参数（保留用于可选的自动导航功能）
trajectory_params = {
    'area_size': 10.0,                     # 活动区域边长 (m)，以初始位置为中心的正方形
    'area_range': (-5.0, 5.0),             # 活动区域范围 (m)
    'waypoint_spacing': 2,               # 航点间隔距离 (m)
    'min_waypoints': 10,                    # 最少航点数量
    'max_waypoints': 30,                   # 最多航点数量
    'trajectory_types': ['random_walk', 'figure_eight', 'spiral', 'grid'],  # 轨迹类型
    'speed': 1.0,                          # 巡航速度 (m/s)
    'dt': 0.005,                           # 数据采集步长 (s)
    'threshold': 1.0,                      # 航点到达判定阈值 (m)
}

# 手动驾驶数据采集参数
collection_params = {
    'duration': 120,                       # 采集时长 (秒)，None表示不限制
    'distance': 100,                       # 采集距离 (米)，None表示不限制
    'max_frames': None,                    # 最大帧数，None表示不限制
    'mode': 'manual',                      # 采集模式: 'manual'(手动驾驶) 或 'auto'(自动导航)
}

# 数据集保存参数
dataset_params = {
    'save_path': './dataset',              # 数据集保存路径
    'save_format': 'both',                 # 保存格式: 'rosbag', 'custom', 'euroc', 'both'（推荐）
    'compress_images': True,               # 是否压缩图像
    'save_raw_depth': True,                # 是否保存原始深度图
    'generate_point_cloud': True,          # 是否生成点云
    'point_cloud_downsample': 0.05,        # 点云降采样体素大小 (m)
    'save_ground_truth': True,             # 是否保存真值位姿
    'metadata': {
        'description': 'Lunar rover multi-sensor fusion dataset',
        'environment': 'UE4+AirSim Lunar Surface',
        'sensors': ['stereo_nav', 'stereo_obstacle', 'imu', 'wheel_encoder', 'star_tracker(simulated)', 'wheel_odom(simulated)'],
        'fusion_pipeline': 'VINS (obstacle_camera+IMU) -> EKF (VINS+star_tracker+wheel_odom)'
    }
}

# 相机内参矩阵（将根据分辨率和FOV自动计算）
def get_camera_intrinsics(resolution, fov):
    """
    根据分辨率和FOV计算相机内参矩阵
    Args:
        resolution: (width, height)
        fov: 视场角（度）
    Returns:
        K: 3x3 内参矩阵 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    """
    import math
    width, height = resolution
    fx = width / (2.0 * math.tan(math.radians(fov) / 2.0))
    fy = fx  # 假设像素是正方形
    cx = width / 2.0
    cy = height / 2.0
    return [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
