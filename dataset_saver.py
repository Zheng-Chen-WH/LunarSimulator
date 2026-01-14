"""
数据集保存工具
支持多种格式保存采集的传感器数据：
1. EuRoC MAV格式（常用于VINS评估）
2. 自定义格式（JSON + 图像）
3. ROS bag格式（需要rosbag库）
"""

import os
import json
import numpy as np
from datetime import datetime
import cv2
from pathlib import Path
import yaml

class DatasetSaver:
    def __init__(self, args):
        """
        初始化数据集保存器
        """
        self.dataset_params = args['dataset_params']
        self.nav_camera_params = args['nav_camera_params']
        self.obstacle_camera_params = args['obstacle_camera_params']
        self.imu_params = args['imu_params']
        
        # 创建保存目录
        self.base_path = Path(self.dataset_params['save_path'])
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dataset_path = self.base_path / self.timestamp_str
        
        self.save_format = self.dataset_params['save_format']
        
    def save_dataset(self, dataset, format_type=None):
        """
        保存数据集
        Args:
            dataset: 采集的数据集
            format_type: 保存格式 ('euroc', 'custom', 'rosbag', 'both')
        """
        if format_type is None:
            format_type = self.save_format
        
        print(f"\n开始保存数据集，格式: {format_type}")
        
        if format_type == 'euroc':
            self.save_euroc_format(dataset)
        elif format_type == 'custom':
            self.save_custom_format(dataset)
        elif format_type == 'rosbag':
            self.save_rosbag_format(dataset)
        elif format_type == 'both':
            # 同时保存EuRoC和ROS bag（推荐用于多传感器融合）
            print("保存EuRoC格式（用于VINS）...")
            self.save_euroc_format(dataset)
            print("\n保存ROS bag格式（用于EKF融合）...")
            self.save_rosbag_format(dataset)
        else:
            raise ValueError(f"不支持的保存格式: {format_type}")
        
        print(f"数据集已保存到: {self.dataset_path}")
    
    def save_euroc_format(self, dataset):
        """
        保存为EuRoC MAV数据集格式
        目录结构：
        dataset/
        ├── mav0/
        │   ├── cam0/          # 导航相机左
        │   │   ├── data/
        │   │   └── data.csv
        │   ├── cam1/          # 导航相机右
        │   ├── cam2/          # 避障相机左
        │   ├── cam3/          # 避障相机右
        │   ├── imu0/
        │   │   └── data.csv
        │   └── state_groundtruth_estimate0/
        │       └── data.csv
        """
        mav_path = self.dataset_path / "mav0"
        
        # 创建目录结构
        cam_dirs = ['cam0', 'cam1', 'cam2', 'cam3']
        for cam_dir in cam_dirs:
            (mav_path / cam_dir / "data").mkdir(parents=True, exist_ok=True)
        
        (mav_path / "imu0").mkdir(parents=True, exist_ok=True)
        (mav_path / "state_groundtruth_estimate0").mkdir(parents=True, exist_ok=True)
        
        # 准备CSV文件
        cam_csv_files = {
            'cam0': open(mav_path / "cam0" / "data.csv", 'w'),
            'cam1': open(mav_path / "cam1" / "data.csv", 'w'),
            'cam2': open(mav_path / "cam2" / "data.csv", 'w'),
            'cam3': open(mav_path / "cam3" / "data.csv", 'w'),
        }
        
        imu_csv = open(mav_path / "imu0" / "data.csv", 'w')
        gt_csv = open(mav_path / "state_groundtruth_estimate0" / "data.csv", 'w')
        
        # 写入CSV头
        for f in cam_csv_files.values():
            f.write("#timestamp [ns],filename\n")
        
        imu_csv.write("#timestamp [ns],w_x [rad/s],w_y [rad/s],w_z [rad/s],"
                     "a_x [m/s^2],a_y [m/s^2],a_z [m/s^2]\n")
        
        gt_csv.write("#timestamp [ns],p_x [m],p_y [m],p_z [m],"
                    "q_w,q_x,q_y,q_z,v_x [m/s],v_y [m/s],v_z [m/s],"
                    "w_x [rad/s],w_y [rad/s],w_z [rad/s]\n")
        
        # 处理数据
        print("处理相机图像...")
        for idx, data_frame in enumerate(dataset['data']):
            timestamp_ns = int(data_frame['timestamp'] * 1e9)
            
            # 保存相机图像
            if 'nav_camera' in data_frame:
                nav_data = data_frame['nav_camera']
                cam_ts = int(nav_data['timestamp'] * 1e9)
                
                # cam0: 导航左
                if 'left_rgb' in nav_data:
                    filename = f"{cam_ts}.png"
                    img_path = mav_path / "cam0" / "data" / filename
                    cv2.imwrite(str(img_path), cv2.cvtColor(nav_data['left_rgb'], cv2.COLOR_RGB2BGR))
                    cam_csv_files['cam0'].write(f"{cam_ts},{filename}\n")
                
                # cam1: 导航右
                if 'right_rgb' in nav_data:
                    filename = f"{cam_ts}.png"
                    img_path = mav_path / "cam1" / "data" / filename
                    cv2.imwrite(str(img_path), cv2.cvtColor(nav_data['right_rgb'], cv2.COLOR_RGB2BGR))
                    cam_csv_files['cam1'].write(f"{cam_ts},{filename}\n")
            
            if 'obstacle_camera' in data_frame:
                obs_data = data_frame['obstacle_camera']
                cam_ts = int(obs_data['timestamp'] * 1e9)
                
                # cam2: 避障左
                if 'left_rgb' in obs_data:
                    filename = f"{cam_ts}.png"
                    img_path = mav_path / "cam2" / "data" / filename
                    cv2.imwrite(str(img_path), cv2.cvtColor(obs_data['left_rgb'], cv2.COLOR_RGB2BGR))
                    cam_csv_files['cam2'].write(f"{cam_ts},{filename}\n")
                
                # cam3: 避障右
                if 'right_rgb' in obs_data:
                    filename = f"{cam_ts}.png"
                    img_path = mav_path / "cam3" / "data" / filename
                    cv2.imwrite(str(img_path), cv2.cvtColor(obs_data['right_rgb'], cv2.COLOR_RGB2BGR))
                    cam_csv_files['cam3'].write(f"{cam_ts},{filename}\n")
            
            # 保存IMU数据
            if 'imu' in data_frame:
                imu_data = data_frame['imu']
                imu_ts = int(imu_data['timestamp'] * 1e9)
                w = imu_data['angular_velocity']
                a = imu_data['linear_acceleration']
                imu_csv.write(f"{imu_ts},{w[0]},{w[1]},{w[2]},"
                            f"{a[0]},{a[1]},{a[2]}\n")
            
            # 保存真值位姿
            if 'state' in data_frame:
                state = data_frame['state']
                gt_ts = int(state['timestamp'] * 1e9)
                p = state['position']
                q = state['quaternion']
                v = state['velocity']
                w = state['angular_velocity']
                gt_csv.write(f"{gt_ts},{p[0]},{p[1]},{p[2]},"
                           f"{q[0]},{q[1]},{q[2]},{q[3]},"
                           f"{v[0]},{v[1]},{v[2]},"
                           f"{w[0]},{w[1]},{w[2]}\n")
            
            if (idx + 1) % 100 == 0:
                print(f"  已处理 {idx + 1}/{len(dataset['data'])} 帧")
        
        # 关闭文件
        for f in cam_csv_files.values():
            f.close()
        imu_csv.close()
        gt_csv.close()
        
        # 保存传感器配置文件
        self.save_sensor_yaml(mav_path)
        
        # 保存轨迹信息（如果存在）
        if 'trajectory' in dataset and dataset['trajectory']:
            trajectory_file = self.dataset_path / "trajectory.json"
            with open(trajectory_file, 'w') as f:
                trajectory_list = [{'x': wp[0], 'y': wp[1], 'yaw': wp[2]} 
                                 for wp in dataset['trajectory']]
                json.dump(trajectory_list, f, indent=2)
            print("轨迹信息已保存")
        
        # 保存采集元数据（时长、距离等）
        if 'metadata' in dataset:
            metadata_file = self.dataset_path / "collection_metadata.json"
            with open(metadata_file, 'w') as f:
                json.dump(dataset['metadata'], f, indent=2)
            print("采集元数据已保存")
        
        print("EuRoC格式数据集保存完成")
    
    def save_sensor_yaml(self, mav_path):
        """
        保存传感器配置YAML文件（EuRoC格式）
        """
        # 计算相机内参
        from config import get_camera_intrinsics
        
        nav_K = get_camera_intrinsics(self.nav_camera_params['resolution'],
                                      self.nav_camera_params['fov'])
        obs_K = get_camera_intrinsics(self.obstacle_camera_params['resolution'],
                                      self.obstacle_camera_params['fov'])
        
        # 相机外参（相对于IMU）
        sensor_config = {
            'cam0': {
                'camera_model': 'pinhole',
                'intrinsics': [nav_K[0][0], nav_K[1][1], nav_K[0][2], nav_K[1][2]],
                'distortion_model': 'radtan',
                'distortion_coefficients': [0.0, 0.0, 0.0, 0.0],
                'T_cam_imu': self.get_camera_extrinsics('nav', 'left'),
                'resolution': list(self.nav_camera_params['resolution']),
                'timeshift_cam_imu': 0.0
            },
            'cam1': {
                'camera_model': 'pinhole',
                'intrinsics': [nav_K[0][0], nav_K[1][1], nav_K[0][2], nav_K[1][2]],
                'distortion_model': 'radtan',
                'distortion_coefficients': [0.0, 0.0, 0.0, 0.0],
                'T_cam_imu': self.get_camera_extrinsics('nav', 'right'),
                'resolution': list(self.nav_camera_params['resolution']),
                'timeshift_cam_imu': 0.0
            },
            'cam2': {
                'camera_model': 'pinhole',
                'intrinsics': [obs_K[0][0], obs_K[1][1], obs_K[0][2], obs_K[1][2]],
                'distortion_model': 'radtan',
                'distortion_coefficients': [0.0, 0.0, 0.0, 0.0],
                'T_cam_imu': self.get_camera_extrinsics('obstacle', 'left'),
                'resolution': list(self.obstacle_camera_params['resolution']),
                'timeshift_cam_imu': 0.0
            },
            'cam3': {
                'camera_model': 'pinhole',
                'intrinsics': [obs_K[0][0], obs_K[1][1], obs_K[0][2], obs_K[1][2]],
                'distortion_model': 'radtan',
                'distortion_coefficients': [0.0, 0.0, 0.0, 0.0],
                'T_cam_imu': self.get_camera_extrinsics('obstacle', 'right'),
                'resolution': list(self.obstacle_camera_params['resolution']),
                'timeshift_cam_imu': 0.0
            }
        }
        
        # 保存YAML
        sensor_yaml_path = mav_path / "sensor.yaml"
        with open(sensor_yaml_path, 'w') as f:
            yaml.dump(sensor_config, f, default_flow_style=False)
        
        print(f"传感器配置已保存到: {sensor_yaml_path}")
    
    def get_camera_extrinsics(self, camera_type, side):
        """
        获取相机外参矩阵（相对于IMU）
        Args:
            camera_type: 'nav' 或 'obstacle'
            side: 'left' 或 'right'
        Returns:
            T: 4x4变换矩阵的列表形式
        """
        if camera_type == 'nav':
            params = self.nav_camera_params
        else:
            params = self.obstacle_camera_params
        
        # 相机位置
        x = params['x_offset']
        y = params['baseline'] if side == 'right' else -params['baseline']
        z = params['z_offset']
        
        # 相机姿态（俯仰角）
        pitch = np.radians(params['pitch_angle'])
        
        # 构造旋转矩阵（绕Y轴旋转pitch角）
        R = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # 构造4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        
        return T.flatten().tolist()
    
    def add_sensor_noise(self, true_value, noise_sigma):
        """
        添加高斯噪声模拟传感器测量
        Args:
            true_value: 真值（标量或数组）
            noise_sigma: 噪声标准差
        Returns:
            noisy_value: 带噪声的测量值
        """
        noise = np.random.randn(*np.array(true_value).shape) * noise_sigma
        return np.array(true_value) + noise
    
    def simulate_star_tracker(self, quaternion):
        """
        模拟星敏感器测量（姿态四元数 + 噪声）
        Args:
            quaternion: 真实姿态四元数 [w, x, y, z]
        Returns:
            noisy_quaternion: 带噪声的姿态四元数
        """
        # 将四元数转换为欧拉角
        w, x, y, z = quaternion
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # 添加噪声（星敏感器精度约0.001度）
        angle_noise = 1.5e-4  # rad (约0.001度)
        roll_noisy = roll + np.random.randn() * angle_noise
        pitch_noisy = pitch + np.random.randn() * angle_noise
        yaw_noisy = yaw + np.random.randn() * angle_noise
        
        # 转换回四元数
        cy = np.cos(yaw_noisy * 0.5)
        sy = np.sin(yaw_noisy * 0.5)
        cp = np.cos(pitch_noisy * 0.5)
        sp = np.sin(pitch_noisy * 0.5)
        cr = np.cos(roll_noisy * 0.5)
        sr = np.sin(roll_noisy * 0.5)
        
        w_noisy = cr * cp * cy + sr * sp * sy
        x_noisy = sr * cp * cy - cr * sp * sy
        y_noisy = cr * sp * cy + sr * cp * sy
        z_noisy = cr * cp * sy - sr * sp * cy
        
        return np.array([w_noisy, x_noisy, y_noisy, z_noisy])
    
    def simulate_wheel_odometry(self, velocity, angular_velocity):
        """
        模拟轮速计测量（速度 + 噪声）
        Args:
            velocity: 真实线速度 [vx, vy, vz]
            angular_velocity: 真实角速度 [wx, wy, wz]
        Returns:
            dict: 轮速计数据（线速度和角速度）
        
        注：轮速计在月球环境精度会降低，考虑打滑和地形影响
        """
        # 轮速计线速度噪声（约2%的误差）
        velocity_noise = 0.02 * np.linalg.norm(velocity) if np.linalg.norm(velocity) > 0 else 0.01
        noisy_velocity = self.add_sensor_noise(velocity, velocity_noise)
        
        # 轮速计角速度噪声（约5%的误差）
        angular_noise = 0.05 * np.linalg.norm(angular_velocity) if np.linalg.norm(angular_velocity) > 0 else 0.001
        noisy_angular_velocity = self.add_sensor_noise(angular_velocity, angular_noise)
        
        # 模拟偶尔的打滑（速度测量突然不准）
        if np.random.rand() < 0.05:  # 5%概率
            noisy_velocity *= (1.0 + np.random.uniform(-0.3, 0.1))
        
        return {
            'linear_velocity': noisy_velocity,
            'angular_velocity': noisy_angular_velocity
        }
    
    def save_custom_format(self, dataset):
        """
        保存为自定义格式（JSON + 图像）
        """
        self.dataset_path.mkdir(parents=True, exist_ok=True)
        
        # 创建子目录
        (self.dataset_path / "nav_camera_left").mkdir(exist_ok=True)
        (self.dataset_path / "nav_camera_right").mkdir(exist_ok=True)
        (self.dataset_path / "obstacle_camera_left").mkdir(exist_ok=True)
        (self.dataset_path / "obstacle_camera_right").mkdir(exist_ok=True)
        (self.dataset_path / "depth").mkdir(exist_ok=True)
        
        # 准备元数据
        metadata = {
            'dataset_info': self.dataset_params['metadata'],
            'sensor_params': {
                'nav_camera': self.nav_camera_params,
                'obstacle_camera': self.obstacle_camera_params,
                'imu': self.imu_params
            },
            'frames': []
        }
        
        # 添加轨迹信息（如果存在）
        if 'trajectory' in dataset:
            metadata['trajectory'] = dataset['trajectory']
        
        # 添加采集元数据（如果存在）
        if 'metadata' in dataset:
            metadata['collection_metadata'] = dataset['metadata']
        
        print("保存图像和数据...")
        for idx, data_frame in enumerate(dataset['data']):
            frame_info = {
                'timestamp': data_frame['timestamp'],
                'frame_id': idx
            }
            
            # 保存相机数据
            if 'nav_camera' in data_frame:
                nav_data = data_frame['nav_camera']
                if 'left_rgb' in nav_data:
                    img_path = f"nav_camera_left/{idx:06d}.png"
                    cv2.imwrite(str(self.dataset_path / img_path),
                              cv2.cvtColor(nav_data['left_rgb'], cv2.COLOR_RGB2BGR))
                    frame_info['nav_left'] = img_path
                
                if 'right_rgb' in nav_data:
                    img_path = f"nav_camera_right/{idx:06d}.png"
                    cv2.imwrite(str(self.dataset_path / img_path),
                              cv2.cvtColor(nav_data['right_rgb'], cv2.COLOR_RGB2BGR))
                    frame_info['nav_right'] = img_path
            
            if 'obstacle_camera' in data_frame:
                obs_data = data_frame['obstacle_camera']
                if 'left_rgb' in obs_data:
                    img_path = f"obstacle_camera_left/{idx:06d}.png"
                    cv2.imwrite(str(self.dataset_path / img_path),
                              cv2.cvtColor(obs_data['left_rgb'], cv2.COLOR_RGB2BGR))
                    frame_info['obstacle_left'] = img_path
                
                if 'right_rgb' in obs_data:
                    img_path = f"obstacle_camera_right/{idx:06d}.png"
                    cv2.imwrite(str(self.dataset_path / img_path),
                              cv2.cvtColor(obs_data['right_rgb'], cv2.COLOR_RGB2BGR))
                    frame_info['obstacle_right'] = img_path
            
            # 保存IMU数据
            if 'imu' in data_frame:
                frame_info['imu'] = {
                    'timestamp': data_frame['imu']['timestamp'],
                    'linear_acceleration': data_frame['imu']['linear_acceleration'].tolist(),
                    'angular_velocity': data_frame['imu']['angular_velocity'].tolist(),
                    'orientation': data_frame['imu']['orientation'].tolist()
                }
            
            # 保存轮速数据
            if 'wheel_encoder' in data_frame:
                frame_info['wheel_encoder'] = {
                    'timestamp': data_frame['wheel_encoder']['timestamp'],
                    'wheel_speeds': data_frame['wheel_encoder']['wheel_speeds'].tolist(),
                    'encoder_counts': data_frame['wheel_encoder']['encoder_counts'].tolist()
                }
            
            # 保存真值位姿
            if 'state' in data_frame:
                state = data_frame['state']
                frame_info['ground_truth'] = {
                    'position': state['position'].tolist(),
                    'velocity': state['velocity'].tolist(),
                    'quaternion': state['quaternion'].tolist(),
                    'angular_velocity': state['angular_velocity'].tolist()
                }
            
            metadata['frames'].append(frame_info)
            
            if (idx + 1) % 100 == 0:
                print(f"  已保存 {idx + 1}/{len(dataset['data'])} 帧")
        
        # 保存元数据JSON
        metadata_path = self.dataset_path / "metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print("自定义格式数据集保存完成")
    
    def save_rosbag_format(self, dataset):
        """
        保存为ROS bag格式（使用rosbags库，Windows兼容）
        """
        try:
            from rosbags.rosbag1 import Writer
            from rosbags.typesys import Stores, get_typestore
        except ImportError:
            print("错误：需要安装rosbags库")
            print("请运行: pip install rosbags")
            return
        
        bag_path = self.dataset_path / "dataset.bag"
        self.dataset_path.mkdir(parents=True, exist_ok=True)
        
        # 获取类型存储
        typestore = get_typestore(Stores.ROS1_NOETIC)
        
        # 定义消息类型
        sensor_msgs_Image = typestore.types['sensor_msgs/msg/Image']
        sensor_msgs_Imu = typestore.types['sensor_msgs/msg/Imu']
        nav_msgs_Odometry = typestore.types['nav_msgs/msg/Odometry']  # 使用Odometry代替TwistStamped
        geometry_msgs_PoseStamped = typestore.types['geometry_msgs/msg/PoseStamped']
        geometry_msgs_TwistStamped = typestore.types['geometry_msgs/msg/TwistStamped']
        geometry_msgs_QuaternionStamped = typestore.types['geometry_msgs/msg/QuaternionStamped']
        geometry_msgs_PoseWithCovariance = typestore.types['geometry_msgs/msg/PoseWithCovariance']
        geometry_msgs_TwistWithCovariance = typestore.types['geometry_msgs/msg/TwistWithCovariance']
        std_msgs_Header = typestore.types['std_msgs/msg/Header']
        geometry_msgs_Point = typestore.types['geometry_msgs/msg/Point']
        geometry_msgs_Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
        geometry_msgs_Pose = typestore.types['geometry_msgs/msg/Pose']
        geometry_msgs_Vector3 = typestore.types['geometry_msgs/msg/Vector3']
        geometry_msgs_Twist = typestore.types['geometry_msgs/msg/Twist']
        
        # 创建一个简单的时间类
        class Time:
            def __init__(self, sec, nanosec):
                self.sec = sec
                self.nanosec = nanosec
        
        with Writer(str(bag_path)) as writer:
            print("写入ROS bag（包含多传感器融合所需数据）...")
            
            # 创建连接（定义topic）
            # 避障相机（供VINS使用）- 使用由于EuRoC标准Topic名称
            conn_obs_left = writer.add_connection(
                '/cam0/image_raw',
                sensor_msgs_Image.__msgtype__,
                typestore=typestore
            )
            conn_obs_right = writer.add_connection(
                '/cam1/image_raw',
                sensor_msgs_Image.__msgtype__,
                typestore=typestore
            )
            
            # 导航相机（可选，供额外视觉里程计使用）
            conn_nav_left = writer.add_connection(
                '/nav_camera/left/image_raw',
                sensor_msgs_Image.__msgtype__,
                typestore=typestore
            )
            conn_nav_right = writer.add_connection(
                '/nav_camera/right/image_raw',
                sensor_msgs_Image.__msgtype__,
                typestore=typestore
            )
            
            # IMU（供VINS和EKF使用）
            conn_imu = writer.add_connection(
                '/imu0',
                sensor_msgs_Imu.__msgtype__,
                typestore=typestore
            )
            
            # 模拟星敏感器（姿态测量，供EKF使用）
            conn_star_tracker = writer.add_connection(
                '/star_tracker/attitude',
                geometry_msgs_QuaternionStamped.__msgtype__,
                typestore=typestore
            )
            
            # 模拟轮速计（速度测量，供EKF使用）- 使用Odometry避免兼容性问题
            conn_wheel_odom = writer.add_connection(
                '/wheel_odometry',
                nav_msgs_Odometry.__msgtype__,
                typestore=typestore
            )
            
            # 真值位姿（用于评估）
            conn_gt_pose = writer.add_connection(
                '/ground_truth/pose',
                geometry_msgs_PoseStamped.__msgtype__,
                typestore=typestore
            )
            
            # 真值速度（用于评估）
            conn_gt_twist = writer.add_connection(
                '/ground_truth/twist',
                geometry_msgs_TwistStamped.__msgtype__,
                typestore=typestore
            )
            
            for idx, data_frame in enumerate(dataset['data']):
                # 转换时间戳为纳秒
                timestamp_ns = int(data_frame['timestamp'] * 1e9)
                
                # 导航相机图像
                if 'nav_camera' in data_frame:
                    nav_data = data_frame['nav_camera']
                    cam_timestamp_ns = int(nav_data['timestamp'] * 1e9)
                    
                    if 'left_rgb' in nav_data:
                        img = nav_data['left_rgb']
                        height, width = img.shape[:2]
                        
                        header = std_msgs_Header(
                            seq=idx,
                            stamp=Time(
                                sec=int(cam_timestamp_ns // 1000000000),
                                nanosec=int(cam_timestamp_ns % 1000000000)
                            ),
                            frame_id='nav_camera_left'
                        )
                        
                        img_msg = sensor_msgs_Image(
                            header=header,
                            height=height,
                            width=width,
                            encoding='rgb8',
                            is_bigendian=0,
                            step=width * 3,
                            data=np.ascontiguousarray(img).flatten()
                        )
                        
                        writer.write(conn_nav_left, cam_timestamp_ns, typestore.serialize_ros1(img_msg, img_msg.__msgtype__))
                    
                    if 'right_rgb' in nav_data:
                        img = nav_data['right_rgb']
                        height, width = img.shape[:2]
                        
                        header = std_msgs_Header(
                            seq=idx,
                            stamp=Time(
                                sec=int(cam_timestamp_ns // 1000000000),
                                nanosec=int(cam_timestamp_ns % 1000000000)
                            ),
                            frame_id='nav_camera_right'
                        )
                        
                        img_msg = sensor_msgs_Image(
                            header=header,
                            height=height,
                            width=width,
                            encoding='rgb8',
                            is_bigendian=0,
                            step=width * 3,
                            data=np.ascontiguousarray(img).flatten()
                        )
                        
                        writer.write(conn_nav_right, cam_timestamp_ns, typestore.serialize_ros1(img_msg, img_msg.__msgtype__))
                
                # 避障相机图像（供VINS使用）
                if 'obstacle_camera' in data_frame:
                    obs_data = data_frame['obstacle_camera']
                    cam_timestamp_ns = int(obs_data['timestamp'] * 1e9)
                    
                    if 'left_rgb' in obs_data:
                        img = obs_data['left_rgb']
                        height, width = img.shape[:2]
                        
                        header = std_msgs_Header(
                            seq=idx,
                            stamp=Time(
                                sec=int(cam_timestamp_ns // 1000000000),
                                nanosec=int(cam_timestamp_ns % 1000000000)
                            ),
                            frame_id='obstacle_camera_left'
                        )
                        
                        img_msg = sensor_msgs_Image(
                            header=header,
                            height=height,
                            width=width,
                            encoding='rgb8',
                            is_bigendian=0,
                            step=width * 3,
                            data=np.ascontiguousarray(img).flatten()
                        )
                        
                        writer.write(conn_obs_left, cam_timestamp_ns, typestore.serialize_ros1(img_msg, img_msg.__msgtype__))
                    
                    if 'right_rgb' in obs_data:
                        img = obs_data['right_rgb']
                        height, width = img.shape[:2]
                        
                        header = std_msgs_Header(
                            seq=idx,
                            stamp=Time(
                                sec=int(cam_timestamp_ns // 1000000000),
                                nanosec=int(cam_timestamp_ns % 1000000000)
                            ),
                            frame_id='obstacle_camera_right'
                        )
                        
                        img_msg = sensor_msgs_Image(
                            header=header,
                            height=height,
                            width=width,
                            encoding='rgb8',
                            is_bigendian=0,
                            step=width * 3,
                            data=np.ascontiguousarray(img).flatten()
                        )
                        
                        writer.write(conn_obs_right, cam_timestamp_ns, typestore.serialize_ros1(img_msg, img_msg.__msgtype__))
                
                # IMU数据
                if 'imu' in data_frame:
                    imu_data = data_frame['imu']
                    imu_timestamp_ns = int(imu_data['timestamp'] * 1e9)
                    
                    header = std_msgs_Header(
                        seq=idx,
                        stamp=Time(
                            sec=int(imu_timestamp_ns // 1000000000),
                            nanosec=int(imu_timestamp_ns % 1000000000)
                        ),
                        frame_id='imu'
                    )
                    
                    imu_msg = sensor_msgs_Imu(
                        header=header,
                        linear_acceleration=geometry_msgs_Vector3(
                            x=float(imu_data['linear_acceleration'][0]),
                            y=float(imu_data['linear_acceleration'][1]),
                            z=float(imu_data['linear_acceleration'][2])
                        ),
                        angular_velocity=geometry_msgs_Vector3(
                            x=float(imu_data['angular_velocity'][0]),
                            y=float(imu_data['angular_velocity'][1]),
                            z=float(imu_data['angular_velocity'][2])
                        ),
                        orientation=geometry_msgs_Quaternion(
                            w=float(imu_data['orientation'][0]),
                            x=float(imu_data['orientation'][1]),
                            y=float(imu_data['orientation'][2]),
                            z=float(imu_data['orientation'][3])
                        ),
                        orientation_covariance=np.zeros(9, dtype=np.float64),
                        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
                        linear_acceleration_covariance=np.zeros(9, dtype=np.float64)
                    )
                    
                    writer.write(conn_imu, imu_timestamp_ns, typestore.serialize_ros1(imu_msg, imu_msg.__msgtype__))
                
                # 处理真值数据并生成模拟传感器
                if 'state' in data_frame:
                    state = data_frame['state']
                    
                    # 1. 真值位姿（用于评估）
                    header = std_msgs_Header(
                        seq=idx,
                        stamp=Time(
                            sec=int(timestamp_ns // 1000000000),
                            nanosec=int(timestamp_ns % 1000000000)
                        ),
                        frame_id='world'
                    )
                    
                    pose_msg = geometry_msgs_PoseStamped(
                        header=header,
                        pose=geometry_msgs_Pose(
                            position=geometry_msgs_Point(
                                x=float(state['position'][0]),
                                y=float(state['position'][1]),
                                z=float(state['position'][2])
                            ),
                            orientation=geometry_msgs_Quaternion(
                                w=float(state['quaternion'][0]),
                                x=float(state['quaternion'][1]),
                                y=float(state['quaternion'][2]),
                                z=float(state['quaternion'][3])
                            )
                        )
                    )
                    
                    writer.write(conn_gt_pose, timestamp_ns, typestore.serialize_ros1(pose_msg, pose_msg.__msgtype__))
                    
                    # 2. 真值速度（用于评估）
                    twist_msg = geometry_msgs_TwistStamped(
                        header=header,
                        twist=geometry_msgs_Twist(
                            linear=geometry_msgs_Vector3(
                                x=float(state['velocity'][0]),
                                y=float(state['velocity'][1]),
                                z=float(state['velocity'][2])
                            ),
                            angular=geometry_msgs_Vector3(
                                x=float(state['angular_velocity'][0]),
                                y=float(state['angular_velocity'][1]),
                                z=float(state['angular_velocity'][2])
                            )
                        )
                    )
                    
                    writer.write(conn_gt_twist, timestamp_ns, typestore.serialize_ros1(twist_msg, twist_msg.__msgtype__))
                    
                    # 3. 模拟星敏感器（姿态 + 高精度噪声）
                    noisy_quaternion = self.simulate_star_tracker(state['quaternion'])
                    
                    star_tracker_msg = geometry_msgs_QuaternionStamped(
                        header=header,
                        quaternion=geometry_msgs_Quaternion(
                            w=float(noisy_quaternion[0]),
                            x=float(noisy_quaternion[1]),
                            y=float(noisy_quaternion[2]),
                            z=float(noisy_quaternion[3])
                        )
                    )
                    
                    writer.write(conn_star_tracker, timestamp_ns, typestore.serialize_ros1(star_tracker_msg, star_tracker_msg.__msgtype__))
                    
                    # 4. 模拟轮速计（速度 + 噪声）- 使用Odometry消息
                    wheel_data = self.simulate_wheel_odometry(state['velocity'], state['angular_velocity'])
                    
                    wheel_odom_msg = nav_msgs_Odometry(
                        header=header,
                        child_frame_id='base_link',
                        pose=geometry_msgs_PoseWithCovariance(
                            pose=geometry_msgs_Pose(
                                position=geometry_msgs_Point(x=0.0, y=0.0, z=0.0),
                                orientation=geometry_msgs_Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                            ),
                            covariance=np.zeros(36, dtype=np.float64)
                        ),
                        twist=geometry_msgs_TwistWithCovariance(
                            twist=geometry_msgs_Twist(
                                linear=geometry_msgs_Vector3(
                                    x=float(wheel_data['linear_velocity'][0]),
                                    y=float(wheel_data['linear_velocity'][1]),
                                    z=float(wheel_data['linear_velocity'][2])
                                ),
                                angular=geometry_msgs_Vector3(
                                    x=float(wheel_data['angular_velocity'][0]),
                                    y=float(wheel_data['angular_velocity'][1]),
                                    z=float(wheel_data['angular_velocity'][2])
                                )
                            ),
                            covariance=np.zeros(36, dtype=np.float64)
                        )
                    )
                    
                    writer.write(conn_wheel_odom, timestamp_ns, typestore.serialize_ros1(wheel_odom_msg, wheel_odom_msg.__msgtype__))
                
                if (idx + 1) % 100 == 0:
                    print(f"  已写入 {idx + 1}/{len(dataset['data'])} 帧")
        
        print(f"ROS bag保存完成: {bag_path}")
        print(f"提示: 可以使用 rosbag info 查看topic信息")
        print(f"\nROS bag包含以下topic：")
        print(f"  - /cam0/image_raw                        (左目，供VINS使用)")
        print(f"  - /cam1/image_raw                        (右目，供VINS使用)")
        print(f"  - /nav_camera/left|right/image_raw       (可选)")
        print(f"  - /imu0                                   (IMU数据)")
        print(f"  - /star_tracker/attitude                 (姿态测量)")
        print(f"  - /wheel_odometry/twist                  (速度测量)")
        print(f"  - /ground_truth/pose                     (位姿真值)")
        print(f"  - /ground_truth/twist                    (速度真值)")


# if __name__ == "__main__":
#     # 测试代码
#     from config import (dataset_params, nav_camera_params, obstacle_camera_params, imu_params)
    
#     saver = DatasetSaver(dataset_params, nav_camera_params, 
#                         obstacle_camera_params, imu_params)
    
#     # 假设已经有采集的数据集
#     # saver.save_dataset(dataset, format_type='euroc')
#     print("数据集保存工具初始化完成")
