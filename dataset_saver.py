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
import math

class DatasetSaver:
    def __init__(self, args):
        """
        初始化数据集保存器
        """
        self.dataset_params = args['dataset_params']
        self.nav_camera_params = args['nav_camera_params']
        self.obstacle_camera_params = args['obstacle_camera_params']
        self.imu_params = args['imu_params']
        self.star_tracker_params = args.get('star_tracker_params', {
            'xy_precision_sigma': 4.85e-6,
            'z_precision_sigma': 4.85e-5
        })
        
        # 创建保存目录
        self.base_path = Path(self.dataset_params['save_path'])
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dataset_path = self.base_path / self.timestamp_str
        
        self.save_format = self.dataset_params['save_format']
        
        # 计算避障相机图像裁剪参数（从70°×70°裁剪为target FOV）
        self._compute_obstacle_crop_params()
        
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
    
    def _compute_obstacle_crop_params(self):
        """
        计算避障相机图像裁剪参数
        AirSim采集70°×70°正方形图像，根据target_fov裁剪为目标视场角
        """
        obs_params = self.obstacle_camera_params
        
        # 获取AirSim采集参数
        width, height = obs_params['resolution']  # (840, 840)
        fov = obs_params['fov']  # 70°
        
        # 获取目标视场角
        target_fov_h = obs_params.get('target_fov_h', fov)  # 默认70°
        target_fov_v = obs_params.get('target_fov_v', fov)  # 默认49°
        
        # 计算裁剪后的分辨率（使用三角函数精确计算）
        # 焦距 fx = (width/2) / tan(fov/2)
        # 目标像素 = 2 * fx * tan(target_fov/2)
        import math
        fx = (width / 2.0) / math.tan(math.radians(fov / 2.0))
        target_width = round(2 * fx * math.tan(math.radians(target_fov_h / 2.0))) if target_fov_h < fov else width
        target_height = round(2 * fx * math.tan(math.radians(target_fov_v / 2.0))) if target_fov_v < fov else height
        
        # 计算裁剪边界（居中裁剪）
        crop_left = (width - target_width) // 2
        crop_top = (height - target_height) // 2
        crop_right = crop_left + target_width
        crop_bottom = crop_top + target_height
        
        self.obstacle_crop = {
            'enabled': (target_width != width or target_height != height),
            'original_resolution': (width, height),
            'target_resolution': (target_width, target_height),
            'target_fov': (target_fov_h, target_fov_v),
            'crop_box': (crop_left, crop_top, crop_right, crop_bottom)  # (left, top, right, bottom)
        }
        
        if self.obstacle_crop['enabled']:
            print(f"避障相机图像裁剪配置:")
            print(f"  原始分辨率: {width}×{height} (FOV {fov}°×{fov}°)")
            print(f"  目标分辨率: {target_width}×{target_height} (FOV {target_fov_h}°×{target_fov_v}°)")
            print(f"  裁剪区域: ({crop_left}, {crop_top}, {crop_right}, {crop_bottom})")
        else:
            print(f"避障相机图像无需裁剪: {width}×{height}")
    
    def _crop_obstacle_image(self, img):
        """
        裁剪避障相机图像
        Args:
            img: numpy数组，图像数据
        Returns:
            裁剪后的图像
        """
        if not self.obstacle_crop['enabled']:
            return img
        
        left, top, right, bottom = self.obstacle_crop['crop_box']
        return img[top:bottom, left:right]

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
            # [已注释 - 导航相机现阶段暂时用不到]
            # if 'nav_camera' in data_frame:
            #     nav_data = data_frame['nav_camera']
            #     cam_ts = int(nav_data['timestamp'] * 1e9)
            #     
            #     # cam0: 导航左
            #     if 'left_rgb' in nav_data:
            #         filename = f"{cam_ts}.png"
            #         img_path = mav_path / "cam0" / "data" / filename
            #         cv2.imwrite(str(img_path), cv2.cvtColor(nav_data['left_rgb'], cv2.COLOR_RGB2BGR))
            #         cam_csv_files['cam0'].write(f"{cam_ts},{filename}\n")
            #     
            #     # cam1: 导航右
            #     if 'right_rgb' in nav_data:
            #         filename = f"{cam_ts}.png"
            #         img_path = mav_path / "cam1" / "data" / filename
            #         cv2.imwrite(str(img_path), cv2.cvtColor(nav_data['right_rgb'], cv2.COLOR_RGB2BGR))
            #         cam_csv_files['cam1'].write(f"{cam_ts},{filename}\n")
            
            if 'obstacle_camera' in data_frame:
                obs_data = data_frame['obstacle_camera']
                cam_ts = int(obs_data['timestamp'] * 1e9)
                
                # cam2: 避障左
                if 'left_rgb' in obs_data:
                    filename = f"{cam_ts}.png"
                    img_path = mav_path / "cam2" / "data" / filename
                    # 裁剪图像
                    img_cropped = self._crop_obstacle_image(obs_data['left_rgb'])
                    cv2.imwrite(str(img_path), cv2.cvtColor(img_cropped, cv2.COLOR_RGB2BGR))
                    cam_csv_files['cam2'].write(f"{cam_ts},{filename}\n")
                
                # cam3: 避障右
                if 'right_rgb' in obs_data:
                    filename = f"{cam_ts}.png"
                    img_path = mav_path / "cam3" / "data" / filename
                    # 裁剪图像
                    img_cropped = self._crop_obstacle_image(obs_data['right_rgb'])
                    cv2.imwrite(str(img_path), cv2.cvtColor(img_cropped, cv2.COLOR_RGB2BGR))
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
        保存传感器配置YAML文件（标准EuRoC格式）
        
        参数来源原则：
        - 外参（位置/旋转）：从 settings.json 读取，位置÷20
        - 内参（焦距/主点）：FOV 从 settings.json 读取，分辨率使用实际输出尺寸
        - 裁剪参数（target_fov）：从 config.py 读取
        
        T_cam_imu 约定：body_T_cam = T_{imu←optical}，即将相机光学坐标系中的点
        变换到 IMU/车体坐标系。与 VINS-Fusion 的 body_T_cam 一致。
        """
        settings = self.load_airsim_settings()
        vehicle = settings.get("Vehicles", {}).get("LunarRover", {}) if settings else {}
        cameras = vehicle.get("Cameras", {})
        
        # 相机映射: cam_id → (settings.json候选名, 类型, 侧)
        camera_defs = [
            ('cam0', ['nav_camera_left', 'nav_left', 'cam0'], 'nav', 'left'),
            ('cam1', ['nav_camera_right', 'nav_right', 'cam1'], 'nav', 'right'),
            ('cam2', ['obstacle_camera_left', 'obstacle_left', 'cam2'], 'obstacle', 'left'),
            ('cam3', ['obstacle_camera_right', 'obstacle_right', 'cam3'], 'obstacle', 'right'),
        ]
        
        sensor_config = {}
        print("从 settings.json 读取相机内参/外参（标准EuRoC body_T_cam 格式）...")
        
        for cam_id, name_candidates, cam_type, side in camera_defs:
            # 在 settings.json 中查找相机
            cam_data = None
            for name in name_candidates:
                if name in cameras:
                    cam_data = cameras[name]
                    break
            
            # 从 settings.json 读取 FOV 和分辨率
            if cam_data:
                cap_settings = cam_data.get("CaptureSettings", [{}])
                if isinstance(cap_settings, list) and len(cap_settings) > 0:
                    cap_settings = cap_settings[0]
                raw_width = cap_settings.get("Width", 840)
                raw_height = cap_settings.get("Height", 840)
                fov_deg = cap_settings.get("FOV_Degrees", 90)
            else:
                # 回退到 config.py
                if cam_type == 'obstacle':
                    raw_width, raw_height = self.obstacle_camera_params['resolution']
                    fov_deg = self.obstacle_camera_params['fov']
                else:
                    raw_width, raw_height = self.nav_camera_params['resolution']
                    fov_deg = self.nav_camera_params['fov']
            
            # 确定输出分辨率（避障相机需要裁剪）
            if cam_type == 'obstacle' and self.obstacle_crop['enabled']:
                out_width, out_height = self.obstacle_crop['target_resolution']
            else:
                out_width, out_height = raw_width, raw_height
            
            # 从 settings.json FOV 计算内参
            # fx 由采集 FOV 和采集宽度决定（裁剪不改变焦距）
            fx = raw_width / (2.0 * math.tan(math.radians(fov_deg / 2.0)))
            fy = fx  # 正方形像素
            cx = out_width / 2.0
            cy = out_height / 2.0
            # 注：裁剪后的 out_width/out_height 是偶数最优（对称裁剪），但此处 round 结果可能为奇数
            # cx/cy 取 out_width/2.0（浮点）确保几何中心正确
            
            # 获取外参 (body_T_cam，含光学坐标系变换)
            T_cam_imu = self.get_camera_extrinsics_from_settings(cam_type, side)
            
            sensor_config[cam_id] = {
                'camera_model': 'pinhole',
                'intrinsics': [fx, fy, cx, cy],
                'distortion_model': 'radtan',
                'distortion_coefficients': [0.0, 0.0, 0.0, 0.0],
                'T_cam_imu': T_cam_imu,
                'resolution': [out_width, out_height],
                'timeshift_cam_imu': 0.0
            }
            
            print(f"  {cam_id} ({cam_type}_{side}): FOV={fov_deg}°, "
                  f"res={out_width}x{out_height}, fx={fx:.2f}, cy={cy:.1f}")
        
        # 保存YAML
        sensor_yaml_path = mav_path / "sensor.yaml"
        with open(sensor_yaml_path, 'w') as f:
            yaml.dump(sensor_config, f, default_flow_style=False)
        
        print(f"传感器配置已保存到: {sensor_yaml_path}")
    
    def load_airsim_settings(self):
        """从用户文档目录加载 AirSim settings.json"""
        target_path = r"C:\Users\zchenkf\Documents\AirSim\settings.json"
        
        try:
            with open(target_path, 'r', encoding='utf-8') as f:
                settings_str = f.read()
                lines = settings_str.split('\n')
                clean_lines = [l for l in lines if not l.strip().startswith("//")]
                clean_json = '\n'.join(clean_lines)
                return json.loads(clean_json)
        except Exception as e:
            print(f"Warning: 无法读取 settings.json: {e}")
            return None

    def get_camera_extrinsics_from_settings(self, camera_type, side):
        """
        从 settings.json 获取相机外参：body_T_cam = T_{imu←optical}
        
        计算流程：
        1. 从 settings.json 读取相机位置 (÷20) 和旋转角（左右相机独立定义，不叠加baseline）
        2. 构造 T_vehicle_from_cambody（AirSim NED body frame）
        3. 应用 T_cambody_from_optical 将 AirSim 相机体坐标系转换为标准光学坐标系
        4. 返回 T_imu_from_optical = T_vehicle_from_cambody @ T_cambody_from_optical
           （IMU在车体原点，故 T_imu = T_vehicle）
        
        坐标系说明：
        - AirSim 相机体坐标系: X-forward, Y-right, Z-down (NED)
        - 标准光学坐标系: X-right, Y-down, Z-forward (OpenCV)
        """
        settings = self.load_airsim_settings()
        if settings is None:
            return self.get_camera_extrinsics(camera_type, side)
        
        vehicle = settings.get("Vehicles", {}).get("LunarRover", {})
        cameras = vehicle.get("Cameras", {})
        
        # 确定相机名称映射（左右相机在settings.json中独立定义完整坐标）
        if camera_type == 'obstacle':
            if side == 'left':
                names = ["obstacle_left", "obstacle_camera_left", "haz_left", "cam2"]
            else:
                names = ["obstacle_right", "obstacle_camera_right", "haz_right", "cam3"]
        else:
            if side == 'left':
                names = ["nav_left", "nav_camera_left", "cam0"]
            else:
                names = ["nav_right", "nav_camera_right", "cam1"]
        
        # 查找相机
        cam_data = None
        for name in names:
            if name in cameras:
                cam_data = cameras[name]
                break
        
        if cam_data is None:
            print(f"Warning: 在 settings.json 中未找到相机 {camera_type}_{side}，使用config.py配置")
            return self.get_camera_extrinsics(camera_type, side)
        
        # 提取并转换（除以 20）- 直接使用settings.json中的完整坐标
        x = cam_data.get("X", 0) / 20.0
        y = cam_data.get("Y", 0) / 20.0
        z = cam_data.get("Z", 0) / 20.0
        roll = math.radians(cam_data.get("Roll", 0))
        pitch = math.radians(cam_data.get("Pitch", 0))
        yaw = math.radians(cam_data.get("Yaw", 0))
        
        # 构造旋转矩阵 R_vehicle_from_cambody（Z-Y-X欧拉角：Yaw-Pitch-Roll）
        cy_r, sy_r = math.cos(yaw), math.sin(yaw)
        cp_r, sp_r = math.cos(pitch), math.sin(pitch)
        cr_r, sr_r = math.cos(roll), math.sin(roll)
        
        R = np.array([
            [cy_r*cp_r, cy_r*sp_r*sr_r - sy_r*cr_r, cy_r*sp_r*cr_r + sy_r*sr_r],
            [sy_r*cp_r, sy_r*sp_r*sr_r + cy_r*cr_r, sy_r*sp_r*cr_r - cy_r*sr_r],
            [-sp_r, cp_r*sr_r, cp_r*cr_r]
        ])
        
        # T_vehicle_from_cambody
        T_vehicle_from_cambody = np.eye(4)
        T_vehicle_from_cambody[:3, :3] = R
        T_vehicle_from_cambody[:3, 3] = [x, y, z]
        
        # AirSim camera body → optical frame 变换矩阵
        # 将光学坐标系的点变换到 AirSim 相机体坐标系:
        #   cambody_X(forward) = optical_Z(forward)
        #   cambody_Y(right)   = optical_X(right)
        #   cambody_Z(down)    = optical_Y(down)
        R_cambody_from_optical = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ])
        T_cambody_from_optical = np.eye(4)
        T_cambody_from_optical[:3, :3] = R_cambody_from_optical
        
        # body_T_cam = T_imu_from_optical = T_vehicle_from_cambody @ T_cambody_from_optical
        T_imu_from_optical = T_vehicle_from_cambody @ T_cambody_from_optical
        
        return T_imu_from_optical.flatten().tolist()

    def get_camera_extrinsics(self, camera_type, side):
        """
        获取相机外参矩阵（相对于IMU）- 从config.py的硬编码参数
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
        
        # 自航向(Yaw)角通常精度较低 (30 arcsec)
        roll_noisy = roll + np.random.randn() * self.star_tracker_params['xy_precision_sigma']
        pitch_noisy = pitch + np.random.randn() * self.star_tracker_params['xy_precision_sigma']
        yaw_noisy = yaw + np.random.randn() * self.star_tracker_params['z_precision_sigma']
        
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
            # [已注释 - 导航相机现阶段暂时用不到]
            # if 'nav_camera' in data_frame:
            #     nav_data = data_frame['nav_camera']
            #     if 'left_rgb' in nav_data:
            #         img_path = f"nav_camera_left/{idx:06d}.png"
            #         cv2.imwrite(str(self.dataset_path / img_path),
            #                   cv2.cvtColor(nav_data['left_rgb'], cv2.COLOR_RGB2BGR))
            #         frame_info['nav_left'] = img_path
            #     
            #     if 'right_rgb' in nav_data:
            #         img_path = f"nav_camera_right/{idx:06d}.png"
            #         cv2.imwrite(str(self.dataset_path / img_path),
            #                   cv2.cvtColor(nav_data['right_rgb'], cv2.COLOR_RGB2BGR))
            #         frame_info['nav_right'] = img_path
            
            if 'obstacle_camera' in data_frame:
                obs_data = data_frame['obstacle_camera']
                if 'left_rgb' in obs_data:
                    img_path = f"obstacle_camera_left/{idx:06d}.png"
                    # 裁剪图像
                    img_cropped = self._crop_obstacle_image(obs_data['left_rgb'])
                    cv2.imwrite(str(self.dataset_path / img_path),
                              cv2.cvtColor(img_cropped, cv2.COLOR_RGB2BGR))
                    frame_info['obstacle_left'] = img_path
                
                if 'right_rgb' in obs_data:
                    img_path = f"obstacle_camera_right/{idx:06d}.png"
                    # 裁剪图像
                    img_cropped = self._crop_obstacle_image(obs_data['right_rgb'])
                    cv2.imwrite(str(self.dataset_path / img_path),
                              cv2.cvtColor(img_cropped, cv2.COLOR_RGB2BGR))
                    frame_info['obstacle_right'] = img_path
            
            # 保存IMU数据
            if 'imu' in data_frame:
                frame_info['imu'] = {
                    'timestamp': data_frame['imu']['timestamp'],
                    'linear_acceleration': data_frame['imu']['linear_acceleration'].tolist(),
                    'angular_velocity': data_frame['imu']['angular_velocity'].tolist(),
                    'orientation': data_frame['imu']['orientation'].tolist()
                }
            
            # 保存轮速数据（四个车轮各自的线速度）
            if 'wheel_encoder' in data_frame:
                frame_info['wheel_encoder'] = {
                    'timestamp': data_frame['wheel_encoder']['timestamp'],
                    'wheel_linear_speeds': data_frame['wheel_encoder']['wheel_linear_speeds'].tolist()  # 四轮线速度 [FL, FR, RL, RR] (m/s)
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
        
        print(f"准备写入ROS bag: {bag_path}")
        print("注意：请勿在写入过程中强制中断程序，否则会导致bag文件损坏")
        
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
        geometry_msgs_PoseWithCovarianceStamped = typestore.types['geometry_msgs/msg/PoseWithCovarianceStamped']
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
            
            # 导航相机（可选，供额外视觉里程计使用）[已注释 - 现阶段暂时用不到]
            # conn_nav_left = writer.add_connection(
            #     '/nav_camera/left/image_raw',
            #     sensor_msgs_Image.__msgtype__,
            #     typestore=typestore
            # )
            # conn_nav_right = writer.add_connection(
            #     '/nav_camera/right/image_raw',
            #     sensor_msgs_Image.__msgtype__,
            #     typestore=typestore
            # )
            
            # IMU（供VINS和EKF使用）
            conn_imu = writer.add_connection(
                '/imu0',
                sensor_msgs_Imu.__msgtype__,
                typestore=typestore
            )
            
            # 模拟星敏感器（姿态测量，供EKF使用）
            conn_star_tracker = writer.add_connection(
                '/star_tracker/attitude',
                geometry_msgs_PoseWithCovarianceStamped.__msgtype__,
                typestore=typestore
            )
            
            # 轮速计（线速度测量，供EKF使用）
            # 注意: 修改为 Odometry 类型，包含积分后的位置信息
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
            
            # 初始化轮速计积分状态 (Dead Reckoning)
            # 假设起始位置为 (0,0,0) world frame，或者与真值初始位置对齐
            # 这里我们为了模拟 "相对位移积分"，从0开始累加
            sim_odom_pos = np.zeros(3)  # x, y, z
            last_timestamp_ns = None

            for idx, data_frame in enumerate(dataset['data']):
                # 转换时间戳为纳秒
                timestamp_ns = int(data_frame['timestamp'] * 1e9)
                
                # 导航相机图像 [已注释 - 现阶段暂时用不到]
                # if 'nav_camera' in data_frame:
                #     nav_data = data_frame['nav_camera']
                #     cam_timestamp_ns = int(nav_data['timestamp'] * 1e9)
                #     
                #     if 'left_rgb' in nav_data:
                #         img = nav_data['left_rgb']
                #         height, width = img.shape[:2]
                #         
                #         header = std_msgs_Header(
                #             seq=idx,
                #             stamp=Time(
                #                 sec=int(cam_timestamp_ns // 1000000000),
                #                 nanosec=int(cam_timestamp_ns % 1000000000)
                #             ),
                #             frame_id='nav_camera_left'
                #         )
                #         
                #         img_msg = sensor_msgs_Image(
                #             header=header,
                #             height=height,
                #             width=width,
                #             encoding='rgb8',
                #             is_bigendian=0,
                #             step=width * 3,
                #             data=np.ascontiguousarray(img).flatten()
                #         )
                #         
                #         writer.write(conn_nav_left, cam_timestamp_ns, typestore.serialize_ros1(img_msg, img_msg.__msgtype__))
                #     
                #     if 'right_rgb' in nav_data:
                #         img = nav_data['right_rgb']
                #         height, width = img.shape[:2]
                #         
                #         header = std_msgs_Header(
                #             seq=idx,
                #             stamp=Time(
                #                 sec=int(cam_timestamp_ns // 1000000000),
                #                 nanosec=int(cam_timestamp_ns % 1000000000)
                #             ),
                #             frame_id='nav_camera_right'
                #         )
                #         
                #         img_msg = sensor_msgs_Image(
                #             header=header,
                #             height=height,
                #             width=width,
                #             encoding='rgb8',
                #             is_bigendian=0,
                #             step=width * 3,
                #             data=np.ascontiguousarray(img).flatten()
                #         )
                #         
                #         writer.write(conn_nav_right, cam_timestamp_ns, typestore.serialize_ros1(img_msg, img_msg.__msgtype__))
                
                # 避障相机图像（供VINS使用）
                if 'obstacle_camera' in data_frame:
                    obs_data = data_frame['obstacle_camera']
                    cam_timestamp_ns = int(obs_data['timestamp'] * 1e9)
                    
                    if 'left_rgb' in obs_data:
                        # 裁剪图像
                        img = self._crop_obstacle_image(obs_data['left_rgb'])
                        height, width = img.shape[:2]
                        
                        header = std_msgs_Header(
                            seq=idx,
                            stamp=Time(
                                sec=int(cam_timestamp_ns // 1000000000),
                                nanosec=int(cam_timestamp_ns % 1000000000)
                            ),
                            frame_id='camera_left'
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
                        # 裁剪图像
                        img = self._crop_obstacle_image(obs_data['right_rgb'])
                        height, width = img.shape[:2]
                        
                        header = std_msgs_Header(
                            seq=idx,
                            stamp=Time(
                                sec=int(cam_timestamp_ns // 1000000000),
                                nanosec=int(cam_timestamp_ns % 1000000000)
                            ),
                            frame_id='camera_right'
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
                        frame_id='body'
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
                    
                    # 修改：EKF 期望的 Pose 输入是 PoseWithCovarianceStamped 或者是 PoseStamped
                    # robot_localization 不直接支持 QuaternionStamped 作为输入进入 pose0 接口（除非是 IMU 接口）
                    # 为了兼容性，我们将星敏感器数据封装为 PoseWithCovarianceStamped (位置设为0或不使用)
                    
                    star_tracker_msg = geometry_msgs_PoseWithCovarianceStamped(
                        header=header,
                        pose=geometry_msgs_PoseWithCovariance(
                            pose=geometry_msgs_Pose(
                                position=geometry_msgs_Point(x=0.0, y=0.0, z=0.0), # 不使用位置
                                orientation=geometry_msgs_Quaternion(
                                    w=float(noisy_quaternion[0]),
                                    x=float(noisy_quaternion[1]),
                                    y=float(noisy_quaternion[2]),
                                    z=float(noisy_quaternion[3])
                                )
                            ),
                            covariance=np.array([
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, self.star_tracker_params['xy_precision_sigma']**2, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, self.star_tracker_params['xy_precision_sigma']**2, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, self.star_tracker_params['z_precision_sigma']**2
                            ], dtype=np.float64)
                        )
                    )
                    
                    writer.write(conn_star_tracker, timestamp_ns, typestore.serialize_ros1(star_tracker_msg, star_tracker_msg.__msgtype__))
                    
                    # 4. 轮速计 (模拟)
                    # 按照用户需求：基于真值速度积分得到路径长度，结合星敏感器姿态推算位置 (Dead Reckoning)
                    # 输出 nav_msgs/Odometry，包含 Pose (积分得到) 和 Twist (模拟测量)
                if 'state' in data_frame:
                    state = data_frame['state']
                    
                    # 使用当前帧时间戳
                    wheel_timestamp_ns = timestamp_ns
                    
                    # 计算 dt
                    if last_timestamp_ns is None:
                        dt = 0.0
                    else:
                        dt = (wheel_timestamp_ns - last_timestamp_ns) / 1e9
                    last_timestamp_ns = wheel_timestamp_ns
                    
                    # --- A. 计模拟速度测量 (Twist) ---
                    # AirSim 返回的速度是 World Frame 下的 [vx, vy, vz]
                    v_world = np.array(state['velocity'])
                    
                    # 获取姿态四元数 [w, x, y, z] ---> 旋转矩阵 R_bw (World->Body)
                    q = state['quaternion'] 
                    q_w, q_x, q_y, q_z = q
                    R_wb = np.array([
                        [1-2*(q_y**2+q_z**2), 2*(q_x*q_y-q_z*q_w), 2*(q_x*q_z+q_y*q_w)],
                        [2*(q_x*q_y+q_z*q_w), 1-2*(q_x**2+q_z**2), 2*(q_y*q_z-q_x*q_w)],
                        [2*(q_x*q_z-q_y*q_w), 2*(q_y*q_z+q_x*q_w), 1-2*(q_x**2+q_y**2)]
                    ])
                    R_bw = R_wb.T
                    
                    v_body = R_bw @ v_world
                    
                    # 添加轮速计噪声 (模拟打滑和测量误差)
                    noise_sigma = 0.02 # 2cm/s 的噪声
                    vx_noisy = v_body[0] + np.random.randn() * noise_sigma
                    
                    # 非完整约束，lateral 速度为 0 (带微小噪声)
                    vy_noisy = 0.0 + np.random.randn() * 0.001 
                    
                    if np.random.rand() < 0.05: # 模拟打滑
                        vx_noisy *= 0.9

                    # --- B. 积分计算位置 (Dead Reckoning Pose) ---
                    # 轮速计只能测量前进距离 ds = vx * dt
                    ds = vx_noisy * dt
                    
                    # 使用星敏感器(或IMU融合)的姿态进行航位推算
                    # 这里直接复用上一步生成的 noisy_quaternion (模拟星敏感器读数) 作为当前姿态
                    # 注意：noisy_quaternion 是 World Frame 下的姿态
                    # 如果要进行 Dead Reckoning: P_k = P_{k-1} + R_{noisy} * [ds, 0, 0]^T
                    
                    # 将 Noisy Quaternion 转为旋转矩阵 R_noisy
                    nq_w, nq_x, nq_y, nq_z = noisy_quaternion
                    R_noisy = np.array([
                        [1-2*(nq_y**2+nq_z**2), 2*(nq_x*nq_y-nq_z*nq_w), 2*(nq_x*nq_z+nq_y*nq_w)],
                        [2*(nq_x*nq_y+nq_z*nq_w), 1-2*(nq_x**2+nq_z**2), 2*(nq_y*nq_z-nq_x*nq_w)],
                        [2*(nq_x*nq_z-nq_y*nq_w), 2*(nq_y*nq_z+nq_x*nq_w), 1-2*(nq_x**2+nq_y**2)]
                    ])
                    
                    # 计算在 World Frame 下的增量 displacement
                    d_pos_body = np.array([ds, 0.0, 0.0])
                    d_pos_world = R_noisy @ d_pos_body
                    
                    # 累加位置
                    # 注意：如果是第一帧，我们可能希望重置位置，或者就从0开始
                    sim_odom_pos += d_pos_world

                    # [DEBUG] 如果想要直接使用真值位置（加噪声），可以取消下面这段注释：
                    # pos_noise = np.random.randn(3) * 0.05  # 5cm 位置噪声
                    # sim_odom_pos = np.array(state['position']) + pos_noise
                    # ----------------------------------------------------

                    # --- C. 构建 Odometry 消息 ---
                    odom_msg = nav_msgs_Odometry(
                        header=std_msgs_Header(
                            seq=idx,
                            stamp=Time(
                                sec=int(wheel_timestamp_ns // 1000000000),
                                nanosec=int(wheel_timestamp_ns % 1000000000)
                            ),
                            frame_id='world'  # 修改：月球车在世界系下积分
                        ),
                        child_frame_id='body',
                        pose=geometry_msgs_PoseWithCovariance(
                            pose=geometry_msgs_Pose(
                                position=geometry_msgs_Point(
                                    x=float(sim_odom_pos[0]), 
                                    y=float(sim_odom_pos[1]), 
                                    z=float(sim_odom_pos[2])
                                ),
                                orientation=geometry_msgs_Quaternion(
                                    w=float(noisy_quaternion[0]),
                                    x=float(noisy_quaternion[1]),
                                    y=float(noisy_quaternion[2]),
                                    z=float(noisy_quaternion[3])
                                )
                            ),
                            covariance=np.zeros(36, dtype=np.float64) # 简化，设为0
                        ),
                        twist=geometry_msgs_TwistWithCovariance(
                            twist=geometry_msgs_Twist(
                                linear=geometry_msgs_Vector3(
                                    x=float(vx_noisy),
                                    y=float(vy_noisy),
                                    z=0.0
                                ),
                                angular=geometry_msgs_Vector3( # 轮速计通常不提供角速度，或者信度低
                                    x=0.0, y=0.0, z=0.0
                                )
                            ),
                            covariance=np.zeros(36, dtype=np.float64)
                        )
                    )
                    
                    writer.write(conn_wheel_odom, wheel_timestamp_ns, typestore.serialize_ros1(odom_msg, odom_msg.__msgtype__))
                
                if (idx + 1) % 100 == 0:
                    print(f"  已写入 {idx + 1}/{len(dataset['data'])} 帧")
            
            print("正在关闭bag文件并写入索引...")
        
        print(f"✓ ROS bag保存完成: {bag_path}")
        print(f"  文件大小: {bag_path.stat().st_size / 1024 / 1024:.2f} MB")
        print(f"\n验证bag文件:")
        print(f"  rosbag info {bag_path.name}")
        print(f"提示: 可以使用 rosbag info 查看topic信息")
        print(f"\nROS bag包含以下topic：")
        print(f"  - /cam0/image_raw                        (避障相机左目，供VINS使用)")
        print(f"  - /cam1/image_raw                        (避障相机右目，供VINS使用)")
        print(f"  - /imu0                                   (IMU数据，供VINS和EKF使用)")
        print(f"  - /star_tracker/attitude                 (姿态测量，供EKF使用)")
        print(f"  - /wheel_odometry                        (轮速计里程计，含位置积分与线速度)")
        print(f"  - /ground_truth/pose                     (位姿真值，用于评估)")
        print(f"  - /ground_truth/twist                    (速度真值，用于评估)")


# if __name__ == "__main__":
#     # 测试代码
#     from config import (dataset_params, nav_camera_params, obstacle_camera_params, imu_params)
    
#     saver = DatasetSaver(dataset_params, nav_camera_params, 
#                         obstacle_camera_params, imu_params)
    
#     # 假设已经有采集的数据集
#     # saver.save_dataset(dataset, format_type='euroc')
#     print("数据集保存工具初始化完成")
