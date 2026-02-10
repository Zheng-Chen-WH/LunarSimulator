"""
月球车数据采集环境
用于生成随机轨迹并采集双目相机、IMU、轮速计数据，供VINS-Fusion使用
"""

import airsim
import numpy as np
import time
import math
import os
from datetime import datetime
from PIL import Image
import json
import cv2
import threading
import queue
import copy

'''每次都会忘的vsc快捷键：
    打开设置脚本：ctrl+shift+P
    多行注释：ctrl+/
    关闭多行注释：选中之后再来一次ctrl+/
    多行缩进：tab
    关闭多行缩进：选中多行shift+tab'''

class LunarRoverEnv:
    def __init__(self, args):
        """
        初始化月球车环境
        """
        # 连接到AirSim
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        
        # 车辆名称（必须与settings.json中的Vehicles名称一致）
        self.vehicle_name = "LunarRover"
        
        # 启用API控制
        self.client.enableApiControl(False, self.vehicle_name)
        
        # 保存参数
        self.rover_params = args['rover_params']
        self.nav_camera_params = args['nav_camera_params']
        self.obstacle_camera_params = args['obstacle_camera_params']
        self.imu_params = args['imu_params']
        self.wheel_encoder_params = args['wheel_encoder_params']
        self.trajectory_params = args['trajectory_params']
        self.trajectory_type = args['trajectory_type']
        
        # 时间步长
        self.dt = self.trajectory_params['dt']
        
        # 轨迹数据
        self.trajectory = []
        self.current_waypoint_idx = 0
        
        # IMU偏置（模拟传感器偏置）
        self.accel_bias = np.random.randn(3) * self.imu_params['accel_bias_sigma']
        self.gyro_bias = np.random.randn(3) * self.imu_params['gyro_bias_sigma']
        
        # 轮速编码器状态
        self.wheel_encoder_counts = np.zeros(4)  # 四个车轮的编码器计数
        self.last_position = None
        self.last_dist_pos = None # 用于计算累计距离的上一位置
        self.last_encoder_pos = None # 用于计算轮速计的上一位置
        
        # 时间戳（使用 AirSim 的仿真时间戳）
        self.start_time = None
        self.last_imu_time = 0
        self.last_camera_time = 0
        self.last_encoder_time = 0
        
        # 初始化时拉手刹，确保车辆静止
        car_controls = airsim.CarControls()
        car_controls.handbrake = True
        car_controls.brake = 1.0
        car_controls.throttle = 0
        car_controls.steering = 0
        self.client.setCarControls(car_controls, self.vehicle_name)
        print(f"车辆 {self.vehicle_name} 初始化完成，手刹已拉起")

        # ==========================================
        # 同步模式配置（图像采集时暂停仿真）
        # ==========================================
        print("同步模式已启用：图像采集期间仿真将暂停，确保IMU时间戳连续")
        
    def _image_capture_thread(self):
        """
        后台图像采集线程
        """
        # 为线程创建独立的AirSim客户端连接
        #这是必须的，因为msgpack-rpc在多线程共享同一个client时可能会有问题
        thread_client = airsim.CarClient()
        thread_client.confirmConnection()
        
        while not self.img_stop_event.is_set():
            # 等待采集请求
            if self.img_request_event.wait(timeout=0.1):
                self.img_request_event.clear()
                
                # 开始采集
                try:
                    # 获取 AirSim 当前仿真时间戳
                    car_state = thread_client.getCarState(self.vehicle_name)
                    capture_timestamp = (car_state.timestamp / 1e9) - self.start_time
                    
                    # 采集导航相机 [已注释 - 现阶段暂时用不到]
                    # nav_data = self._capture_stereo_images_sync(thread_client, 'nav_camera', capture_timestamp)
                    
                    # 采集避障相机
                    obs_data = self._capture_stereo_images_sync(thread_client, 'obstacle_camera', capture_timestamp)
                    
                    # 将结果放入队列
                    result = {
                        # 'nav_camera': nav_data,  # [已注释 - 现阶段暂时用不到]
                        'obstacle_camera': obs_data
                    }
                    self.img_queue.put(result)
                    
                except Exception as e:
                    print(f"后台图像采集发生错误: {e}")

    def reset(self, seed=None):
        """
        重置环境，生成新轨迹
        """
        if seed is not None:
            np.random.seed(seed)
        
        # 重置车辆位置
        self.client.reset()
        time.sleep(1)
        self.client.enableApiControl(False, self.vehicle_name)  # 禁用API控制，允许手动驾驶
        
        # 将车辆放置在地面上方0.5米，让其自然下落
        pose = self.client.simGetVehiclePose(self.vehicle_name)
        pose.position.x_val = 0
        pose.position.y_val = 0
        pose.position.z_val = -0.5  # Z轴向下为正，负值表示在地面上方
        
        # 设置朝向（四元数，朝向正X轴）
        # pose.orientation.w_val = 1.0
        # pose.orientation.x_val = 0
        # pose.orientation.y_val = 0
        # pose.orientation.z_val = 0
        
        self.client.simSetVehiclePose(pose, True, self.vehicle_name)
        time.sleep(1)  # 等待车辆稳定
        
        # 验证车辆位置
        final_pose = self.client.simGetVehiclePose(self.vehicle_name)
        print(f"车辆已重置到原点")
        print(f"  位置: ({final_pose.position.x_val:.2f}, {final_pose.position.y_val:.2f}, {final_pose.position.z_val:.2f})")
        print(f"  Z值说明: AirSim中Z轴向下为正，当前Z={final_pose.position.z_val:.2f}m")
        
        # 重置时间（记录起始时间戳）
        # 获取 AirSim 的当前仿真时间戳（纳秒）
        initial_state = self.client.getCarState(self.vehicle_name)
        self.start_time = initial_state.timestamp / 1e9  # 转换为秒
        self.last_imu_time = 0
        self.last_camera_time = 0
        self.last_encoder_time = 0
        
        # 获取初始位置
        car_state = self.client.getCarState(self.vehicle_name)
        pos = car_state.kinematics_estimated.position
        self.last_position = np.array([pos.x_val, pos.y_val, pos.z_val])
        self.last_dist_pos = self.last_position.copy()
        self.last_encoder_pos = self.last_position.copy()
        
        # 重置传感器偏置
        self.accel_bias = np.random.randn(3) * self.imu_params['accel_bias_sigma']
        self.gyro_bias = np.random.randn(3) * self.imu_params['gyro_bias_sigma']
        self.wheel_encoder_counts = np.zeros(4)
        
        # 记录起始位置（用于计算行驶距离）
        self.start_position = self.last_position.copy()
        self.total_distance = 0.0
        
        print(f"环境重置完成，手动驾驶模式已启用")
        print(f"请使用键盘/手柄控制车辆，程序将在后台采集传感器数据")
        
        return True
    
    def generate_trajectory(self, trajectory_type='random_walk'):
        """
        生成随机轨迹
        Args:
            trajectory_type: 轨迹类型
        Returns:
            trajectory: 航点列表 [(x, y, yaw), ...]
        """
        area_range = self.trajectory_params['area_range']
        spacing = self.trajectory_params['waypoint_spacing']
        min_wp = self.trajectory_params['min_waypoints']
        max_wp = self.trajectory_params['max_waypoints']
        
        n_waypoints = np.random.randint(min_wp, max_wp + 1)
        trajectory = []
        
        if trajectory_type == 'random_walk':
            # 随机游走轨迹
            current_pos = np.array([0.0, 0.0])
            trajectory.append((0.0, 0.0, 0.0))
            
            for i in range(n_waypoints - 1):
                # 随机方向
                angle = np.random.uniform(0, 2 * np.pi)
                step = np.array([spacing * np.cos(angle), spacing * np.sin(angle)]) # 固定步长航点向量
                
                # 新位置（限制在区域内）
                new_pos = current_pos + step
                new_pos = np.clip(new_pos, area_range[0], area_range[1])
                
                # 计算期望朝向（水平面上的行驶方向，不考虑地形起伏）
                # 注意：这是路径规划的期望方向，实际车辆姿态会由物理引擎根据地形自动调整
                direction = new_pos - current_pos
                yaw = np.arctan2(direction[1], direction[0])  # 水平面投影的朝向
                
                trajectory.append((new_pos[0], new_pos[1], yaw))
                current_pos = new_pos
        
        elif trajectory_type == 'figure_eight':
            # 8字形轨迹
            t_values = np.linspace(0, 4 * np.pi, n_waypoints)
            scale = area_range[1] * 0.8
            
            for t in t_values:
                x = scale * np.sin(t) / (1 + np.cos(t)**2)
                y = scale * np.sin(t) * np.cos(t) / (1 + np.cos(t)**2)
                
                # 计算切线方向作为朝向
                dx = scale * np.cos(t) * (1 + np.cos(t)**2) - 2 * scale * np.sin(t) * np.cos(t) * (-np.sin(t))
                dy = scale * np.cos(2*t) * (1 + np.cos(t)**2) - 2 * scale * np.sin(t) * np.cos(t) * (-np.sin(t))
                yaw = np.arctan2(dy, dx)
                
                trajectory.append((x, y, yaw))
        
        elif trajectory_type == 'spiral':
            # 螺旋轨迹
            t_values = np.linspace(0, 4 * np.pi, n_waypoints)
            max_radius = area_range[1] * 0.9
            
            for t in t_values:
                r = max_radius * (t / (4 * np.pi))
                x = r * np.cos(t)
                y = r * np.sin(t)
                
                # 计算切线方向
                dx = np.cos(t) * max_radius / (4 * np.pi) - r * np.sin(t)
                dy = np.sin(t) * max_radius / (4 * np.pi) + r * np.cos(t)
                yaw = np.arctan2(dy, dx)
                
                trajectory.append((x, y, yaw))
        
        elif trajectory_type == 'grid':
            # 网格轨迹
            grid_size = int(np.sqrt(n_waypoints))
            x_points = np.linspace(area_range[0], area_range[1], grid_size)
            y_points = np.linspace(area_range[0], area_range[1], grid_size)
            
            for i, x in enumerate(x_points):
                if i % 2 == 0:
                    y_iter = y_points
                else:
                    y_iter = reversed(y_points)
                
                for y in y_iter:
                    # 朝向指向下一个点
                    if len(trajectory) > 0:
                        prev_x, prev_y, _ = trajectory[-1]
                        yaw = np.arctan2(y - prev_y, x - prev_x)
                    else:
                        yaw = 0.0
                    trajectory.append((x, y, yaw))
        
        return trajectory
    
    def get_current_state(self):
        """
        获取月球车当前状态
        Returns:
            state: 包含位置、速度、姿态等信息的字典
        """
        car_state = self.client.getCarState(self.vehicle_name)
        
        # 位置
        pos = car_state.kinematics_estimated.position
        position = np.array([pos.x_val, pos.y_val, pos.z_val])
        
        # 速度
        vel = car_state.kinematics_estimated.linear_velocity
        velocity = np.array([vel.x_val, vel.y_val, vel.z_val])
        
        # 姿态（四元数）
        orientation = car_state.kinematics_estimated.orientation
        quaternion = np.array([orientation.w_val, orientation.x_val, 
                              orientation.y_val, orientation.z_val])
        
        # 角速度
        ang_vel = car_state.kinematics_estimated.angular_velocity
        angular_velocity = np.array([ang_vel.x_val, ang_vel.y_val, ang_vel.z_val])
        
        # 时间戳（使用 AirSim 的仿真时间）
        timestamp = (car_state.timestamp / 1e9) - self.start_time
        
        state = {
            'timestamp': timestamp,
            'position': position,
            'velocity': velocity,
            'quaternion': quaternion,
            'angular_velocity': angular_velocity,
            'speed': car_state.speed,
            'gear': car_state.gear
        }
        
        return state
    
    def _capture_stereo_images_sync(self, client, camera_name_prefix, timestamp):
        """
        [内部方法] 同步采集双目图像（RGB + 深度）
        由后台线程调用
        Args:
            client: 线程专用的AirSim客户端
            camera_name_prefix: 'nav_camera' 或 'obstacle_camera'
            timestamp: 采集时间戳
        Returns:
            images: 包含左右相机RGB图像和深度图的字典
        """
        # 请求左右相机的RGB图像
        rgb_requests = [
            airsim.ImageRequest(f"{camera_name_prefix}_left", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest(f"{camera_name_prefix}_right", airsim.ImageType.Scene, False, False),
        ]
        
        # 请求左右相机的深度图像
        depth_requests = [
            airsim.ImageRequest(f"{camera_name_prefix}_left", airsim.ImageType.DepthPlanar, True, False),
            airsim.ImageRequest(f"{camera_name_prefix}_right", airsim.ImageType.DepthPlanar, True, False),
        ]
        
        try:
            # 分别获取RGB和深度图像
            rgb_responses = client.simGetImages(rgb_requests, self.vehicle_name)
            depth_responses = client.simGetImages(depth_requests, self.vehicle_name)
            
            # 合并响应
            responses = rgb_responses + depth_responses
            
        except Exception as e:
            print(f"获取图像失败: {e}")
            raise
        
        images = {}
        
        # 目标分辨率（软件超采样后的输出分辨率）
        target_width = 840
        target_height = 840
        
        # 左相机RGB
        if responses[0].width > 0:
            img_left = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            img_left = img_left.reshape(responses[0].height, responses[0].width, 3)
            # 如果采集分辨率高于目标分辨率，进行缩小（软件超采样）
            if responses[0].width > target_width:
                img_left = cv2.resize(img_left, (target_width, target_height), interpolation=cv2.INTER_AREA)
            images['left_rgb'] = img_left
        
        # 右相机RGB
        if responses[1].width > 0:
            img_right = np.frombuffer(responses[1].image_data_uint8, dtype=np.uint8)
            img_right = img_right.reshape(responses[1].height, responses[1].width, 3)
            # 如果采集分辨率高于目标分辨率，进行缩小（软件超采样）
            if responses[1].width > target_width:
                img_right = cv2.resize(img_right, (target_width, target_height), interpolation=cv2.INTER_AREA)
            images['right_rgb'] = img_right
        
        # 左相机深度
        if responses[2].width > 0:
            depth_left = airsim.list_to_2d_float_array(responses[2].image_data_float, 
                                                    responses[2].width, responses[2].height)
            images['left_depth'] = depth_left
        
        # 右相机深度
        if responses[3].width > 0:
            depth_right = airsim.list_to_2d_float_array(responses[3].image_data_float,
                                                        responses[3].width, responses[3].height)
            images['right_depth'] = depth_right
        
        # 使用传入的 AirSim 仿真时间戳
        # 注意：responses[0].time_stamp 使用系统时钟，不受 simPause 影响
        # 因此必须使用传入的 timestamp（来自 get_current_state 的仿真时间）
        images['timestamp'] = timestamp
        
        return images

    def capture_stereo_images(self, camera_name_prefix):
        """
        [弃用/兼容] 原始同步采集方法
        """
        # 使用 AirSim 仿真时间戳
        car_state = self.client.getCarState(self.vehicle_name)
        timestamp = (car_state.timestamp / 1e9) - self.start_time
        return self._capture_stereo_images_sync(self.client, camera_name_prefix, timestamp)
    
    def get_imu_data(self):
        """
        获取IMU数据（加速度、角速度）
        Returns:
            imu_data: 包含加速度和角速度的字典
        """
        imu_data_raw = self.client.getImuData(imu_name='imu', vehicle_name=self.vehicle_name)
        
        # 线加速度（加入噪声和偏置）
        accel = imu_data_raw.linear_acceleration
        linear_accel = np.array([accel.x_val, accel.y_val, accel.z_val])
        linear_accel += self.accel_bias
        linear_accel += np.random.randn(3) * self.imu_params['accel_noise_sigma']
        
        # 角速度（加入噪声和偏置）
        gyro = imu_data_raw.angular_velocity
        angular_vel = np.array([gyro.x_val, gyro.y_val, gyro.z_val])
        angular_vel += self.gyro_bias
        angular_vel += np.random.randn(3) * self.imu_params['gyro_noise_sigma']
        
        # 姿态（四元数）
        orientation = imu_data_raw.orientation
        quaternion = np.array([orientation.w_val, orientation.x_val,
                              orientation.y_val, orientation.z_val])
        
        imu_data = {
            'timestamp': (imu_data_raw.time_stamp / 1e9) - self.start_time,
            'linear_acceleration': linear_accel,
            'angular_velocity': angular_vel,
            'orientation': quaternion
        }
        
        return imu_data
    
    def get_wheel_encoder_data(self, dt=None):
        """
        计算轮速编码器数据（四个车轮各自的线速度）
        Args:
            dt: 自上次采样以来的时间间隔 (s)
        Returns:
            encoder_data: 包含四个车轮线速度的字典
        
        注意：
        - 轮速计测量每个车轮的线速度（m/s）
        - 转向时，外侧轮速度快，内侧轮速度慢
        - 车轮布局：[front_left, front_right, rear_left, rear_right]
        - 根据对接团队需求，输出线速度（不输出角速度）
        """
        # handling default dt if not provided (fallback)
        if dt is None:
            dt = self.dt
        if dt <= 0:
            dt = self.dt

        current_state = self.get_current_state()
        current_pos = current_state['position']
        
        if self.last_encoder_pos is None:
            self.last_encoder_pos = current_pos
            wheel_linear_speeds = np.zeros(4)
        else:
            # 计算车体中心的线速度（水平面）
            displacement = np.linalg.norm(current_pos[:2] - self.last_encoder_pos[:2])  # 只看XY平面
            linear_speed = displacement / dt  # m/s
            
            # 获取车体角速度（yaw rate，绕Z轴旋转）
            angular_velocity = current_state['angular_velocity']
            yaw_rate = angular_velocity[2]  # rad/s，正值表示逆时针旋转
            
            # 车辆参数
            track_width = self.rover_params['track_width']  # 左右轮距
            
            # 使用差速转向模型计算左右轮的线速度
            left_linear_speed = linear_speed - yaw_rate * (track_width / 2.0)
            right_linear_speed = linear_speed + yaw_rate * (track_width / 2.0)
            
            # 四轮线速度：[front_left, front_right, rear_left, rear_right] (m/s)
            wheel_linear_speeds = np.array([
                left_linear_speed,   # 前左
                right_linear_speed,  # 前右
                left_linear_speed,   # 后左
                right_linear_speed   # 后右
            ])
            
            # 前轮是转向轮
            wheel_linear_speeds[0] *= 0.98  # 前左
            wheel_linear_speeds[1] *= 0.98  # 前右
            
            # 添加噪声
            for i in range(4):
                if abs(wheel_linear_speeds[i]) > 1e-6:
                    noise = np.random.randn() * self.wheel_encoder_params['noise_std'] * abs(wheel_linear_speeds[i])
                    wheel_linear_speeds[i] += noise
            
            # 模拟偶尔的打滑
            if np.random.rand() < self.wheel_encoder_params['slip_probability']:
                slip_factor = np.random.uniform(*self.wheel_encoder_params['slip_factor_range'])
                wheel_idx = np.random.randint(0, 4)
                wheel_linear_speeds[wheel_idx] *= slip_factor
            
            self.last_encoder_pos = current_pos
        
        encoder_data = {
            'timestamp': current_state['timestamp'],
            'wheel_linear_speeds': wheel_linear_speeds,
            'dt': dt
        }
        
        return encoder_data
    
    def quaternion_to_yaw(self, q):
        """
        从四元数提取yaw角（水平朝向）
        
        Args:
            q: [w, x, y, z] 完整的姿态四元数（包含pitch、roll、yaw）
        
        Returns:
            yaw: 水平偏航角（弧度），即车辆在XY平面上的朝向
        
        注意：
        - 输入的四元数包含了车辆在地形上的完整姿态（pitch、roll、yaw）
        - 此函数只提取其中的yaw分量（绕Z轴的旋转）
        - pitch（俯仰）和roll（横滚）由地形决定，在此被忽略
        """
        w, x, y, z = q
        # 提取yaw角：绕世界坐标系Z轴的旋转分量
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
    
    def normalize_angle(self, angle):
        """
        将角度规范化到[-pi, pi]
        """
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def step(self):
        """
        同步模式：执行一个时间步，返回采集的传感器数据
        在图像采集期间暂停仿真，确保IMU时间戳连续
        Returns:
            data: 包含所有传感器数据的字典
        """
        # 获取当前状态（每个step只调用一次 AirSim API 以保证数据一致性）
        current_state = self.get_current_state()
        current_time = current_state['timestamp']
        current_pos = current_state['position']
        
        # 更新累计距离
        if self.last_dist_pos is not None:
            displacement = np.linalg.norm(current_pos[:2] - self.last_dist_pos[:2])
            self.total_distance += displacement
        self.last_dist_pos = current_pos

        data = {
            'timestamp': current_time,
            'state': current_state
        }
        
        # IMU数据（高频采集）
        imu_rate = 1.0 / self.imu_params['sampling_rate']
        if current_time - self.last_imu_time >= imu_rate:
            data['imu'] = self.get_imu_data()
            self.last_imu_time = current_time
        
        # 相机数据（低频采集，同步模式）
        camera_rate = 1.0 / self.nav_camera_params['fps']
        if current_time - self.last_camera_time >= camera_rate:
            # 暂停仿真，采集图像
            self.client.simPause(True)
            
            try:
                # 同步采集图像（仿真已暂停，时间不会流逝）
                data['obstacle_camera'] = self._capture_stereo_images_sync(
                    self.client, 'obstacle_camera', current_time
                )
                self.last_camera_time = current_time
            except Exception as e:
                print(f"图像采集失败: {e}")
            finally:
                # 恢复仿真
                self.client.simPause(False)
        
        # 轮速编码器数据
        encoder_rate = 1.0 / self.wheel_encoder_params['sampling_rate']
        if current_time - self.last_encoder_time >= encoder_rate:
            dt_encoder = current_time - self.last_encoder_time
            if self.last_encoder_time == 0:
                dt_encoder = self.dt
            data['wheel_encoder'] = self.get_wheel_encoder_data(dt_encoder)
            self.last_encoder_time = current_time
        
        return data
    
    def collect_data(self, duration=None, distance=None, max_frames=None):
        """
        手动驾驶时持续采集传感器数据
        
        Args:
            duration: 采集时长（秒），None表示不限制
            distance: 采集距离（米），None表示不限制
            max_frames: 最大采集帧数，None表示不限制
        
        Returns:
            dataset: 采集的所有数据
        
        注意：至少需要指定一个停止条件，否则会一直采集
        """
        # 检查至少有一个停止条件
        if duration is None and distance is None and max_frames is None:
            print("警告：未指定停止条件，将采集1000帧数据")
            max_frames = 1000
        
        dataset = {
            'start_time': time.time(),
            'data': [],
            'metadata': {
                'target_duration': duration,
                'target_distance': distance,
                'max_frames': max_frames
            }
        }
        
        print(f"\n========== 开始数据采集 ==========")
        print(f"停止条件：")
        if duration is not None:
            print(f"  - 时长: {duration:.1f}秒")
        if distance is not None:
            print(f"  - 距离: {distance:.1f}米")
        if max_frames is not None:
            print(f"  - 帧数: {max_frames}帧")
        print(f"请手动驾驶车辆，程序将自动采集数据...")
        print(f"==================================\n")
        
        frame_count = 0
        last_print_time = time.time()
        
        while True:
            # 采集传感器数据
            sensor_data = self.step()
            dataset['data'].append(sensor_data)
            frame_count += 1
            
            # 使用 step() 中已经获取并计算好的数据
            current_time = sensor_data['timestamp']
            current_distance = self.total_distance
            
            # 每2秒打印一次进度
            if time.time() - last_print_time > 2.0:
                # 使用已经采集到的数据，避免重复请求 API
                current_state = sensor_data['state']
                pos = current_state['position']
                speed = current_state['speed']
                
                # 计算俯视图下的车体与 X 轴夹角 (Yaw)
                yaw_rad = self.quaternion_to_yaw(current_state['quaternion'])
                yaw_deg = np.degrees(yaw_rad)
                
                print(f"[进度] 时长: {current_time:.1f}s | 距离: {current_distance:.4f}m | "
                      f"航向角(Yaw): {yaw_deg:.2f}° | 位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) | "
                      f"速度: {speed:.4f}m/s")
                last_print_time = time.time()
            
            # 检查停止条件
            should_stop = False
            stop_reason = ""
            
            if duration is not None and current_time >= duration:
                should_stop = True
                stop_reason = f"达到目标时长 {duration:.1f}秒"
            
            if distance is not None and current_distance >= distance:
                should_stop = True
                stop_reason = f"达到目标距离 {distance:.1f}米"
            
            if max_frames is not None and frame_count >= max_frames:
                should_stop = True
                stop_reason = f"达到最大帧数 {max_frames}帧"
            
            if should_stop:
                break
            
            # 控制采集频率（使用配置的dt）
            time.sleep(self.dt)
        
        # 记录最终统计信息（使用 AirSim 的仿真时间）
        final_state = self.get_current_state()
        dataset['metadata']['actual_duration'] = final_state['timestamp']
        dataset['metadata']['actual_distance'] = self.total_distance
        dataset['metadata']['actual_frames'] = frame_count
        
        print(f"\n========== 数据采集完成 ==========")
        print(f"停止原因: {stop_reason}")
        print(f"实际时长: {dataset['metadata']['actual_duration']:.1f}秒")
        print(f"实际距离: {dataset['metadata']['actual_distance']:.1f}米")
        print(f"实际帧数: {dataset['metadata']['actual_frames']}帧")
        print(f"==================================\n")
        
        return dataset


# if __name__ == "__main__":
#     # 测试代码
#     from config import (rover_params, nav_camera_params, obstacle_camera_params,
#                        imu_params, wheel_encoder_params, trajectory_params)
    
#     env = LunarRoverEnv(rover_params, nav_camera_params, obstacle_camera_params,
#                        imu_params, wheel_encoder_params, trajectory_params)
    
#     # 重置环境，生成轨迹
#     env.reset(seed=42)
    
#     # 运行轨迹并采集数据
#     dataset = env.run_trajectory()
    
#     print(f"\n数据采集完成！")
#     print(f"轨迹点数: {len(dataset['trajectory'])}")
#     print(f"数据帧数: {len(dataset['data'])}")
