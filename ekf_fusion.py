"""
扩展卡尔曼滤波器(EKF)多传感器融合模块
融合来源：
1. VINS-Fusion输出（位置+姿态）
2. 星敏感器（姿态）
3. 轮速计（速度）

状态向量 [15维]:
    - 位置: p_x, p_y, p_z
    - 速度: v_x, v_y, v_z
    - 姿态四元数: q_w, q_x, q_y, q_z
    - 陀螺仪偏置: b_gx, b_gy, b_gz
    - 加速度计偏置: b_ax, b_ay, b_az (可选，简化版不使用)

作者: GitHub Copilot
日期: 2026-01-13
"""

import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, List
import time


@dataclass
class EKFState:
    """EKF状态"""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # [w, x, y, z]
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    timestamp: float = 0.0
    
    def to_vector(self) -> np.ndarray:
        """转换为状态向量 [13维: pos(3) + vel(3) + quat(4) + gyro_bias(3)]"""
        return np.concatenate([
            self.position, self.velocity, self.quaternion, self.gyro_bias
        ])
    
    @classmethod
    def from_vector(cls, x: np.ndarray, timestamp: float = 0.0) -> 'EKFState':
        """从状态向量构造"""
        return cls(
            position=x[0:3],
            velocity=x[3:6],
            quaternion=x[6:10] / np.linalg.norm(x[6:10]),  # 归一化四元数
            gyro_bias=x[10:13],
            timestamp=timestamp
        )


class MultiSensorEKF:
    """
    多传感器扩展卡尔曼滤波器
    
    融合策略：
    - 预测：使用IMU数据进行状态预测
    - 更新：
        1. VINS位姿更新（位置+姿态）
        2. 星敏感器更新（仅姿态）
        3. 轮速计更新（仅速度）
    """
    
    def __init__(self, config: Optional[Dict] = None):
        """
        初始化EKF
        
        Args:
            config: 配置参数字典
        """
        self.config = config or self._default_config()
        
        # 状态维度
        self.state_dim = 13  # pos(3) + vel(3) + quat(4) + gyro_bias(3)
        
        # 初始化状态
        self.state = EKFState()
        
        # 协方差矩阵
        self.P = np.eye(self.state_dim) * self.config['initial_covariance']
        
        # 过程噪声协方差
        self.Q = self._build_process_noise()
        
        # 重力向量（月球重力 1.62 m/s^2，向下）
        self.gravity = np.array([0, 0, self.config['gravity']])
        
        # 上次预测时间
        self.last_predict_time = None
        
        # 是否已初始化
        self.initialized = False
        
        # 融合统计
        self.stats = {
            'predict_count': 0,
            'vins_update_count': 0,
            'star_tracker_update_count': 0,
            'wheel_odom_update_count': 0
        }
        
    def _default_config(self) -> Dict:
        """默认配置"""
        return {
            # 初始协方差
            'initial_covariance': 0.1,
            
            # 重力（月球）
            'gravity': 1.62,
            
            # 过程噪声参数
            'process_noise': {
                'position': 0.01,      # 位置过程噪声
                'velocity': 0.1,       # 速度过程噪声
                'attitude': 0.001,     # 姿态过程噪声
                'gyro_bias': 0.0001,   # 陀螺仪偏置随机游走
            },
            
            # VINS测量噪声
            'vins_noise': {
                'position': 0.05,      # 位置测量噪声 (m)
                'attitude': 0.01,      # 姿态测量噪声 (rad)
            },
            
            # 星敏感器测量噪声
            'star_tracker_noise': {
                'attitude': 1.7e-5,    # 约0.001度
            },
            
            # 轮速计测量噪声
            'wheel_odom_noise': {
                'velocity': 0.05,      # 速度测量噪声 (m/s)
            },
            
            # IMU参数
            'imu': {
                'accel_noise': 0.02,
                'gyro_noise': 0.001,
            }
        }
    
    def _build_process_noise(self) -> np.ndarray:
        """构建过程噪声协方差矩阵"""
        Q = np.zeros((self.state_dim, self.state_dim))
        pn = self.config['process_noise']
        
        # 位置噪声
        Q[0:3, 0:3] = np.eye(3) * pn['position']**2
        # 速度噪声
        Q[3:6, 3:6] = np.eye(3) * pn['velocity']**2
        # 姿态噪声（四元数的3个自由度）
        Q[6:10, 6:10] = np.eye(4) * pn['attitude']**2
        # 陀螺仪偏置随机游走
        Q[10:13, 10:13] = np.eye(3) * pn['gyro_bias']**2
        
        return Q
    
    def initialize(self, position: np.ndarray, velocity: np.ndarray, 
                   quaternion: np.ndarray, timestamp: float):
        """
        初始化EKF状态
        
        Args:
            position: 初始位置 [x, y, z]
            velocity: 初始速度 [vx, vy, vz]
            quaternion: 初始姿态四元数 [w, x, y, z]
            timestamp: 时间戳
        """
        self.state = EKFState(
            position=np.array(position),
            velocity=np.array(velocity),
            quaternion=np.array(quaternion) / np.linalg.norm(quaternion),
            gyro_bias=np.zeros(3),
            timestamp=timestamp
        )
        
        self.last_predict_time = timestamp
        self.initialized = True
        
        print(f"[EKF] 初始化完成:")
        print(f"  位置: {self.state.position}")
        print(f"  速度: {self.state.velocity}")
        print(f"  姿态: {self.state.quaternion}")
    
    def predict(self, imu_data: Dict):
        """
        使用IMU数据进行状态预测
        
        Args:
            imu_data: IMU数据字典，包含：
                - timestamp: 时间戳
                - linear_acceleration: 线性加速度 [ax, ay, az]
                - angular_velocity: 角速度 [wx, wy, wz]
        """
        if not self.initialized:
            return
        
        timestamp = imu_data['timestamp']
        accel = np.array(imu_data['linear_acceleration'])
        gyro = np.array(imu_data['angular_velocity'])
        
        # 计算时间间隔
        if self.last_predict_time is None:
            self.last_predict_time = timestamp
            return
        
        dt = timestamp - self.last_predict_time
        if dt <= 0:
            return
        
        # 去除陀螺仪偏置
        gyro_corrected = gyro - self.state.gyro_bias
        
        # 获取当前旋转矩阵
        R = Rotation.from_quat([
            self.state.quaternion[1],  # x
            self.state.quaternion[2],  # y
            self.state.quaternion[3],  # z
            self.state.quaternion[0],  # w (scipy用[x,y,z,w]格式)
        ]).as_matrix()
        
        # 将加速度从IMU坐标系转换到世界坐标系
        accel_world = R @ accel
        
        # 移除重力
        accel_world_no_gravity = accel_world - self.gravity
        
        # 状态预测
        # 位置更新: p = p + v*dt + 0.5*a*dt^2
        self.state.position = (self.state.position + 
                               self.state.velocity * dt + 
                               0.5 * accel_world_no_gravity * dt**2)
        
        # 速度更新: v = v + a*dt
        self.state.velocity = self.state.velocity + accel_world_no_gravity * dt
        
        # 姿态更新（使用角速度积分）
        omega_mag = np.linalg.norm(gyro_corrected)
        if omega_mag > 1e-10:
            # 四元数微分方程积分
            delta_angle = gyro_corrected * dt
            delta_rot = Rotation.from_rotvec(delta_angle)
            current_rot = Rotation.from_quat([
                self.state.quaternion[1],
                self.state.quaternion[2],
                self.state.quaternion[3],
                self.state.quaternion[0],
            ])
            new_rot = current_rot * delta_rot
            q_scipy = new_rot.as_quat()  # [x, y, z, w]
            self.state.quaternion = np.array([q_scipy[3], q_scipy[0], q_scipy[1], q_scipy[2]])
        
        # 归一化四元数
        self.state.quaternion = self.state.quaternion / np.linalg.norm(self.state.quaternion)
        
        # 协方差预测: P = F*P*F' + Q
        F = self._compute_jacobian_F(dt, R, accel)
        self.P = F @ self.P @ F.T + self.Q * dt
        
        self.state.timestamp = timestamp
        self.last_predict_time = timestamp
        self.stats['predict_count'] += 1
    
    def _compute_jacobian_F(self, dt: float, R: np.ndarray, accel: np.ndarray) -> np.ndarray:
        """
        计算状态转移雅可比矩阵
        """
        F = np.eye(self.state_dim)
        
        # 位置对速度的偏导
        F[0:3, 3:6] = np.eye(3) * dt
        
        # 速度对姿态的偏导（近似）
        # 这里简化处理，实际应该更精确计算
        
        return F
    
    def update_vins(self, vins_pose: Dict):
        """
        使用VINS位姿进行更新
        
        Args:
            vins_pose: VINS输出，包含：
                - timestamp: 时间戳
                - position: 位置 [x, y, z]
                - quaternion: 姿态四元数 [w, x, y, z]
        """
        if not self.initialized:
            return
        
        # 测量值
        z_pos = np.array(vins_pose['position'])
        z_quat = np.array(vins_pose['quaternion'])
        z_quat = z_quat / np.linalg.norm(z_quat)
        
        # 位置更新
        H_pos = np.zeros((3, self.state_dim))
        H_pos[0:3, 0:3] = np.eye(3)
        
        R_pos = np.eye(3) * self.config['vins_noise']['position']**2
        
        y_pos = z_pos - self.state.position  # 残差
        S_pos = H_pos @ self.P @ H_pos.T + R_pos
        K_pos = self.P @ H_pos.T @ np.linalg.inv(S_pos)
        
        # 更新状态
        x = self.state.to_vector()
        x = x + K_pos @ y_pos
        
        # 更新协方差
        self.P = (np.eye(self.state_dim) - K_pos @ H_pos) @ self.P
        
        # 姿态更新（使用四元数误差）
        # 计算姿态误差（四元数乘法）
        q_error = self._quaternion_multiply(
            z_quat, 
            self._quaternion_inverse(self.state.quaternion)
        )
        
        # 将四元数误差转换为轴角
        angle_error = 2 * np.arctan2(
            np.linalg.norm(q_error[1:4]),
            q_error[0]
        ) * q_error[1:4] / (np.linalg.norm(q_error[1:4]) + 1e-10)
        
        # 简化的姿态更新
        H_att = np.zeros((3, self.state_dim))
        H_att[0:3, 6:9] = np.eye(3)  # 近似
        
        R_att = np.eye(3) * self.config['vins_noise']['attitude']**2
        
        # 使用SLERP进行四元数插值更新
        alpha = 0.3  # 更新权重
        x[6:10] = self._quaternion_slerp(self.state.quaternion, z_quat, alpha)
        
        self.state = EKFState.from_vector(x, vins_pose['timestamp'])
        self.stats['vins_update_count'] += 1
    
    def update_star_tracker(self, star_tracker_data: Dict):
        """
        使用星敏感器姿态进行更新
        
        Args:
            star_tracker_data: 星敏感器数据，包含：
                - timestamp: 时间戳
                - quaternion: 姿态四元数 [w, x, y, z]
        """
        if not self.initialized:
            return
        
        z_quat = np.array(star_tracker_data['quaternion'])
        z_quat = z_quat / np.linalg.norm(z_quat)
        
        # 星敏感器精度很高，给予较大权重
        alpha = 0.5  # 星敏感器更新权重
        
        # 使用SLERP进行四元数插值
        new_quat = self._quaternion_slerp(self.state.quaternion, z_quat, alpha)
        self.state.quaternion = new_quat
        
        # 更新协方差（减小姿态不确定性）
        attitude_cov_reduction = 0.5
        self.P[6:10, 6:10] *= attitude_cov_reduction
        
        self.stats['star_tracker_update_count'] += 1
    
    def update_wheel_odometry(self, wheel_odom_data: Dict):
        """
        使用轮速计速度进行更新
        
        Args:
            wheel_odom_data: 轮速计数据，包含：
                - timestamp: 时间戳
                - linear_velocity: 线性速度 [vx, vy, vz]
        """
        if not self.initialized:
            return
        
        # 轮速计测量的是车体坐标系下的速度
        z_vel_body = np.array(wheel_odom_data['linear_velocity'])
        
        # 转换到世界坐标系
        R = Rotation.from_quat([
            self.state.quaternion[1],
            self.state.quaternion[2],
            self.state.quaternion[3],
            self.state.quaternion[0],
        ]).as_matrix()
        z_vel_world = R @ z_vel_body
        
        # 速度更新
        H_vel = np.zeros((3, self.state_dim))
        H_vel[0:3, 3:6] = np.eye(3)
        
        R_vel = np.eye(3) * self.config['wheel_odom_noise']['velocity']**2
        
        y_vel = z_vel_world - self.state.velocity
        S_vel = H_vel @ self.P @ H_vel.T + R_vel
        K_vel = self.P @ H_vel.T @ np.linalg.inv(S_vel)
        
        # 更新状态
        x = self.state.to_vector()
        x = x + K_vel @ y_vel
        
        # 更新协方差
        self.P = (np.eye(self.state_dim) - K_vel @ H_vel) @ self.P
        
        self.state = EKFState.from_vector(x, wheel_odom_data['timestamp'])
        self.stats['wheel_odom_update_count'] += 1
    
    def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """四元数乘法 [w, x, y, z]"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def _quaternion_inverse(self, q: np.ndarray) -> np.ndarray:
        """四元数逆 [w, x, y, z]"""
        return np.array([q[0], -q[1], -q[2], -q[3]]) / np.dot(q, q)
    
    def _quaternion_slerp(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """四元数球面线性插值"""
        # 确保四元数在同一半球
        dot = np.dot(q1, q2)
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        if dot > 0.9995:
            # 线性插值（角度太小）
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        theta_0 = np.arccos(dot)
        theta = theta_0 * t
        
        q2_perp = q2 - dot * q1
        q2_perp = q2_perp / np.linalg.norm(q2_perp)
        
        result = q1 * np.cos(theta) + q2_perp * np.sin(theta)
        return result / np.linalg.norm(result)
    
    def get_state(self) -> Dict:
        """获取当前状态"""
        return {
            'timestamp': self.state.timestamp,
            'position': self.state.position.copy(),
            'velocity': self.state.velocity.copy(),
            'quaternion': self.state.quaternion.copy(),
            'gyro_bias': self.state.gyro_bias.copy(),
            'covariance_trace': np.trace(self.P)
        }
    
    def get_stats(self) -> Dict:
        """获取融合统计信息"""
        return self.stats.copy()


class SensorDataBuffer:
    """
    传感器数据缓冲区
    用于时间同步和数据管理
    """
    
    def __init__(self, buffer_size: int = 1000):
        self.buffer_size = buffer_size
        self.imu_buffer: List[Dict] = []
        self.vins_buffer: List[Dict] = []
        self.star_tracker_buffer: List[Dict] = []
        self.wheel_odom_buffer: List[Dict] = []
        
    def add_imu(self, data: Dict):
        self.imu_buffer.append(data)
        if len(self.imu_buffer) > self.buffer_size:
            self.imu_buffer.pop(0)
    
    def add_vins(self, data: Dict):
        self.vins_buffer.append(data)
        if len(self.vins_buffer) > self.buffer_size:
            self.vins_buffer.pop(0)
    
    def add_star_tracker(self, data: Dict):
        self.star_tracker_buffer.append(data)
        if len(self.star_tracker_buffer) > self.buffer_size:
            self.star_tracker_buffer.pop(0)
    
    def add_wheel_odom(self, data: Dict):
        self.wheel_odom_buffer.append(data)
        if len(self.wheel_odom_buffer) > self.buffer_size:
            self.wheel_odom_buffer.pop(0)
    
    def get_imu_between(self, t_start: float, t_end: float) -> List[Dict]:
        """获取指定时间范围内的IMU数据"""
        return [d for d in self.imu_buffer if t_start <= d['timestamp'] <= t_end]
    
    def get_nearest(self, buffer: List[Dict], timestamp: float) -> Optional[Dict]:
        """获取最接近指定时间的数据"""
        if not buffer:
            return None
        
        nearest = min(buffer, key=lambda x: abs(x['timestamp'] - timestamp))
        return nearest


if __name__ == "__main__":
    # 测试EKF
    ekf = MultiSensorEKF()
    
    # 初始化
    ekf.initialize(
        position=[0, 0, 0],
        velocity=[0, 0, 0],
        quaternion=[1, 0, 0, 0],
        timestamp=0.0
    )
    
    # 模拟IMU预测
    for i in range(100):
        imu_data = {
            'timestamp': i * 0.01,
            'linear_acceleration': [0.0, 0.0, 1.62],  # 月球重力
            'angular_velocity': [0.0, 0.0, 0.0]
        }
        ekf.predict(imu_data)
    
    print(f"最终状态: {ekf.get_state()}")
    print(f"统计信息: {ekf.get_stats()}")
