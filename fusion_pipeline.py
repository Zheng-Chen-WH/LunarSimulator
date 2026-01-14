"""
多传感器融合定位主管道
整合VINS-Fusion和EKF实现完整的融合定位系统

融合架构：
┌─────────────────────────────────────────────────────────────┐
│                    传感器数据输入                            │
├─────────────────────────────────────────────────────────────┤
│  避障相机(双目) ──┐                                          │
│                   ├──► VINS-Fusion ──► 位姿估计 ──┐         │
│  IMU ────────────┘                                │         │
│                                                   ▼         │
│  星敏感器(模拟) ────────────────────────────► EKF ──► 最终位姿 │
│                                                   ▲         │
│  轮速计(模拟) ────────────────────────────────────┘         │
│                                                             │
│  导航相机(双目) ──► [可选] 额外视觉里程计                     │
└─────────────────────────────────────────────────────────────┘

作者: GitHub Copilot
日期: 2026-01-13
"""

import os
import sys
import json
import time
import argparse
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from datetime import datetime

# 导入自定义模块
from ekf_fusion import MultiSensorEKF, SensorDataBuffer
from vins_docker_interface import VINSDockerInterface, VINSConfig, VINSResultParser
from config import (
    nav_camera_params, obstacle_camera_params, 
    imu_params, dataset_params
)


@dataclass
class FusionConfig:
    """融合系统配置"""
    # 数据集路径
    dataset_path: str = ""
    
    # 输出路径
    output_path: str = "./fusion_output"
    
    # VINS配置
    use_vins: bool = True
    vins_config_file: str = "/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml"
    
    # EKF配置
    ekf_config: Dict = field(default_factory=dict)
    
    # 传感器融合开关
    use_star_tracker: bool = True
    use_wheel_odometry: bool = True
    
    # 时间同步阈值（秒）
    sync_threshold: float = 0.02
    
    # 是否保存中间结果
    save_intermediate: bool = True
    
    # 可视化选项
    enable_visualization: bool = True


class RosbagDataLoader:
    """
    Rosbag数据加载器
    用于从rosbag中读取传感器数据
    """
    
    def __init__(self, bag_path: str):
        self.bag_path = bag_path
        self.data_loaded = False
        
        # 数据存储
        self.imu_data: List[Dict] = []
        self.star_tracker_data: List[Dict] = []
        self.wheel_odom_data: List[Dict] = []
        self.ground_truth_pose: List[Dict] = []
        self.ground_truth_twist: List[Dict] = []
        
    def load(self) -> bool:
        """加载rosbag数据"""
        try:
            from rosbags.rosbag1 import Reader
            from rosbags.typesys import Stores, get_typestore
            
            typestore = get_typestore(Stores.ROS1_NOETIC)
            
            print(f"[数据加载] 加载rosbag: {self.bag_path}")
            
            with Reader(self.bag_path) as reader:
                # 打印可用的topic
                print(f"[数据加载] 可用topic: {[c.topic for c in reader.connections]}")
                
                for connection, timestamp, rawdata in reader.messages():
                    ts = timestamp / 1e9  # 转换为秒
                    
                    if connection.topic == '/imu0':
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        self.imu_data.append({
                            'timestamp': ts,
                            'linear_acceleration': np.array([
                                msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z
                            ]),
                            'angular_velocity': np.array([
                                msg.angular_velocity.x,
                                msg.angular_velocity.y,
                                msg.angular_velocity.z
                            ]),
                            'orientation': np.array([
                                msg.orientation.w,
                                msg.orientation.x,
                                msg.orientation.y,
                                msg.orientation.z
                            ])
                        })
                    
                    elif connection.topic == '/star_tracker/attitude':
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        self.star_tracker_data.append({
                            'timestamp': ts,
                            'quaternion': np.array([
                                msg.quaternion.w,
                                msg.quaternion.x,
                                msg.quaternion.y,
                                msg.quaternion.z
                            ])
                        })
                    
                    elif connection.topic == '/wheel_odometry/twist':
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        self.wheel_odom_data.append({
                            'timestamp': ts,
                            'linear_velocity': np.array([
                                msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z
                            ]),
                            'angular_velocity': np.array([
                                msg.twist.angular.x,
                                msg.twist.angular.y,
                                msg.twist.angular.z
                            ])
                        })
                    
                    elif connection.topic == '/ground_truth/pose':
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        self.ground_truth_pose.append({
                            'timestamp': ts,
                            'position': np.array([
                                msg.pose.position.x,
                                msg.pose.position.y,
                                msg.pose.position.z
                            ]),
                            'quaternion': np.array([
                                msg.pose.orientation.w,
                                msg.pose.orientation.x,
                                msg.pose.orientation.y,
                                msg.pose.orientation.z
                            ])
                        })
                    
                    elif connection.topic == '/ground_truth/twist':
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        self.ground_truth_twist.append({
                            'timestamp': ts,
                            'linear': np.array([
                                msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z
                            ]),
                            'angular': np.array([
                                msg.twist.angular.x,
                                msg.twist.angular.y,
                                msg.twist.angular.z
                            ])
                        })
            
            self.data_loaded = True
            print(f"[数据加载] 加载完成:")
            print(f"  - IMU数据: {len(self.imu_data)} 帧")
            print(f"  - 星敏感器数据: {len(self.star_tracker_data)} 帧")
            print(f"  - 轮速计数据: {len(self.wheel_odom_data)} 帧")
            print(f"  - 真值位姿: {len(self.ground_truth_pose)} 帧")
            
            return True
            
        except ImportError:
            print("[错误] 请安装rosbags库: pip install rosbags")
            return False
        except Exception as e:
            print(f"[错误] 加载rosbag失败: {e}")
            return False
    
    def get_time_range(self) -> Tuple[float, float]:
        """获取数据时间范围"""
        all_timestamps = []
        for data_list in [self.imu_data, self.star_tracker_data, 
                          self.wheel_odom_data, self.ground_truth_pose]:
            if data_list:
                all_timestamps.extend([d['timestamp'] for d in data_list])
        
        if not all_timestamps:
            return (0.0, 0.0)
        
        return (min(all_timestamps), max(all_timestamps))


class EurocDataLoader:
    """
    EuRoC格式数据加载器
    用于从EuRoC格式的CSV文件中读取数据
    """
    
    def __init__(self, dataset_path: str):
        self.dataset_path = Path(dataset_path)
        self.mav_path = self.dataset_path / "mav0"
        
        self.imu_data: List[Dict] = []
        self.ground_truth: List[Dict] = []
        
    def load(self) -> bool:
        """加载EuRoC格式数据"""
        try:
            # 加载IMU数据
            imu_csv = self.mav_path / "imu0" / "data.csv"
            if imu_csv.exists():
                self._load_imu(imu_csv)
            
            # 加载真值
            gt_csv = self.mav_path / "state_groundtruth_estimate0" / "data.csv"
            if gt_csv.exists():
                self._load_ground_truth(gt_csv)
            
            print(f"[EuRoC加载] 完成:")
            print(f"  - IMU数据: {len(self.imu_data)} 帧")
            print(f"  - 真值数据: {len(self.ground_truth)} 帧")
            
            return True
            
        except Exception as e:
            print(f"[错误] 加载EuRoC数据失败: {e}")
            return False
    
    def _load_imu(self, csv_path: Path):
        """加载IMU数据"""
        with open(csv_path, 'r') as f:
            for line in f:
                if line.startswith('#'):
                    continue
                parts = line.strip().split(',')
                if len(parts) >= 7:
                    self.imu_data.append({
                        'timestamp': float(parts[0]) / 1e9,
                        'angular_velocity': np.array([
                            float(parts[1]), float(parts[2]), float(parts[3])
                        ]),
                        'linear_acceleration': np.array([
                            float(parts[4]), float(parts[5]), float(parts[6])
                        ])
                    })
    
    def _load_ground_truth(self, csv_path: Path):
        """加载真值数据"""
        with open(csv_path, 'r') as f:
            for line in f:
                if line.startswith('#'):
                    continue
                parts = line.strip().split(',')
                if len(parts) >= 14:
                    self.ground_truth.append({
                        'timestamp': float(parts[0]) / 1e9,
                        'position': np.array([
                            float(parts[1]), float(parts[2]), float(parts[3])
                        ]),
                        'quaternion': np.array([
                            float(parts[4]), float(parts[5]), 
                            float(parts[6]), float(parts[7])
                        ]),
                        'velocity': np.array([
                            float(parts[8]), float(parts[9]), float(parts[10])
                        ]),
                        'angular_velocity': np.array([
                            float(parts[11]), float(parts[12]), float(parts[13])
                        ])
                    })


class MultiSensorFusionPipeline:
    """
    多传感器融合主管道
    """
    
    def __init__(self, config: FusionConfig):
        self.config = config
        
        # 初始化EKF
        self.ekf = MultiSensorEKF(config.ekf_config if config.ekf_config else None)
        
        # VINS接口
        self.vins_interface = None
        if config.use_vins:
            vins_config = VINSConfig()
            vins_config.config_file = config.vins_config_file
            self.vins_interface = VINSDockerInterface(vins_config)
        
        # 数据加载器
        self.rosbag_loader: Optional[RosbagDataLoader] = None
        self.euroc_loader: Optional[EurocDataLoader] = None
        
        # VINS轨迹
        self.vins_trajectory: List[Dict] = []
        
        # 融合结果
        self.fusion_results: List[Dict] = []
        
        # 输出目录
        self.output_dir = Path(config.output_path)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
    def load_dataset(self, dataset_path: str) -> bool:
        """
        加载数据集
        
        支持两种格式：
        1. rosbag格式
        2. EuRoC格式
        """
        self.config.dataset_path = dataset_path
        dataset_path = Path(dataset_path)
        
        # 检查是否为rosbag
        bag_file = None
        if dataset_path.suffix == '.bag':
            bag_file = dataset_path
        else:
            # 查找目录下的bag文件
            bag_files = list(dataset_path.glob('*.bag'))
            if bag_files:
                bag_file = bag_files[0]
        
        if bag_file and bag_file.exists():
            print(f"[融合管道] 检测到rosbag格式: {bag_file}")
            self.rosbag_loader = RosbagDataLoader(str(bag_file))
            return self.rosbag_loader.load()
        
        # 检查是否为EuRoC格式
        mav_path = dataset_path / "mav0"
        if mav_path.exists():
            print(f"[融合管道] 检测到EuRoC格式: {dataset_path}")
            self.euroc_loader = EurocDataLoader(str(dataset_path))
            return self.euroc_loader.load()
        
        print(f"[错误] 无法识别数据集格式: {dataset_path}")
        return False
    
    def run_vins(self, bag_path: Optional[str] = None) -> bool:
        """
        运行VINS-Fusion处理
        
        Args:
            bag_path: rosbag路径（可选，默认使用数据集中的bag）
        """
        if not self.config.use_vins or self.vins_interface is None:
            print("[融合管道] VINS已禁用，跳过")
            return False
        
        if bag_path is None:
            if self.rosbag_loader:
                bag_path = self.rosbag_loader.bag_path
            else:
                # 查找数据集目录下的bag文件
                dataset_dir = Path(self.config.dataset_path)
                bag_files = list(dataset_dir.glob('*.bag'))
                if bag_files:
                    bag_path = str(bag_files[0])
        
        if not bag_path:
            print("[错误] 未找到rosbag文件")
            return False
        
        print(f"[融合管道] 运行VINS处理: {bag_path}")
        
        output_csv = str(self.output_dir / "vins_trajectory.csv")
        result = self.vins_interface.process_bag_offline(
            bag_path, output_csv, self.config.vins_config_file
        )
        
        if result['success']:
            self.vins_trajectory = result['trajectory']
            print(f"[融合管道] VINS处理完成，获得 {len(self.vins_trajectory)} 个位姿")
            return True
        else:
            print(f"[错误] VINS处理失败: {result['error']}")
            return False
    
    def run_fusion(self) -> bool:
        """
        运行完整的融合流程
        """
        print("\n" + "="*60)
        print("开始多传感器融合定位")
        print("="*60)
        
        # 获取数据源
        if self.rosbag_loader and self.rosbag_loader.data_loaded:
            return self._run_fusion_from_rosbag()
        elif self.euroc_loader:
            return self._run_fusion_from_euroc()
        else:
            print("[错误] 没有可用的数据")
            return False
    
    def _run_fusion_from_rosbag(self) -> bool:
        """从rosbag数据运行融合"""
        loader = self.rosbag_loader
        
        # 获取初始状态
        if loader.ground_truth_pose:
            init_pose = loader.ground_truth_pose[0]
            init_twist = loader.ground_truth_twist[0] if loader.ground_truth_twist else None
            
            init_velocity = init_twist['linear'] if init_twist else np.zeros(3)
            
            self.ekf.initialize(
                position=init_pose['position'],
                velocity=init_velocity,
                quaternion=init_pose['quaternion'],
                timestamp=init_pose['timestamp']
            )
        else:
            # 使用默认初始化
            self.ekf.initialize(
                position=[0, 0, 0],
                velocity=[0, 0, 0],
                quaternion=[1, 0, 0, 0],
                timestamp=loader.imu_data[0]['timestamp'] if loader.imu_data else 0.0
            )
        
        # 合并所有数据并按时间排序
        all_events = []
        
        for imu in loader.imu_data:
            all_events.append(('imu', imu['timestamp'], imu))
        
        if self.config.use_star_tracker:
            for st in loader.star_tracker_data:
                all_events.append(('star_tracker', st['timestamp'], st))
        
        if self.config.use_wheel_odometry:
            for wo in loader.wheel_odom_data:
                all_events.append(('wheel_odom', wo['timestamp'], wo))
        
        # VINS轨迹数据
        for vins_pose in self.vins_trajectory:
            all_events.append(('vins', vins_pose['timestamp'], vins_pose))
        
        # 按时间排序
        all_events.sort(key=lambda x: x[1])
        
        print(f"[融合] 开始处理 {len(all_events)} 个事件...")
        
        # 真值索引（用于评估）
        gt_idx = 0
        
        # 处理所有事件
        start_time = time.time()
        for i, (event_type, timestamp, data) in enumerate(all_events):
            if event_type == 'imu':
                self.ekf.predict(data)
            elif event_type == 'vins':
                self.ekf.update_vins(data)
            elif event_type == 'star_tracker':
                self.ekf.update_star_tracker(data)
            elif event_type == 'wheel_odom':
                self.ekf.update_wheel_odometry(data)
            
            # 定期记录状态
            if i % 10 == 0:  # 每10个事件记录一次
                state = self.ekf.get_state()
                
                # 查找对应的真值
                gt_pose = None
                while (gt_idx < len(loader.ground_truth_pose) and 
                       loader.ground_truth_pose[gt_idx]['timestamp'] < timestamp):
                    gt_idx += 1
                
                if gt_idx < len(loader.ground_truth_pose):
                    gt_pose = loader.ground_truth_pose[gt_idx]
                
                self.fusion_results.append({
                    'timestamp': timestamp,
                    'estimated': state,
                    'ground_truth': gt_pose
                })
            
            # 进度显示
            if (i + 1) % 1000 == 0:
                elapsed = time.time() - start_time
                progress = (i + 1) / len(all_events) * 100
                print(f"  处理进度: {progress:.1f}% ({i+1}/{len(all_events)}), "
                      f"耗时: {elapsed:.1f}s")
        
        print(f"\n[融合] 完成，共处理 {len(all_events)} 个事件")
        print(f"[融合] 统计: {self.ekf.get_stats()}")
        
        return True
    
    def _run_fusion_from_euroc(self) -> bool:
        """从EuRoC数据运行融合"""
        loader = self.euroc_loader
        
        # 使用真值初始化
        if loader.ground_truth:
            init_gt = loader.ground_truth[0]
            self.ekf.initialize(
                position=init_gt['position'],
                velocity=init_gt.get('velocity', np.zeros(3)),
                quaternion=init_gt['quaternion'],
                timestamp=init_gt['timestamp']
            )
        else:
            self.ekf.initialize(
                position=[0, 0, 0],
                velocity=[0, 0, 0],
                quaternion=[1, 0, 0, 0],
                timestamp=loader.imu_data[0]['timestamp'] if loader.imu_data else 0.0
            )
        
        # 合并数据
        all_events = []
        
        for imu in loader.imu_data:
            all_events.append(('imu', imu['timestamp'], imu))
        
        for vins_pose in self.vins_trajectory:
            all_events.append(('vins', vins_pose['timestamp'], vins_pose))
        
        all_events.sort(key=lambda x: x[1])
        
        print(f"[融合] 开始处理 {len(all_events)} 个事件...")
        
        gt_idx = 0
        start_time = time.time()
        
        for i, (event_type, timestamp, data) in enumerate(all_events):
            if event_type == 'imu':
                self.ekf.predict(data)
            elif event_type == 'vins':
                self.ekf.update_vins(data)
            
            if i % 10 == 0:
                state = self.ekf.get_state()
                
                gt_pose = None
                while (gt_idx < len(loader.ground_truth) and 
                       loader.ground_truth[gt_idx]['timestamp'] < timestamp):
                    gt_idx += 1
                
                if gt_idx < len(loader.ground_truth):
                    gt_pose = loader.ground_truth[gt_idx]
                
                self.fusion_results.append({
                    'timestamp': timestamp,
                    'estimated': state,
                    'ground_truth': gt_pose
                })
            
            if (i + 1) % 1000 == 0:
                elapsed = time.time() - start_time
                progress = (i + 1) / len(all_events) * 100
                print(f"  处理进度: {progress:.1f}%")
        
        print(f"\n[融合] 完成")
        return True
    
    def evaluate(self) -> Dict:
        """
        评估融合结果
        
        计算：
        - 绝对轨迹误差 (ATE)
        - 相对位姿误差 (RPE)
        - 姿态误差
        """
        if not self.fusion_results:
            print("[评估] 没有融合结果")
            return {}
        
        # 提取有真值的结果
        valid_results = [r for r in self.fusion_results if r['ground_truth'] is not None]
        
        if not valid_results:
            print("[评估] 没有有效的真值对比数据")
            return {}
        
        # 计算位置误差
        position_errors = []
        for r in valid_results:
            est_pos = np.array(r['estimated']['position'])
            gt_pos = np.array(r['ground_truth']['position'])
            error = np.linalg.norm(est_pos - gt_pos)
            position_errors.append(error)
        
        position_errors = np.array(position_errors)
        
        evaluation = {
            'total_frames': len(self.fusion_results),
            'valid_frames': len(valid_results),
            'position_error': {
                'rmse': float(np.sqrt(np.mean(position_errors**2))),
                'mean': float(np.mean(position_errors)),
                'median': float(np.median(position_errors)),
                'max': float(np.max(position_errors)),
                'min': float(np.min(position_errors)),
                'std': float(np.std(position_errors))
            }
        }
        
        print("\n" + "="*60)
        print("融合评估结果")
        print("="*60)
        print(f"总帧数: {evaluation['total_frames']}")
        print(f"有效帧数: {evaluation['valid_frames']}")
        print(f"\n位置误差 (米):")
        print(f"  RMSE: {evaluation['position_error']['rmse']:.4f}")
        print(f"  均值: {evaluation['position_error']['mean']:.4f}")
        print(f"  中位数: {evaluation['position_error']['median']:.4f}")
        print(f"  最大: {evaluation['position_error']['max']:.4f}")
        print(f"  标准差: {evaluation['position_error']['std']:.4f}")
        
        return evaluation
    
    def save_results(self):
        """保存融合结果"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 保存轨迹CSV
        trajectory_file = self.output_dir / f"fusion_trajectory_{timestamp}.csv"
        with open(trajectory_file, 'w') as f:
            f.write("#timestamp,est_x,est_y,est_z,est_qw,est_qx,est_qy,est_qz,"
                   "gt_x,gt_y,gt_z,gt_qw,gt_qx,gt_qy,gt_qz\n")
            
            for r in self.fusion_results:
                est = r['estimated']
                gt = r['ground_truth']
                
                line = f"{r['timestamp']},"
                line += f"{est['position'][0]},{est['position'][1]},{est['position'][2]},"
                line += f"{est['quaternion'][0]},{est['quaternion'][1]},"
                line += f"{est['quaternion'][2]},{est['quaternion'][3]},"
                
                if gt:
                    line += f"{gt['position'][0]},{gt['position'][1]},{gt['position'][2]},"
                    line += f"{gt['quaternion'][0]},{gt['quaternion'][1]},"
                    line += f"{gt['quaternion'][2]},{gt['quaternion'][3]}"
                else:
                    line += ",,,,,,,"
                
                f.write(line + "\n")
        
        print(f"[保存] 轨迹已保存到: {trajectory_file}")
        
        # 保存评估结果
        evaluation = self.evaluate()
        eval_file = self.output_dir / f"evaluation_{timestamp}.json"
        with open(eval_file, 'w') as f:
            json.dump(evaluation, f, indent=2)
        
        print(f"[保存] 评估结果已保存到: {eval_file}")
        
        # 保存配置
        config_file = self.output_dir / f"config_{timestamp}.json"
        config_dict = {
            'dataset_path': self.config.dataset_path,
            'use_vins': self.config.use_vins,
            'use_star_tracker': self.config.use_star_tracker,
            'use_wheel_odometry': self.config.use_wheel_odometry,
            'ekf_stats': self.ekf.get_stats()
        }
        with open(config_file, 'w') as f:
            json.dump(config_dict, f, indent=2)
        
        print(f"[保存] 配置已保存到: {config_file}")
    
    def visualize(self):
        """可视化融合结果"""
        if not self.config.enable_visualization:
            return
        
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
        except ImportError:
            print("[警告] matplotlib未安装，跳过可视化")
            return
        
        if not self.fusion_results:
            return
        
        # 提取轨迹
        est_trajectory = np.array([r['estimated']['position'] for r in self.fusion_results])
        
        gt_trajectory = []
        for r in self.fusion_results:
            if r['ground_truth'] is not None:
                gt_trajectory.append(r['ground_truth']['position'])
        gt_trajectory = np.array(gt_trajectory) if gt_trajectory else None
        
        # 2D轨迹图
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # XY平面
        axes[0, 0].plot(est_trajectory[:, 0], est_trajectory[:, 1], 
                        'b-', label='估计轨迹', linewidth=1)
        if gt_trajectory is not None:
            axes[0, 0].plot(gt_trajectory[:, 0], gt_trajectory[:, 1], 
                           'r--', label='真值轨迹', linewidth=1)
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Y (m)')
        axes[0, 0].set_title('XY平面轨迹')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        axes[0, 0].axis('equal')
        
        # XZ平面
        axes[0, 1].plot(est_trajectory[:, 0], est_trajectory[:, 2], 
                        'b-', label='估计轨迹', linewidth=1)
        if gt_trajectory is not None:
            axes[0, 1].plot(gt_trajectory[:, 0], gt_trajectory[:, 2], 
                           'r--', label='真值轨迹', linewidth=1)
        axes[0, 1].set_xlabel('X (m)')
        axes[0, 1].set_ylabel('Z (m)')
        axes[0, 1].set_title('XZ平面轨迹')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # 位置误差
        if gt_trajectory is not None and len(gt_trajectory) == len(est_trajectory):
            errors = np.linalg.norm(est_trajectory - gt_trajectory, axis=1)
            timestamps = [r['timestamp'] for r in self.fusion_results]
            axes[1, 0].plot(timestamps, errors, 'g-', linewidth=1)
            axes[1, 0].set_xlabel('时间 (s)')
            axes[1, 0].set_ylabel('位置误差 (m)')
            axes[1, 0].set_title('位置误差随时间变化')
            axes[1, 0].grid(True)
        
        # 3D轨迹
        ax3d = fig.add_subplot(2, 2, 4, projection='3d')
        ax3d.plot(est_trajectory[:, 0], est_trajectory[:, 1], est_trajectory[:, 2], 
                  'b-', label='估计轨迹', linewidth=1)
        if gt_trajectory is not None:
            ax3d.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2], 
                     'r--', label='真值轨迹', linewidth=1)
        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Y (m)')
        ax3d.set_zlabel('Z (m)')
        ax3d.set_title('3D轨迹')
        ax3d.legend()
        
        plt.tight_layout()
        
        # 保存图像
        fig_path = self.output_dir / "trajectory_visualization.png"
        plt.savefig(fig_path, dpi=150)
        print(f"[可视化] 图像已保存到: {fig_path}")
        
        plt.show()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='多传感器融合定位系统',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 使用默认配置处理数据集
  python fusion_pipeline.py --dataset ./dataset/20260113_164908
  
  # 禁用VINS，仅使用EKF融合
  python fusion_pipeline.py --dataset ./dataset/20260113_164908 --no-vins
  
  # 指定输出目录
  python fusion_pipeline.py --dataset ./dataset/20260113_164908 -o ./output
        """
    )
    
    parser.add_argument('--dataset', '-d', type=str, required=True,
                       help='数据集路径（EuRoC格式目录或rosbag文件）')
    parser.add_argument('--output', '-o', type=str, default='./fusion_output',
                       help='输出目录（默认: ./fusion_output）')
    parser.add_argument('--no-vins', action='store_true',
                       help='禁用VINS-Fusion')
    parser.add_argument('--no-star-tracker', action='store_true',
                       help='禁用星敏感器融合')
    parser.add_argument('--no-wheel-odom', action='store_true',
                       help='禁用轮速计融合')
    parser.add_argument('--no-viz', action='store_true',
                       help='禁用可视化')
    parser.add_argument('--vins-config', type=str,
                       default='/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml',
                       help='VINS配置文件路径（容器内）')
    
    args = parser.parse_args()
    
    # 创建配置
    config = FusionConfig(
        dataset_path=args.dataset,
        output_path=args.output,
        use_vins=not args.no_vins,
        vins_config_file=args.vins_config,
        use_star_tracker=not args.no_star_tracker,
        use_wheel_odometry=not args.no_wheel_odom,
        enable_visualization=not args.no_viz
    )
    
    # 创建融合管道
    pipeline = MultiSensorFusionPipeline(config)
    
    # 加载数据集
    if not pipeline.load_dataset(args.dataset):
        print("[错误] 加载数据集失败")
        sys.exit(1)
    
    # 运行VINS（如果启用）
    if config.use_vins:
        if not pipeline.run_vins():
            print("[警告] VINS处理失败，继续使用其他传感器")
    
    # 运行融合
    if not pipeline.run_fusion():
        print("[错误] 融合处理失败")
        sys.exit(1)
    
    # 保存结果
    pipeline.save_results()
    
    # 可视化
    pipeline.visualize()
    
    print("\n" + "="*60)
    print("融合定位完成!")
    print("="*60)


if __name__ == "__main__":
    main()
