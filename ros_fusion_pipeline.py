"""
基于ROS robot_localization的多传感器融合管道
使用Docker中的ROS环境运行robot_localization EKF

融合架构：
┌─────────────────────────────────────────────────────────────────────┐
│                    Docker Container (ROS Melodic)                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   rosbag play ──► /obstacle_camera/* ──► VINS-Fusion                │
│              ──► /imu0                        │                      │
│                                               ▼                      │
│                                    /vins_estimator/odometry          │
│                                               │                      │
│   rosbag play ──► /star_tracker/attitude ─────┼──► robot_localization│
│              ──► /wheel_odometry/twist ───────┘      (EKF)          │
│                                                        │             │
│                                               /odometry/filtered     │
│                                                        │             │
│   rosbag record ◄──────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────────────┘

作者: GitHub Copilot
日期: 2026-01-13
"""

import os
import sys
import json
import time
import argparse
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from datetime import datetime
import numpy as np

# 导入自定义模块
from vins_docker_interface import VINSDockerInterface, VINSConfig


@dataclass
class ROSFusionConfig:
    """ROS融合配置"""
    # Docker配置
    container_name: str = "vins_fusion"
    image_name: str = "vins_fusion:melodic"
    host_data_dir: str = "~/data"
    container_data_dir: str = "/root/data"
    
    # 数据集路径
    dataset_path: str = ""
    
    # 输出路径
    output_path: str = "./ros_fusion_output"
    
    # VINS配置
    vins_config_file: str = "/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml"
    
    # robot_localization配置
    use_vins: bool = True
    use_star_tracker: bool = True
    use_wheel_odometry: bool = True
    
    # EKF配置参数
    ekf_frequency: float = 50.0  # EKF更新频率
    
    # 超时设置
    processing_timeout: int = 600


class ROSLaunchGenerator:
    """
    生成robot_localization所需的launch文件和配置文件
    """
    
    @staticmethod
    def generate_ekf_config(config: ROSFusionConfig) -> str:
        """
        生成robot_localization EKF配置YAML
        
        robot_localization状态向量 [15维]:
        [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
        """
        
        # 配置矩阵：指定每个传感器贡献哪些状态
        # [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
        
        ekf_config = f"""# robot_localization EKF配置
# 自动生成于 {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}

frequency: {config.ekf_frequency}
sensor_timeout: 0.1
two_d_mode: false
transform_time_offset: 0.0
transform_timeout: 0.0
print_diagnostics: true
debug: false

# 世界坐标系
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom

# ========== 传感器配置 ==========

# 传感器0: VINS-Fusion输出 (位置 + 姿态)
odom0: /vins_estimator/odometry
odom0_config: [true,  true,  true,   # x, y, z
               true,  true,  true,   # roll, pitch, yaw
               false, false, false,  # vx, vy, vz (不使用VINS速度，可能有延迟)
               false, false, false,  # vroll, vpitch, vyaw
               false, false, false]  # ax, ay, az
odom0_queue_size: 10
odom0_nodelay: true
odom0_differential: false
odom0_relative: false
odom0_pose_rejection_threshold: 5.0
odom0_twist_rejection_threshold: 1.0

# 传感器1: IMU数据 (姿态 + 角速度 + 加速度)
imu0: /imu0
imu0_config: [false, false, false,  # x, y, z
              true,  true,  true,   # roll, pitch, yaw
              false, false, false,  # vx, vy, vz
              true,  true,  true,   # vroll, vpitch, vyaw
              true,  true,  true]   # ax, ay, az
imu0_nodelay: true
imu0_differential: false
imu0_relative: false
imu0_queue_size: 50
imu0_pose_rejection_threshold: 0.8
imu0_twist_rejection_threshold: 0.8
imu0_linear_acceleration_rejection_threshold: 0.8
imu0_remove_gravitational_acceleration: true

# 传感器2: 轮速计 (速度) - 使用Odometry消息
odom1: /wheel_odometry
odom1_config: [false, false, false,  # x, y, z (不用位置)
               false, false, false,  # roll, pitch, yaw (不用姿态)
               true,  true,  true,   # vx, vy, vz (使用线速度)
               true,  true,  true,   # vroll, vpitch, vyaw (使用角速度)
               false, false, false]  # ax, ay, az (不用加速度)
odom1_queue_size: 10
odom1_differential: false
odom1_relative: false
odom1_twist_rejection_threshold: 2.0

# 传感器3: 星敏感器 (仅姿态，通过pose话题)
# 注：需要将QuaternionStamped转换为PoseStamped
pose0: /star_tracker/pose
pose0_config: [false, false, false,  # x, y, z
               true,  true,  true,   # roll, pitch, yaw
               false, false, false,  # vx, vy, vz
               false, false, false,  # vroll, vpitch, vyaw
               false, false, false]  # ax, ay, az
pose0_queue_size: 10
pose0_differential: false
pose0_relative: false
pose0_rejection_threshold: 2.0

# ========== 过程噪声协方差 ==========
# 对角线元素，对应15维状态
process_noise_covariance: [0.05,   0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                           0,      0.05,   0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                           0,      0,      0.06,   0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                           0,      0,      0,      0.03,   0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                           0,      0,      0,      0,      0.03,   0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                           0,      0,      0,      0,      0,      0.06,   0,      0,      0,      0,      0,      0,      0,      0,      0,
                           0,      0,      0,      0,      0,      0,      0.025,  0,      0,      0,      0,      0,      0,      0,      0,
                           0,      0,      0,      0,      0,      0,      0,      0.025,  0,      0,      0,      0,      0,      0,      0,
                           0,      0,      0,      0,      0,      0,      0,      0,      0.04,   0,      0,      0,      0,      0,      0,
                           0,      0,      0,      0,      0,      0,      0,      0,      0,      0.01,   0,      0,      0,      0,      0,
                           0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.01,   0,      0,      0,      0,
                           0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.02,   0,      0,      0,
                           0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.01,   0,      0,
                           0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.01,   0,
                           0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.015]

# ========== 初始协方差 ==========
initial_estimate_covariance: [1e-9,  0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
                              0,     1e-9,  0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
                              0,     0,     1e-9,  0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
                              0,     0,     0,     1e-9,  0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
                              0,     0,     0,     0,     1e-9,  0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
                              0,     0,     0,     0,     0,     1e-9,  0,     0,     0,     0,     0,     0,     0,     0,     0,
                              0,     0,     0,     0,     0,     0,     1e-9,  0,     0,     0,     0,     0,     0,     0,     0,
                              0,     0,     0,     0,     0,     0,     0,     1e-9,  0,     0,     0,     0,     0,     0,     0,
                              0,     0,     0,     0,     0,     0,     0,     0,     1e-9,  0,     0,     0,     0,     0,     0,
                              0,     0,     0,     0,     0,     0,     0,     0,     0,     1e-9,  0,     0,     0,     0,     0,
                              0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1e-9,  0,     0,     0,     0,
                              0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1e-9,  0,     0,     0,
                              0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1e-9,  0,     0,
                              0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1e-9,  0,
                              0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1e-9]
"""
        return ekf_config
    
    @staticmethod
    def generate_launch_file(config: ROSFusionConfig) -> str:
        """生成launch文件"""
        launch_content = f"""<?xml version="1.0"?>
<launch>
    <!-- 
    多传感器融合定位 Launch文件
    自动生成于 {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
    -->
    
    <!-- 参数 -->
    <arg name="bag_file" default="{config.container_data_dir}/dataset.bag"/>
    <arg name="output_bag" default="{config.container_data_dir}/fusion_output.bag"/>
    <arg name="ekf_config" default="{config.container_data_dir}/ekf_config.yaml"/>
    <arg name="vins_config" default="{config.vins_config_file}"/>
    
    <!-- TF静态变换 (根据实际传感器安装位置调整) -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_imu"
          args="0 0 0 0 0 0 base_link imu"/>
    
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_camera"
          args="0.2 0 0.3 0 0.26 0 base_link camera"/>
    
    <!-- 星敏感器消息转换节点 (QuaternionStamped -> PoseStamped) -->
    <node pkg="topic_tools" type="transform" name="star_tracker_to_pose"
          args="/star_tracker/attitude /star_tracker/pose geometry_msgs/PoseStamped
          'geometry_msgs.msg.PoseStamped(header=m.header, 
           pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0,0,0), 
           orientation=m.quaternion))' 
          --import geometry_msgs std_msgs"
          output="screen"/>
    
    <!-- robot_localization EKF节点 -->
    <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization"
          output="screen">
        <rosparam command="load" file="$(arg ekf_config)"/>
        <remap from="odometry/filtered" to="/ekf/odometry"/>
    </node>
    
    <!-- VINS-Fusion节点 -->
    <node pkg="vins" type="vins_node" name="vins_estimator"
          args="$(arg vins_config)"
          output="screen"/>
    
    <!-- 录制输出 -->
    <node pkg="rosbag" type="record" name="bag_recorder"
          args="-O $(arg output_bag) 
                /ekf/odometry 
                /vins_estimator/odometry 
                /vins_estimator/path
                /ground_truth/pose"/>
    
</launch>
"""
        return launch_content
    
    @staticmethod
    def generate_simple_launch_file() -> str:
        """生成简化版launch文件（仅EKF，不包含VINS）"""
        return """<?xml version="1.0"?>
<launch>
    <!-- 简化版：仅运行robot_localization EKF -->
    
    <arg name="ekf_config" default="/root/data/ekf_config.yaml"/>
    
    <!-- robot_localization EKF节点 -->
    <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization"
          output="screen">
        <rosparam command="load" file="$(arg ekf_config)"/>
        <remap from="odometry/filtered" to="/ekf/odometry"/>
    </node>
    
</launch>
"""


class ROSFusionPipeline:
    """
    基于ROS robot_localization的融合管道
    """
    
    def __init__(self, config: ROSFusionConfig):
        self.config = config
        
        # Docker接口
        vins_config = VINSConfig()
        vins_config.container_name = config.container_name
        vins_config.image_name = config.image_name
        vins_config.host_data_dir = config.host_data_dir
        vins_config.container_data_dir = config.container_data_dir
        self.docker = VINSDockerInterface(vins_config)
        
        # 输出目录
        self.output_dir = Path(config.output_path)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 融合结果
        self.fusion_results: List[Dict] = []
        
    def check_robot_localization(self) -> bool:
        """检查robot_localization是否安装在Docker中"""
        print("[检查] 检查robot_localization包...")
        
        result = self.docker.exec_in_container(
            "rospack find robot_localization",
            timeout=10
        )
        
        if result.returncode == 0:
            print(f"[检查] robot_localization已安装: {result.stdout.strip()}")
            return True
        else:
            print("[检查] robot_localization未安装，需要安装...")
            return False
    
    def install_robot_localization(self) -> bool:
        """在Docker容器中安装robot_localization"""
        print("[安装] 正在安装robot_localization...")
        
        # 完整的安装命令（包括配置软件源）
        install_cmds = [
            # 1. 配置ROS软件源
            "sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu bionic main\" > /etc/apt/sources.list.d/ros-latest.list'",
            
            # 2. 添加ROS GPG密钥（尝试多种方法）
            "apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654 || "
            "apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 || "
            "curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -",
            
            # 3. 更新包列表
            "apt-get update",
            
            # 4. 安装robot_localization和依赖
            "apt-get install -y ros-melodic-robot-localization || "
            "apt-get install -y --fix-missing ros-melodic-robot-localization",
            
            # 5. 安装topic-tools（用于消息转换）
            "apt-get install -y ros-melodic-topic-tools || true",
            
            # 6. 安装其他可能需要的包
            "apt-get install -y ros-melodic-tf2-ros || true",
        ]
        
        for i, cmd in enumerate(install_cmds, 1):
            print(f"  [{i}/{len(install_cmds)}] 执行: {cmd[:80]}...")
            result = self.docker.exec_in_container(cmd, timeout=300)
            
            # 对于关键步骤检查返回码
            if i >= 4 and result.returncode != 0:  # 从第4步开始检查
                print(f"[错误] 命令失败: {result.stderr}")
                if "robot-localization" in cmd and result.returncode != 0:
                    print("\n[提示] 可能是网络问题或软件源配置问题")
                    print("请尝试手动安装，详见下方说明")
                    return False
        
        # 验证安装
        print("  [验证] 检查robot_localization是否安装成功...")
        verify_result = self.docker.exec_in_container(
            "rospack find robot_localization",
            timeout=10
        )
        
        if verify_result.returncode == 0:
            print("[安装] robot_localization安装完成")
            return True
        else:
            print("[错误] 安装验证失败")
            return False
    
    def prepare_config_files(self) -> bool:
        """准备配置文件并复制到容器"""
        print("[准备] 生成配置文件...")
        
        # 生成EKF配置
        ekf_config = ROSLaunchGenerator.generate_ekf_config(self.config)
        ekf_config_path = self.output_dir / "ekf_config.yaml"
        with open(ekf_config_path, 'w') as f:
            f.write(ekf_config)
        print(f"  EKF配置: {ekf_config_path}")
        
        # 生成launch文件
        launch_content = ROSLaunchGenerator.generate_launch_file(self.config)
        launch_path = self.output_dir / "fusion.launch"
        with open(launch_path, 'w') as f:
            f.write(launch_content)
        print(f"  Launch文件: {launch_path}")
        
        # 生成简化版launch
        simple_launch = ROSLaunchGenerator.generate_simple_launch_file()
        simple_launch_path = self.output_dir / "ekf_only.launch"
        with open(simple_launch_path, 'w') as f:
            f.write(simple_launch)
        
        # 复制到容器
        print("[准备] 复制配置文件到容器...")
        
        copy_cmd = f"docker cp {ekf_config_path} {self.config.container_name}:{self.config.container_data_dir}/"
        subprocess.run(copy_cmd, shell=True)
        
        copy_cmd = f"docker cp {launch_path} {self.config.container_name}:{self.config.container_data_dir}/"
        subprocess.run(copy_cmd, shell=True)
        
        copy_cmd = f"docker cp {simple_launch_path} {self.config.container_name}:{self.config.container_data_dir}/"
        subprocess.run(copy_cmd, shell=True)
        
        return True
    
    def run_fusion(self, bag_path: str) -> Dict:
        """
        运行完整的融合流程
        
        Args:
            bag_path: 输入的rosbag路径
            
        Returns:
            处理结果字典
        """
        result = {
            'success': False,
            'output_bag': None,
            'trajectory': [],
            'error': None
        }
        
        print("\n" + "="*60)
        print("ROS robot_localization 融合管道")
        print("="*60)
        
        # 1. 确保容器运行
        print("\n[步骤1] 启动Docker容器...")
        if not self.docker.container_running:
            if not self.docker.start_container():
                result['error'] = "无法启动容器"
                return result
        
        # 2. 检查并安装robot_localization
        print("\n[步骤2] 检查robot_localization...")
        if not self.check_robot_localization():
            if not self.install_robot_localization():
                result['error'] = "无法安装robot_localization"
                return result
        
        # 3. 准备配置文件
        print("\n[步骤3] 准备配置文件...")
        if not self.prepare_config_files():
            result['error'] = "配置文件准备失败"
            return result
        
        # 4. 启动roscore
        print("\n[步骤4] 启动roscore...")
        if not self.docker.roscore_running:
            if not self.docker.start_roscore():
                result['error'] = "无法启动roscore"
                return result
        
        # 5. 运行融合
        print("\n[步骤5] 运行融合流程...")
        try:
            result = self._run_fusion_process(bag_path)
        except Exception as e:
            result['error'] = str(e)
            print(f"[错误] 融合过程出错: {e}")
        
        return result
    
    def _run_fusion_process(self, bag_path: str) -> Dict:
        """执行融合处理"""
        result = {
            'success': False,
            'output_bag': None,
            'trajectory': [],
            'error': None
        }
        
        bag_name = os.path.basename(bag_path)
        container_bag = f"{self.config.container_data_dir}/{bag_name}"
        output_bag = f"{self.config.container_data_dir}/fusion_output.bag"
        
        # 启动录制
        print("  启动轨迹录制...")
        record_cmd = f"rosbag record -O {output_bag} /ekf/odometry /vins_estimator/odometry /ground_truth/pose"
        self.docker.exec_in_container_background(record_cmd, "recorder")
        time.sleep(2)
        
        # 启动robot_localization EKF
        print("  启动robot_localization EKF...")
        ekf_cmd = f"roslaunch {self.config.container_data_dir}/ekf_only.launch"
        self.docker.exec_in_container_background(ekf_cmd, "ekf")
        time.sleep(3)
        
        # 如果使用VINS
        if self.config.use_vins:
            print("  启动VINS-Fusion...")
            vins_cmd = f"rosrun vins vins_node {self.config.vins_config_file}"
            self.docker.exec_in_container_background(vins_cmd, "vins")
            time.sleep(3)
        
        # 播放rosbag
        print(f"  播放数据集: {bag_name}")
        play_result = self.docker.exec_in_container(
            f"rosbag play {container_bag} --clock",
            timeout=self.config.processing_timeout
        )
        
        # 等待处理完成
        print("  等待处理完成...")
        time.sleep(5)
        
        # 停止所有节点
        print("  停止节点...")
        self.docker.exec_in_container("pkill -f 'rosbag record'")
        self.docker.exec_in_container("pkill -f 'ekf_localization'")
        if self.config.use_vins:
            self.docker.exec_in_container("pkill -f 'vins_node'")
        time.sleep(2)
        
        # 复制输出
        print("  复制输出文件...")
        local_output = str(self.output_dir / "fusion_output.bag")
        copy_cmd = f"docker cp {self.config.container_name}:{output_bag} {local_output}"
        subprocess.run(copy_cmd, shell=True)
        
        if os.path.exists(local_output):
            result['success'] = True
            result['output_bag'] = local_output
            
            # 提取轨迹
            result['trajectory'] = self._extract_trajectory(local_output)
            print(f"  提取了 {len(result['trajectory'])} 个位姿")
        else:
            result['error'] = "输出文件未生成"
        
        return result
    
    def _extract_trajectory(self, bag_path: str) -> List[Dict]:
        """从输出bag中提取融合轨迹"""
        trajectory = []
        
        try:
            from rosbags.rosbag1 import Reader
            from rosbags.typesys import Stores, get_typestore
            
            typestore = get_typestore(Stores.ROS1_NOETIC)
            
            with Reader(bag_path) as reader:
                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == '/ekf/odometry':
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        
                        pose = {
                            'timestamp': timestamp / 1e9,
                            'position': [
                                msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                msg.pose.pose.position.z
                            ],
                            'quaternion': [
                                msg.pose.pose.orientation.w,
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z
                            ],
                            'velocity': [
                                msg.twist.twist.linear.x,
                                msg.twist.twist.linear.y,
                                msg.twist.twist.linear.z
                            ]
                        }
                        trajectory.append(pose)
        
        except ImportError:
            print("[警告] rosbags库未安装")
        except Exception as e:
            print(f"[警告] 提取轨迹失败: {e}")
        
        return trajectory
    
    def run_ekf_only(self, bag_path: str) -> Dict:
        """
        仅运行robot_localization EKF（不使用VINS）
        适用于已有VINS输出的情况
        """
        self.config.use_vins = False
        return self.run_fusion(bag_path)
    
    def save_results(self):
        """保存融合结果"""
        if not self.fusion_results:
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 保存轨迹CSV
        trajectory_file = self.output_dir / f"ros_fusion_trajectory_{timestamp}.csv"
        with open(trajectory_file, 'w') as f:
            f.write("#timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz\n")
            for pose in self.fusion_results:
                line = f"{pose['timestamp']},"
                line += f"{pose['position'][0]},{pose['position'][1]},{pose['position'][2]},"
                line += f"{pose['quaternion'][0]},{pose['quaternion'][1]},"
                line += f"{pose['quaternion'][2]},{pose['quaternion'][3]},"
                line += f"{pose.get('velocity', [0,0,0])[0]},"
                line += f"{pose.get('velocity', [0,0,0])[1]},"
                line += f"{pose.get('velocity', [0,0,0])[2]}"
                f.write(line + "\n")
        
        print(f"[保存] 轨迹已保存到: {trajectory_file}")


def print_manual_steps():
    """打印手动运行步骤（如果自动化失败）"""
    print("""
================================================================================
手动运行 robot_localization 步骤
================================================================================

如果自动化流程失败，可以按以下步骤手动操作：

【准备工作】
1. 将生成的配置文件复制到Docker数据目录：
   - ekf_config.yaml
   - fusion.launch 或 ekf_only.launch

【在Ubuntu/WSL2中操作】

# 终端1: 启动容器和roscore
docker start vins_fusion
docker exec -it vins_fusion bash
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash
roscore

# 终端2: 安装robot_localization（如果未安装）
docker exec -it vins_fusion bash
apt-get update
apt-get install -y ros-melodic-robot-localization ros-melodic-topic-tools
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash

# 启动EKF
roslaunch /root/data/ekf_only.launch

# 终端3: 启动VINS（可选）
docker exec -it vins_fusion bash
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash
rosrun vins vins_node /root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml

# 终端4: 录制输出
docker exec -it vins_fusion bash
source /opt/ros/melodic/setup.bash
rosbag record -O /root/data/fusion_output.bag /ekf/odometry /vins_estimator/odometry

# 终端5: 播放数据集
docker exec -it vins_fusion bash
source /opt/ros/melodic/setup.bash
rosbag play /root/data/dataset.bag --clock

【完成后】
# 复制输出到主机
docker cp vins_fusion:/root/data/fusion_output.bag ~/data/

================================================================================
""")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='基于ROS robot_localization的多传感器融合',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 完整融合（VINS + robot_localization EKF）
  python ros_fusion_pipeline.py --bag ./dataset/dataset.bag
  
  # 仅EKF融合（不运行VINS）
  python ros_fusion_pipeline.py --bag ./dataset/dataset.bag --ekf-only
  
  # 显示手动操作步骤
  python ros_fusion_pipeline.py --manual
        """
    )
    
    parser.add_argument('--bag', '-b', type=str,
                       help='输入rosbag文件路径')
    parser.add_argument('--output', '-o', type=str, default='./ros_fusion_output',
                       help='输出目录（默认: ./ros_fusion_output）')
    parser.add_argument('--ekf-only', action='store_true',
                       help='仅运行EKF（不启动VINS）')
    parser.add_argument('--no-star-tracker', action='store_true',
                       help='禁用星敏感器')
    parser.add_argument('--no-wheel-odom', action='store_true',
                       help='禁用轮速计')
    parser.add_argument('--manual', action='store_true',
                       help='显示手动操作步骤')
    parser.add_argument('--generate-config', action='store_true',
                       help='仅生成配置文件，不运行')
    
    args = parser.parse_args()
    
    # 显示手动步骤
    if args.manual:
        print_manual_steps()
        return
    
    # 创建配置
    config = ROSFusionConfig(
        output_path=args.output,
        use_vins=not args.ekf_only,
        use_star_tracker=not args.no_star_tracker,
        use_wheel_odometry=not args.no_wheel_odom
    )
    
    # 仅生成配置
    if args.generate_config:
        pipeline = ROSFusionPipeline(config)
        
        # 生成配置文件
        ekf_config = ROSLaunchGenerator.generate_ekf_config(config)
        ekf_path = Path(args.output) / "ekf_config.yaml"
        ekf_path.parent.mkdir(parents=True, exist_ok=True)
        with open(ekf_path, 'w') as f:
            f.write(ekf_config)
        print(f"已生成EKF配置: {ekf_path}")
        
        launch_content = ROSLaunchGenerator.generate_launch_file(config)
        launch_path = Path(args.output) / "fusion.launch"
        with open(launch_path, 'w') as f:
            f.write(launch_content)
        print(f"已生成Launch文件: {launch_path}")
        
        simple_launch = ROSLaunchGenerator.generate_simple_launch_file()
        simple_path = Path(args.output) / "ekf_only.launch"
        with open(simple_path, 'w') as f:
            f.write(simple_launch)
        print(f"已生成简化Launch文件: {simple_path}")
        
        print("\n配置文件生成完成！")
        print("请将这些文件复制到Docker容器的数据目录中使用。")
        return
    
    # 检查必需参数
    if not args.bag:
        parser.error("需要指定 --bag 参数，或使用 --manual 查看手动步骤")
    
    # 创建管道并运行
    pipeline = ROSFusionPipeline(config)
    
    result = pipeline.run_fusion(args.bag)
    
    if result['success']:
        pipeline.fusion_results = result['trajectory']
        pipeline.save_results()
        print("\n融合完成！")
    else:
        print(f"\n融合失败: {result['error']}")
        print("\n请尝试手动运行，使用 --manual 参数查看步骤")


if __name__ == "__main__":
    main()
