"""
VINS-Fusion Docker自动化接口
用于从Windows主机自动控制Ubuntu Docker中的VINS-Fusion

功能：
1. 自动启动Docker容器
2. 自动运行VINS-Fusion节点
3. 处理rosbag数据
4. 提取轨迹输出

作者: GitHub Copilot
日期: 2026-01-13

注意：此脚本需要WSL2环境或SSH连接到Ubuntu主机
"""

import subprocess
import os
import time
import json
import csv
import threading
from pathlib import Path
from typing import Optional, Dict, List, Callable
from dataclasses import dataclass
import numpy as np


@dataclass
class VINSConfig:
    """VINS配置"""
    # Docker配置
    container_name: str = "vins_fusion"
    image_name: str = "vins_fusion:melodic"
    host_data_dir: str = "~/data"
    container_data_dir: str = "/root/data"
    
    # VINS配置文件路径（容器内）
    config_file: str = "/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml"
    
    # 超时设置（秒）
    startup_timeout: int = 30
    processing_timeout: int = 600  # 10分钟
    
    # SSH配置（如果使用远程Ubuntu）
    use_ssh: bool = False
    ssh_host: str = ""
    ssh_user: str = ""
    ssh_key: str = ""


class VINSDockerInterface:
    """
    VINS-Fusion Docker接口
    
    支持两种运行模式：
    1. WSL2模式：Windows下通过WSL2运行Docker
    2. SSH模式：通过SSH连接到远程Ubuntu主机
    """
    
    def __init__(self, config: Optional[VINSConfig] = None):
        self.config = config or VINSConfig()
        self.container_running = False
        self.roscore_running = False
        self._process_threads: List[threading.Thread] = []
        
        # 检测运行环境
        self.execution_mode = self._detect_execution_mode()
        print(f"[VINS接口] 执行模式: {self.execution_mode}")
    
    def _detect_execution_mode(self) -> str:
        """检测执行模式"""
        if self.config.use_ssh and self.config.ssh_host:
            return "ssh"
        
        # 检查是否在WSL2中
        try:
            result = subprocess.run(
                ["wsl", "--status"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                return "wsl"
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        
        # 检查是否直接在Linux中
        if os.name == 'posix':
            return "native"
        
        return "unknown"
    
    def _run_command(self, cmd: str, timeout: int = 60, 
                     capture_output: bool = True) -> subprocess.CompletedProcess:
        """
        在适当的环境中运行命令
        """
        if self.execution_mode == "wsl":
            full_cmd = f'wsl -d Ubuntu-24.04 -- bash -c "{cmd}"'
        elif self.execution_mode == "ssh":
            ssh_cmd = f"ssh -i {self.config.ssh_key} {self.config.ssh_user}@{self.config.ssh_host}"
            full_cmd = f'{ssh_cmd} "{cmd}"'
        else:
            full_cmd = cmd
        
        try:
            result = subprocess.run(
                full_cmd,
                shell=True,
                capture_output=capture_output,
                text=True,
                timeout=timeout
            )
            return result
        except subprocess.TimeoutExpired:
            print(f"[警告] 命令超时: {cmd[:50]}...")
            return subprocess.CompletedProcess(full_cmd, -1, "", "Timeout")
    
    def _run_background_command(self, cmd: str, name: str = "background") -> subprocess.Popen:
        """
        在后台运行命令
        """
        if self.execution_mode == "wsl":
            full_cmd = f'wsl -d Ubuntu-24.04 -- bash -c "{cmd}"'
        elif self.execution_mode == "ssh":
            ssh_cmd = f"ssh -i {self.config.ssh_key} {self.config.ssh_user}@{self.config.ssh_host}"
            full_cmd = f'{ssh_cmd} "{cmd}"'
        else:
            full_cmd = cmd
        
        process = subprocess.Popen(
            full_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        print(f"[{name}] 后台进程已启动, PID: {process.pid}")
        return process
    
    def check_docker_status(self) -> Dict:
        """检查Docker和容器状态"""
        status = {
            'docker_running': False,
            'container_exists': False,
            'container_running': False,
            'image_exists': False
        }
        
        # 检查Docker是否运行
        result = self._run_command("docker info", timeout=10)
        status['docker_running'] = result.returncode == 0
        
        if not status['docker_running']:
            return status
        
        # 检查镜像是否存在
        result = self._run_command(f"docker images -q {self.config.image_name}")
        status['image_exists'] = bool(result.stdout.strip())
        
        # 检查容器是否存在和运行
        result = self._run_command(f"docker ps -a --filter name={self.config.container_name} --format '{{{{.Status}}}}'")
        if result.stdout.strip():
            status['container_exists'] = True
            status['container_running'] = 'Up' in result.stdout
        
        return status
    
    def start_container(self, with_gui: bool = False) -> bool:
        """
        启动VINS容器
        
        Args:
            with_gui: 是否启用GUI支持（用于RViz）
        """
        status = self.check_docker_status()
        
        if not status['docker_running']:
            print("[错误] Docker未运行")
            return False
        
        if not status['image_exists']:
            print(f"[错误] 镜像 {self.config.image_name} 不存在")
            return False
        
        # 如果容器已存在但未运行，启动它
        if status['container_exists']:
            if status['container_running']:
                print("[VINS] 容器已在运行")
                self.container_running = True
                return True
            else:
                print("[VINS] 启动已存在的容器...")
                result = self._run_command(f"docker start {self.config.container_name}")
                if result.returncode == 0:
                    self.container_running = True
                    return True
                else:
                    # 删除旧容器重新创建
                    self._run_command(f"docker rm {self.config.container_name}")
        
        # 创建新容器
        print("[VINS] 创建新容器...")
        
        if with_gui:
            # 允许X11转发
            self._run_command("xhost +local:docker")
            
            docker_cmd = f"""docker run -d \\
                --name {self.config.container_name} \\
                --network host \\
                --privileged \\
                -e DISPLAY=$DISPLAY \\
                -v /tmp/.X11-unix:/tmp/.X11-unix \\
                -v {self.config.host_data_dir}:{self.config.container_data_dir} \\
                {self.config.image_name} \\
                sleep infinity"""
        else:
            docker_cmd = f"""docker run -d \\
                --name {self.config.container_name} \\
                --network host \\
                -v {self.config.host_data_dir}:{self.config.container_data_dir} \\
                {self.config.image_name} \\
                sleep infinity"""
        
        result = self._run_command(docker_cmd.replace('\n', ' ').replace('\\', ''))
        
        if result.returncode == 0:
            print("[VINS] 容器创建成功")
            self.container_running = True
            time.sleep(2)  # 等待容器完全启动
            return True
        else:
            print(f"[错误] 容器创建失败: {result.stderr}")
            return False
    
    def stop_container(self):
        """停止容器"""
        self._run_command(f"docker stop {self.config.container_name}")
        self.container_running = False
        self.roscore_running = False
        print("[VINS] 容器已停止")
    
    def exec_in_container(self, cmd: str, timeout: int = 60) -> subprocess.CompletedProcess:
        """在容器内执行命令"""
        docker_exec = f"docker exec {self.config.container_name} bash -c 'source /opt/ros/melodic/setup.bash && source /root/catkin_ws/devel/setup.bash && {cmd}'"
        return self._run_command(docker_exec, timeout=timeout)
    
    def exec_in_container_background(self, cmd: str, name: str = "vins") -> subprocess.Popen:
        """在容器内后台执行命令"""
        docker_exec = f"docker exec {self.config.container_name} bash -c 'source /opt/ros/melodic/setup.bash && source /root/catkin_ws/devel/setup.bash && {cmd}'"
        return self._run_background_command(docker_exec, name)
    
    def start_roscore(self) -> bool:
        """启动roscore"""
        if not self.container_running:
            print("[错误] 容器未运行")
            return False
        
        print("[VINS] 启动roscore...")
        # 后台启动roscore
        self.exec_in_container_background("roscore", "roscore")
        time.sleep(3)
        
        # 验证roscore是否启动
        result = self.exec_in_container("rostopic list", timeout=10)
        if result.returncode == 0:
            print("[VINS] roscore启动成功")
            self.roscore_running = True
            return True
        else:
            print("[错误] roscore启动失败")
            return False
    
    def process_bag(self, bag_path: str, output_path: str,
                    config_file: Optional[str] = None,
                    progress_callback: Optional[Callable[[float], None]] = None) -> Dict:
        """
        处理rosbag文件
        
        Args:
            bag_path: rosbag文件路径（主机路径）
            output_path: 输出文件路径（主机路径）
            config_file: VINS配置文件路径（可选）
            progress_callback: 进度回调函数
        
        Returns:
            处理结果字典
        """
        result = {
            'success': False,
            'output_file': None,
            'trajectory': [],
            'error': None
        }
        
        # 确保容器和roscore运行
        if not self.container_running:
            if not self.start_container():
                result['error'] = "无法启动容器"
                return result
        
        if not self.roscore_running:
            if not self.start_roscore():
                result['error'] = "无法启动roscore"
                return result
        
        config = config_file or self.config.config_file
        
        # 转换路径到容器内路径
        # 假设bag_path在~/data下
        bag_name = os.path.basename(bag_path)
        container_bag_path = f"{self.config.container_data_dir}/{bag_name}"
        container_output_path = f"{self.config.container_data_dir}/vins_output.bag"
        
        try:
            # 1. 启动录制（后台）
            print("[VINS] 启动轨迹录制...")
            record_cmd = f"rosbag record -O {container_output_path} /vins_estimator/odometry /vins_estimator/path"
            record_process = self.exec_in_container_background(record_cmd, "recorder")
            time.sleep(2)
            
            # 2. 启动VINS节点（后台）
            print("[VINS] 启动VINS节点...")
            vins_cmd = f"rosrun vins vins_node {config}"
            vins_process = self.exec_in_container_background(vins_cmd, "vins_node")
            time.sleep(3)
            
            # 3. 播放rosbag
            print(f"[VINS] 播放数据集: {bag_name}")
            play_result = self.exec_in_container(
                f"rosbag play {container_bag_path} --clock",
                timeout=self.config.processing_timeout
            )
            
            # 4. 等待处理完成
            print("[VINS] 等待处理完成...")
            time.sleep(5)
            
            # 5. 停止录制和VINS节点
            self.exec_in_container("pkill -f 'rosbag record'")
            self.exec_in_container("pkill -f 'vins_node'")
            time.sleep(2)
            
            # 6. 复制输出文件到主机
            print("[VINS] 复制输出文件...")
            copy_cmd = f"docker cp {self.config.container_name}:{container_output_path} {output_path}"
            self._run_command(copy_cmd)
            
            # 7. 提取轨迹数据
            trajectory = self._extract_trajectory_from_bag(output_path)
            
            result['success'] = True
            result['output_file'] = output_path
            result['trajectory'] = trajectory
            
        except Exception as e:
            result['error'] = str(e)
            print(f"[错误] VINS处理失败: {e}")
        
        return result
    
    def process_bag_offline(self, bag_path: str, output_csv: str,
                           config_file: Optional[str] = None) -> Dict:
        """
        离线处理rosbag并直接输出CSV轨迹
        
        这个方法使用VINS的输出文件而不是录制rosbag
        
        Args:
            bag_path: 输入rosbag路径
            output_csv: 输出CSV路径
            config_file: VINS配置文件
        
        Returns:
            处理结果
        """
        result = {
            'success': False,
            'output_file': None,
            'trajectory': [],
            'error': None
        }
        
        # 确保容器运行
        if not self.container_running:
            if not self.start_container():
                result['error'] = "无法启动容器"
                return result
        
        if not self.roscore_running:
            if not self.start_roscore():
                result['error'] = "无法启动roscore"
                return result
        
        config = config_file or self.config.config_file
        bag_name = os.path.basename(bag_path)
        container_bag_path = f"{self.config.container_data_dir}/{bag_name}"
        
        try:
            # 1. 启动VINS节点
            print("[VINS] 启动VINS节点...")
            vins_process = self.exec_in_container_background(
                f"rosrun vins vins_node {config}",
                "vins_node"
            )
            time.sleep(3)
            
            # 2. 播放rosbag
            print(f"[VINS] 处理数据集: {bag_name}")
            self.exec_in_container(
                f"rosbag play {container_bag_path} --clock",
                timeout=self.config.processing_timeout
            )
            
            # 等待输出文件生成
            time.sleep(5)
            
            # 3. 停止VINS节点
            self.exec_in_container("pkill -f 'vins_node'")
            time.sleep(2)
            
            # 4. 复制VINS输出的CSV文件
            # VINS默认输出到~/.ros/目录
            vins_output_files = [
                '/root/.ros/vins_result_loop.csv',
                '/root/.ros/vins_result_no_loop.csv',
                '/root/.ros/vio.csv'
            ]
            
            for vins_file in vins_output_files:
                check_result = self.exec_in_container(f"test -f {vins_file} && echo exists")
                if 'exists' in check_result.stdout:
                    copy_cmd = f"docker cp {self.config.container_name}:{vins_file} {output_csv}"
                    self._run_command(copy_cmd)
                    print(f"[VINS] 已复制输出文件: {vins_file}")
                    break
            
            # 5. 读取轨迹
            if os.path.exists(output_csv):
                trajectory = self._read_vins_csv(output_csv)
                result['success'] = True
                result['output_file'] = output_csv
                result['trajectory'] = trajectory
            else:
                result['error'] = "未找到VINS输出文件"
            
        except Exception as e:
            result['error'] = str(e)
            print(f"[错误] VINS处理失败: {e}")
        
        return result
    
    def _extract_trajectory_from_bag(self, bag_path: str) -> List[Dict]:
        """从rosbag提取轨迹"""
        trajectory = []
        
        try:
            from rosbags.rosbag1 import Reader
            from rosbags.typesys import Stores, get_typestore
            
            typestore = get_typestore(Stores.ROS1_NOETIC)
            
            with Reader(bag_path) as reader:
                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == '/vins_estimator/odometry':
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
            print("[警告] rosbags库未安装，无法解析rosbag")
        except Exception as e:
            print(f"[警告] 解析rosbag失败: {e}")
        
        return trajectory
    
    def _read_vins_csv(self, csv_path: str) -> List[Dict]:
        """读取VINS输出的CSV文件"""
        trajectory = []
        
        try:
            with open(csv_path, 'r') as f:
                # 跳过注释行
                lines = [l for l in f.readlines() if not l.startswith('#')]
            
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) >= 8:
                    pose = {
                        'timestamp': float(parts[0]) / 1e9,  # ns -> s
                        'position': [float(parts[1]), float(parts[2]), float(parts[3])],
                        'quaternion': [float(parts[4]), float(parts[5]), 
                                      float(parts[6]), float(parts[7])]
                    }
                    trajectory.append(pose)
        
        except Exception as e:
            print(f"[警告] 读取CSV失败: {e}")
        
        return trajectory


class VINSResultParser:
    """VINS结果解析器"""
    
    @staticmethod
    def parse_odometry_csv(csv_path: str) -> List[Dict]:
        """解析VINS输出的轨迹CSV"""
        trajectory = []
        
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if row[0].startswith('#'):
                    continue
                
                try:
                    pose = {
                        'timestamp': float(row[0]) / 1e9,
                        'position': np.array([float(row[1]), float(row[2]), float(row[3])]),
                        'quaternion': np.array([float(row[4]), float(row[5]), 
                                               float(row[6]), float(row[7])]),
                    }
                    trajectory.append(pose)
                except (ValueError, IndexError):
                    continue
        
        return trajectory
    
    @staticmethod
    def interpolate_pose(trajectory: List[Dict], timestamp: float) -> Optional[Dict]:
        """在轨迹中插值获取指定时间的位姿"""
        if not trajectory:
            return None
        
        # 找到最近的两个时间点
        prev_pose = None
        next_pose = None
        
        for pose in trajectory:
            if pose['timestamp'] <= timestamp:
                prev_pose = pose
            elif pose['timestamp'] > timestamp:
                next_pose = pose
                break
        
        if prev_pose is None:
            return trajectory[0]
        if next_pose is None:
            return trajectory[-1]
        
        # 线性插值
        t = (timestamp - prev_pose['timestamp']) / (next_pose['timestamp'] - prev_pose['timestamp'])
        
        interp_pose = {
            'timestamp': timestamp,
            'position': prev_pose['position'] * (1-t) + next_pose['position'] * t,
            'quaternion': prev_pose['quaternion']  # 简化：使用前一个四元数
        }
        
        return interp_pose


if __name__ == "__main__":
    # 测试代码
    vins = VINSDockerInterface()
    
    # 检查状态
    status = vins.check_docker_status()
    print(f"Docker状态: {status}")
    
    # 如果需要处理数据
    # result = vins.process_bag_offline(
    #     "~/data/dataset.bag",
    #     "~/data/vins_output.csv"
    # )
    # print(f"处理结果: {result}")
