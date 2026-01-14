"""
从ROS bag文件提取轨迹数据并保存为CSV
支持多种轨迹话题类型

使用方法：
1. 修改下面的配置变量
2. 在VSCode中直接运行（F5或右键运行）
"""

import csv
from pathlib import Path
import sys

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

# ==================== 配置区域 ====================
# 请根据需要修改以下参数

# ROS bag文件路径
BAG_FILE = r"./vins_output.bag"

# 操作模式：'list' 或 'extract'
# 'list': 列出bag中的所有话题
# 'extract': 从指定话题提取轨迹
MODE = 'extract'

# 要提取的话题名称（仅当MODE='extract'时使用）
TOPIC = '/vins_estimator/odometry'

# 输出CSV文件路径（None表示自动生成，仅当MODE='extract'时使用）
OUTPUT_CSV = None  # 例如: r"C:\output\trajectory.csv"

# ==================== 配置区域结束 ====================

class BagTrajectoryExtractor:
    """从ROS bag提取轨迹数据"""
    
    def __init__(self, bag_path):
        """
        初始化提取器
        Args:
            bag_path: bag文件路径
        """
        self.bag_path = Path(bag_path)
        if not self.bag_path.exists():
            raise FileNotFoundError(f"Bag文件不存在: {bag_path}")
        
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        
    def list_topics(self):
        """列出bag文件中的所有话题"""
        print(f"\n读取bag文件: {self.bag_path}")
        print("\n可用话题:")
        print("-" * 80)
        
        with Reader(str(self.bag_path)) as reader:
            for connection in reader.connections:
                print(f"  {connection.topic:<50} [{connection.msgtype}]")
                print(f"    消息数量: {connection.msgcount}")
        
        print("-" * 80)
    
    def extract_trajectory(self, topic, output_csv=None):
        """
        从指定话题提取轨迹
        Args:
            topic: 话题名称
            output_csv: 输出CSV文件路径（可选）
        Returns:
            trajectory_data: 轨迹数据列表
        """
        if output_csv is None:
            output_csv = self.bag_path.stem + "_trajectory.csv"
        
        output_path = Path(output_csv)
        
        trajectory_data = []
        
        print(f"\n从话题 '{topic}' 提取轨迹数据...")
        
        with Reader(str(self.bag_path)) as reader:
            # 查找匹配的连接
            connections = [c for c in reader.connections if c.topic == topic]
            
            if not connections:
                print(f"错误：未找到话题 '{topic}'")
                print("请使用 --list 参数查看可用话题")
                return []
            
            connection = connections[0]
            msgtype = connection.msgtype
            
            print(f"话题类型: {msgtype}")
            print(f"消息数量: {connection.msgcount}")
            
            # 根据消息类型提取数据
            if 'Odometry' in msgtype:
                trajectory_data = self._extract_odometry(reader, connection)
            elif 'PoseStamped' in msgtype:
                trajectory_data = self._extract_pose_stamped(reader, connection)
            elif 'PoseWithCovarianceStamped' in msgtype:
                trajectory_data = self._extract_pose_with_covariance(reader, connection)
            elif 'Path' in msgtype:
                trajectory_data = self._extract_path(reader, connection)
            else:
                print(f"警告：不支持的消息类型 '{msgtype}'")
                print("支持的类型: Odometry, PoseStamped, PoseWithCovarianceStamped, Path")
                return []
        
        if trajectory_data:
            self._save_to_csv(trajectory_data, output_path)
            print(f"\n成功提取 {len(trajectory_data)} 个位姿")
            print(f"已保存到: {output_path}")
        else:
            print("警告：未提取到任何数据")
        
        return trajectory_data
    
    def _extract_odometry(self, reader, connection):
        """提取Odometry消息"""
        data = []
        
        for connection_iter, timestamp, rawdata in reader.messages(connections=[connection]):
            msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
            
            # 提取时间戳（纳秒转秒）
            ts = timestamp / 1e9
            
            # 提取位置
            pos = msg.pose.pose.position
            tx, ty, tz = pos.x, pos.y, pos.z
            
            # 提取姿态（四元数）
            quat = msg.pose.pose.orientation
            qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
            
            # 提取速度（如果有）
            vel = msg.twist.twist.linear
            vx, vy, vz = vel.x, vel.y, vel.z
            
            data.append({
                'timestamp': ts,
                'tx': tx, 'ty': ty, 'tz': tz,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                'vx': vx, 'vy': vy, 'vz': vz
            })
        
        return data
    
    def _extract_pose_stamped(self, reader, connection):
        """提取PoseStamped消息"""
        data = []
        
        for connection_iter, timestamp, rawdata in reader.messages(connections=[connection]):
            msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
            
            ts = timestamp / 1e9
            
            pos = msg.pose.position
            tx, ty, tz = pos.x, pos.y, pos.z
            
            quat = msg.pose.orientation
            qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
            
            data.append({
                'timestamp': ts,
                'tx': tx, 'ty': ty, 'tz': tz,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
            })
        
        return data
    
    def _extract_pose_with_covariance(self, reader, connection):
        """提取PoseWithCovarianceStamped消息"""
        data = []
        
        for connection_iter, timestamp, rawdata in reader.messages(connections=[connection]):
            msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
            
            ts = timestamp / 1e9
            
            pos = msg.pose.pose.position
            tx, ty, tz = pos.x, pos.y, pos.z
            
            quat = msg.pose.pose.orientation
            qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
            
            data.append({
                'timestamp': ts,
                'tx': tx, 'ty': ty, 'tz': tz,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
            })
        
        return data
    
    def _extract_path(self, reader, connection):
        """提取Path消息（包含多个位姿）"""
        data = []
        
        for connection_iter, timestamp, rawdata in reader.messages(connections=[connection]):
            msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
            
            ts = timestamp / 1e9
            
            # Path包含多个pose
            for pose_stamped in msg.poses:
                pos = pose_stamped.pose.position
                tx, ty, tz = pos.x, pos.y, pos.z
                
                quat = pose_stamped.pose.orientation
                qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
                
                data.append({
                    'timestamp': ts,
                    'tx': tx, 'ty': ty, 'tz': tz,
                    'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
                })
        
        return data
    
    def _save_to_csv(self, data, output_path):
        """保存为CSV文件"""
        if not data:
            return
        
        # 获取所有字段
        fieldnames = list(data[0].keys())
        
        with open(output_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            # 写入表头
            writer.writeheader()
            
            # 写入数据
            for row in data:
                writer.writerow(row)
        
        print(f"CSV列: {', '.join(fieldnames)}")


def main():
    """主函数"""
    print("=" * 80)
    print("ROS Bag 轨迹提取工具")
    print("=" * 80)
    
    # 显示当前配置
    print(f"\n当前配置:")
    print(f"  Bag文件: {BAG_FILE}")
    print(f"  操作模式: {MODE}")
    if MODE == 'extract':
        print(f"  提取话题: {TOPIC}")
        print(f"  输出文件: {OUTPUT_CSV if OUTPUT_CSV else '自动生成'}")
    print()
    
    try:
        extractor = BagTrajectoryExtractor(BAG_FILE)
        
        if MODE == 'list':
            extractor.list_topics()
            print("\n提示：")
            print("1. 查看上面的话题列表")
            print("2. 选择要提取的话题（如 /vins_estimator/odometry）")
            print("3. 修改代码顶部的 MODE='extract' 和 TOPIC='你选择的话题'")
            print("4. 重新运行程序")
            
        elif MODE == 'extract':
            if not TOPIC:
                print("错误：MODE='extract' 时必须指定 TOPIC 变量")
                print("请在代码顶部设置 TOPIC 变量，例如:")
                print("  TOPIC = '/vins_estimator/odometry'")
                sys.exit(1)
            
            extractor.extract_trajectory(TOPIC, OUTPUT_CSV)
            
        else:
            print(f"错误：不支持的MODE值 '{MODE}'")
            print("MODE 必须是 'list' 或 'extract'")
            sys.exit(1)
    
    except FileNotFoundError as e:
        print(f"\n错误: {e}")
        print("\n请检查:")
        print("1. BAG_FILE 路径是否正确")
        print("2. 文件是否存在")
        print(f"\n当前设置的路径: {BAG_FILE}")
    
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    print("\n" + "=" * 80)
    print("处理完成")
    print("=" * 80)


if __name__ == "__main__":
    main()
