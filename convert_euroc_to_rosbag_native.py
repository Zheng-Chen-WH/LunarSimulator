"""
将EuRoC格式数据集转换为ROS bag（在Docker容器内原生录制）
这样可以避免rosbags库的兼容性问题

使用方法：
1. 将此脚本复制到Docker容器
2. 在容器内运行：python3 convert_euroc_to_rosbag_native.py

作者: GitHub Copilot
日期: 2026-01-13
"""

import os
import sys
import csv
import cv2
import numpy as np
from pathlib import Path
import subprocess
import time


def generate_rosbag_script(dataset_path: str, output_bag: str) -> str:
    """
    生成一个shell脚本，使用ROS原生工具发布数据并录制
    """
    
    script_content = f"""#!/bin/bash
# EuRoC数据集 -> ROS bag 转换脚本
# 在Docker容器内运行

DATASET_PATH="{dataset_path}"
OUTPUT_BAG="{output_bag}"
MAV_PATH="$DATASET_PATH/mav0"

echo "开始转换 EuRoC 数据集到 ROS bag"
echo "数据集: $DATASET_PATH"
echo "输出: $OUTPUT_BAG"

# 检查数据集
if [ ! -d "$MAV_PATH" ]; then
    echo "错误: 找不到 mav0 目录"
    exit 1
fi

# Source ROS
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash

# 启动roscore（如果未运行）
if ! pgrep -x "roscore" > /dev/null; then
    echo "启动 roscore..."
    roscore &
    sleep 3
fi

# 启动录制
echo "启动 rosbag 录制..."
rosbag record -O $OUTPUT_BAG \\
    /imu0 \\
    /star_tracker/attitude \\
    /wheel_odometry/twist \\
    /ground_truth/pose \\
    /ground_truth/twist \\
    /obstacle_camera/left/image_raw \\
    /obstacle_camera/right/image_raw &

RECORD_PID=$!
sleep 2

# 创建Python发布脚本
cat > /tmp/publish_euroc_data.py << 'PYTHON_EOF'
import rospy
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseStamped, TwistStamped, QuaternionStamped, Vector3
from cv_bridge import CvBridge
import cv2
import csv
import numpy as np
from pathlib import Path

def read_csv_data(csv_path):
    data = []
    with open(csv_path, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split(',')
            if parts:
                data.append(parts)
    return data

def publish_euroc_data(dataset_path):
    rospy.init_node('euroc_publisher', anonymous=True)
    
    # 发布者
    imu_pub = rospy.Publisher('/imu0', Imu, queue_size=100)
    gt_pose_pub = rospy.Publisher('/ground_truth/pose', PoseStamped, queue_size=100)
    
    mav_path = Path(dataset_path) / "mav0"
    
    # 读取IMU数据
    imu_csv = mav_path / "imu0" / "data.csv"
    print(f"读取IMU数据: {{imu_csv}}")
    imu_data = read_csv_data(str(imu_csv))
    
    # 读取真值数据
    gt_csv = mav_path / "state_groundtruth_estimate0" / "data.csv"
    print(f"读取真值数据: {{gt_csv}}")
    gt_data = read_csv_data(str(gt_csv))
    
    rate = rospy.Rate(100)  # 100Hz
    
    print(f"开始发布数据...")
    print(f"IMU数据: {{len(imu_data)}} 帧")
    print(f"真值数据: {{len(gt_data)}} 帧")
    
    # 发布IMU数据
    for i, row in enumerate(imu_data):
        if rospy.is_shutdown():
            break
        
        if len(row) < 7:
            continue
        
        timestamp_ns = int(row[0])
        timestamp = rospy.Time(timestamp_ns // 1000000000, timestamp_ns % 1000000000)
        
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = "imu"
        
        imu_msg.angular_velocity.x = float(row[1])
        imu_msg.angular_velocity.y = float(row[2])
        imu_msg.angular_velocity.z = float(row[3])
        
        imu_msg.linear_acceleration.x = float(row[4])
        imu_msg.linear_acceleration.y = float(row[5])
        imu_msg.linear_acceleration.z = float(row[6])
        
        imu_pub.publish(imu_msg)
        
        # 发布真值（如果有对应时间戳）
        for gt_row in gt_data:
            if len(gt_row) < 14:
                continue
            gt_ts = int(gt_row[0])
            if abs(gt_ts - timestamp_ns) < 5000000:  # 5ms内
                pose_msg = PoseStamped()
                pose_msg.header.stamp = timestamp
                pose_msg.header.frame_id = "world"
                
                pose_msg.pose.position.x = float(gt_row[1])
                pose_msg.pose.position.y = float(gt_row[2])
                pose_msg.pose.position.z = float(gt_row[3])
                
                pose_msg.pose.orientation.w = float(gt_row[4])
                pose_msg.pose.orientation.x = float(gt_row[5])
                pose_msg.pose.orientation.y = float(gt_row[6])
                pose_msg.pose.orientation.z = float(gt_row[7])
                
                gt_pose_pub.publish(pose_msg)
                break
        
        if (i + 1) % 1000 == 0:
            print(f"已发布 {{i+1}}/{{len(imu_data)}} 帧")
        
        rate.sleep()
    
    print("数据发布完成")

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("用法: python3 publish_euroc_data.py <dataset_path>")
        sys.exit(1)
    
    dataset_path = sys.argv[1]
    publish_euroc_data(dataset_path)
PYTHON_EOF

# 运行Python发布脚本
echo "发布数据..."
python3 /tmp/publish_euroc_data.py "$DATASET_PATH"

# 等待一下确保所有消息都被录制
sleep 3

# 停止录制
echo "停止录制..."
kill $RECORD_PID
wait $RECORD_PID

# 清理
rm /tmp/publish_euroc_data.py

echo "转换完成！"
echo "输出文件: $OUTPUT_BAG"
rosbag info "$OUTPUT_BAG"
"""
    
    return script_content


def main():
    print("EuRoC到ROS bag转换工具（原生ROS录制）")
    print("="*60)
    
    # 获取参数
    if len(sys.argv) < 2:
        print("用法: python3 convert_euroc_to_rosbag_native.py <dataset_path>")
        print("示例: python3 convert_euroc_to_rosbag_native.py /root/data/dataset/20260113_164908")
        sys.exit(1)
    
    dataset_path = sys.argv[1]
    output_bag = sys.argv[2] if len(sys.argv) >= 3 else f"{dataset_path}_native.bag"
    
    # 生成脚本
    script_content = generate_rosbag_script(dataset_path, output_bag)
    
    script_path = "/tmp/convert_to_rosbag.sh"
    with open(script_path, 'w') as f:
        f.write(script_content)
    
    os.chmod(script_path, 0o755)
    
    print(f"生成转换脚本: {script_path}")
    print("运行脚本进行转换...")
    
    # 运行脚本
    result = subprocess.run(["/bin/bash", script_path], 
                          capture_output=False, 
                          text=True)
    
    if result.returncode == 0:
        print("\n转换成功！")
    else:
        print(f"\n转换失败，返回码: {result.returncode}")


if __name__ == "__main__":
    main()
