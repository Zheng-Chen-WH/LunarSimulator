"""
ROS Bag 修复工具 V2
功能：
1. IMU 频率插值 (提升至 100Hz)
2. 时间戳归零 (解决初始时间为 26s 导致的 EKF 初始化问题)
"""
import sys
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader, Writer
# from rosbags.serde import deserialize_ros1, serialize_ros1
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg

# 获取类型存储
typestore = get_typestore(Stores.ROS1_NOETIC)

def get_start_time(bag_path):
    """获取bag的起始时间"""
    with Reader(bag_path) as reader:
        # 简单取第一个连接的第一个消息的时间，或者遍历一遍找最小值
        # 为了效率，我们只看前几条
        min_ts = float('inf')
        count = 0
        for _, timestamp, _ in reader.messages():
            if timestamp < min_ts:
                min_ts = timestamp
            count += 1
            if count > 1000: # 假设数据大概是按时间排序的
                break
        if min_ts == float('inf'):
            return 0
        return min_ts

def create_time(timestamp_ns):
    """从纳秒创建ROS Time对象"""
    sec = int(timestamp_ns // 1e9)
    nanosec = int(timestamp_ns % 1e9)
    return typestore.types['builtin_interfaces/msg/Time'](sec=sec, nanosec=nanosec)

def offset_header(header, offset_ns):
    """修改Header的时间戳"""
    if header is None:
        return None
    old_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
    new_ns = old_ns - offset_ns
    if new_ns < 0: new_ns = 0
    
    return typestore.types['std_msgs/msg/Header'](
        seq=header.seq,
        stamp=create_time(new_ns),
        frame_id=header.frame_id
    )

def interpolate_imu(msg1, msg2, t1, t2, offset_ns):
    """IMU插值并应用时间偏移"""
    new_msgs = []
    target_rate = 100.0
    dt_ns = t2 - t1
    if dt_ns <= 0: return []
    
    interval_ns = int(1e9 / target_rate)
    num_steps = int(dt_ns / interval_ns)
    
    # 提取数据
    acc1 = np.array([msg1.linear_acceleration.x, msg1.linear_acceleration.y, msg1.linear_acceleration.z])
    gyr1 = np.array([msg1.angular_velocity.x, msg1.angular_velocity.y, msg1.angular_velocity.z])
    rot1 = np.array([msg1.orientation.w, msg1.orientation.x, msg1.orientation.y, msg1.orientation.z])
    
    acc2 = np.array([msg2.linear_acceleration.x, msg2.linear_acceleration.y, msg2.linear_acceleration.z])
    gyr2 = np.array([msg2.angular_velocity.x, msg2.angular_velocity.y, msg2.angular_velocity.z])
    rot2 = np.array([msg2.orientation.w, msg2.orientation.x, msg2.orientation.y, msg2.orientation.z])

    for i in range(1, num_steps):
        alpha = i / num_steps
        current_ts = int(t1 + i * interval_ns)
        
        # 应用时间偏移
        shifted_ts = current_ts - offset_ns
        if shifted_ts < 0: shifted_ts = 0
        
        # 插值
        acc_i = (1 - alpha) * acc1 + alpha * acc2
        gyr_i = (1 - alpha) * gyr1 + alpha * gyr2
        rot_i = (1 - alpha) * rot1 + alpha * rot2
        rot_i = rot_i / np.linalg.norm(rot_i)
        
        # 构造新Header
        new_header = typestore.types['std_msgs/msg/Header'](
            seq=0,
            stamp=create_time(shifted_ts),
            frame_id=msg1.header.frame_id
        )
        
        # 构造新消息
        new_imu = typestore.types['sensor_msgs/msg/Imu'](
            header=new_header,
            orientation=typestore.types['geometry_msgs/msg/Quaternion'](w=rot_i[0], x=rot_i[1], y=rot_i[2], z=rot_i[3]),
            orientation_covariance=msg1.orientation_covariance,
            angular_velocity=typestore.types['geometry_msgs/msg/Vector3'](x=gyr_i[0], y=gyr_i[1], z=gyr_i[2]),
            angular_velocity_covariance=msg1.angular_velocity_covariance,
            linear_acceleration=typestore.types['geometry_msgs/msg/Vector3'](x=acc_i[0], y=acc_i[1], z=acc_i[2]),
            linear_acceleration_covariance=msg1.linear_acceleration_covariance
        )
        
        new_msgs.append((shifted_ts, new_imu))
        
    return new_msgs

def fix_bag(input_path, output_path):
    print(f"正在读取起始时间: {input_path}")
    start_time_ns = get_start_time(input_path)
    print(f"检测到起始时间: {start_time_ns} ns ({start_time_ns/1e9:.2f} s)")
    print("将执行: 时间戳归零 + IMU插值")
    
    imu_topic = '/imu0'
    last_imu_msg = None
    last_imu_ts = 0

    with Reader(input_path) as reader, Writer(output_path) as writer:
        conn_map = {}
        for conn in reader.connections:
            # Revert to standard add_connection which handles type normalization automatically
            new_conn = writer.add_connection(conn.topic, conn.msgtype, typestore=typestore)
            conn_map[conn.id] = new_conn
            
        count = 0
        for connection, timestamp, rawdata in reader.messages():
            # 计算新的 Bag 时间
            new_ts = timestamp - start_time_ns
            if new_ts < 0: new_ts = 0
            
            # 反序列化并修改 Header
            try:
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            except Exception as e:
                # 忽略无法反序列化的类型，直接写入（时间戳会偏移，但Header不会，这可能导致不一致）
                writer.write(conn_map[connection.id], new_ts, rawdata)
                continue
            
            # 尝试修改 msg 的 Header
            if hasattr(msg, 'header'):
                new_header = offset_header(msg.header, start_time_ns)
                
                if connection.msgtype == 'sensor_msgs/msg/Imu':
                    # IMU 特殊处理 (插值)
                    if last_imu_msg is not None:
                         # 插值 (使用原始时间戳计算间隔，但生成偏移后的时间戳)
                        interpolated = interpolate_imu(last_imu_msg, msg, last_imu_ts, timestamp, start_time_ns)
                        for t, m in interpolated:
                            writer.write(conn_map[connection.id], t, typestore.serialize_ros1(m, connection.msgtype))
                    
                    # 更新 last_imu 信息
                    # 注意：保存原始的 last_imu_msg (数据) 和 原始时间戳，以便下一次正确插值
                    # 但为了写入当前帧，我们需要构造带有新Header的msg
                    
                    # 先保存当前帧原始数据用于下次插值
                    # (因为msg是刚反序列化的，带有旧Header，正好)
                    current_msg_raw = msg 
                    current_ts_raw = timestamp
                    
                    # 现在构造用于写入的msg (新Header)
                    msg_to_write = typestore.types['sensor_msgs/msg/Imu'](
                        header=new_header,
                        orientation=msg.orientation,
                        orientation_covariance=msg.orientation_covariance,
                        angular_velocity=msg.angular_velocity,
                        angular_velocity_covariance=msg.angular_velocity_covariance,
                        linear_acceleration=msg.linear_acceleration,
                        linear_acceleration_covariance=msg.linear_acceleration_covariance
                    )
                    
                    msg = msg_to_write # 用于下面的通用序列化写入
                    
                    # 更新 state
                    last_imu_msg = current_msg_raw
                    last_imu_ts = current_ts_raw
                    
                # 处理其他带 Header 的常见类型
                elif connection.msgtype == 'sensor_msgs/msg/Image':
                    msg = typestore.types['sensor_msgs/msg/Image'](
                        header=new_header,
                        height=msg.height, width=msg.width, encoding=msg.encoding,
                        is_bigendian=msg.is_bigendian, step=msg.step, data=msg.data
                    )
                elif connection.msgtype == 'nav_msgs/msg/Odometry':
                    msg = typestore.types['nav_msgs/msg/Odometry'](
                        header=new_header,
                        child_frame_id=msg.child_frame_id, pose=msg.pose, twist=msg.twist
                    )
                elif connection.msgtype == 'geometry_msgs/msg/PoseWithCovarianceStamped':
                    msg = typestore.types['geometry_msgs/msg/PoseWithCovarianceStamped'](
                        header=new_header, pose=msg.pose
                    )
                elif connection.msgtype == 'geometry_msgs/msg/PoseStamped':
                    msg = typestore.types['geometry_msgs/msg/PoseStamped'](
                        header=new_header, pose=msg.pose
                    )
                elif connection.msgtype == 'geometry_msgs/msg/TwistStamped':
                     msg = typestore.types['geometry_msgs/msg/TwistStamped'](
                        header=new_header, twist=msg.twist
                    )
                elif connection.msgtype == 'geometry_msgs/msg/QuaternionStamped':
                     msg = typestore.types['geometry_msgs/msg/QuaternionStamped'](
                        header=new_header, quaternion=msg.quaternion
                    )
                
            # 序列化并写入 (使用新的 bag time)
            writer.write(conn_map[connection.id], new_ts, typestore.serialize_ros1(msg, connection.msgtype))
            
            count += 1
            if count % 1000 == 0:
                print(f"已处理 {count} 条消息")

    print(f"处理完成: {output_path}")

if __name__ == "__main__":
    # 自动查找最新的bag
    base_dir = Path(r"c:\Users\Dendrobium\Desktop\Personal Docs\HKUST\LunarSimulator\dataset")
    if not base_dir.exists():
        print("未找到dataset目录")
        sys.exit(1)
        
    subdirs = [d for d in base_dir.iterdir() if d.is_dir()]
    if not subdirs:
        print("未找到数据子目录")
        sys.exit(1)
        
    latest_dir = max(subdirs, key=lambda x: x.stat().st_mtime)
    
    # 优先寻找原始的 dataset.bag
    input_bag = latest_dir / "dataset.bag"
    if not input_bag.exists():
        print(f"未找到 dataset.bag 在 {latest_dir}")
        sys.exit(1)
        
    output_bag = latest_dir / "dataset_final.bag"
    
    print(f"输入: {input_bag}")
    print(f"输出: {output_bag}")
    
    fix_bag(str(input_bag), str(output_bag))
