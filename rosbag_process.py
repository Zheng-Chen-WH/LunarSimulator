import sys
import numpy as np
import struct
from pathlib import Path
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg
import csv

args = {"task": "inspect",  # validate: 验证时间戳同步性（检查尺度漂移根源）
                            # interpolation： 时间戳归零并插值IMU
                            # downsample: 自动降采样图像频率
                            # inspect: 检查bag内容与完整性，不完整时需要fix
                            # fix: 修复序列号不完整的bag文件
                            # extract: 导出bag内容为csv文件，需要配合topic使用
        "path_in": r"D:\Codes\LunarSimulator\dataset\two_turns_trajectory\dataset_two_turns_downsampled.bag",    # 输入bag路径，例:r"D:\Codes\LunarSimulator\dataset\20260115_153825\dataset_final.bag"
                            # None表示自动在dataset文件夹下查找最新
        "path_out": None,    # 输出bag路径，None表示自动生成
        "topic": "/ground_truth/pose",    # 仅在task为extract时使用，指定导出话题
        "target_hz": 2.0     # 降采样目标频率
         }
# 获取类型存储
typestore = get_typestore(Stores.ROS1_NOETIC)

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
            output_csv = self.bag_path.stem + ".csv"
        
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
            elif 'Imu' in msgtype:
                trajectory_data = self._extract_imu(reader, connection)
            else:
                print(f"警告：不支持的消息类型 '{msgtype}'")
                print("支持的类型: Odometry, PoseStamped, PoseWithCovarianceStamped, Path, Imu")
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
    
    def _extract_imu(self, reader, connection):
        """提取IMU消息（加速度、角速度、方向）"""
        data = []
        
        for connection_iter, timestamp, rawdata in reader.messages(connections=[connection]):
            msg = self.typestore.deserialize_ros1(rawdata, connection.msgtype)
            
            # 提取时间戳（纳秒转秒）
            ts = timestamp / 1e9
            
            # 提取方向（四元数）
            quat = msg.orientation
            qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
            
            # 提取角速度（rad/s）
            gyro = msg.angular_velocity
            wx, wy, wz = gyro.x, gyro.y, gyro.z
            
            # 提取线性加速度（m/s^2）
            accel = msg.linear_acceleration
            ax, ay, az = accel.x, accel.y, accel.z
            
            data.append({
                'timestamp': ts,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                'wx': wx, 'wy': wy, 'wz': wz,
                'ax': ax, 'ay': ay, 'az': az
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

def validate_timestamps(bag_path):
    """
    验证Bag中不同Topic的时间戳一致性
    用于诊断 VINS 尺度漂移问题
    """
    print(f"\n正在验证时间戳同步性: {bag_path}")
    print("=" * 60)
    
    # 定义关注的Topic
    img_topic = '/cam0/image_raw'
    imu_topic = '/imu0'
    
    img_timestamps = []
    imu_timestamps = []
    
    try:
        with Reader(str(bag_path)) as reader:
            for connection, timestamp, rawdata in reader.messages():
                # timestamp 是 bag 记录的时间（也就是 dataset_saver 写入的 timestamp_ns）
                ts_sec = timestamp / 1e9
                
                if connection.topic == img_topic:
                    img_timestamps.append(ts_sec)
                elif connection.topic == imu_topic:
                    imu_timestamps.append(ts_sec)
                    
        if not img_timestamps:
            print(f"错误: 未找到图像数据 ({img_topic})")
            return
        if not imu_timestamps:
            print(f"错误: 未找到IMU数据 ({imu_topic})")
            return
            
        img_start, img_end = img_timestamps[0], img_timestamps[-1]
        imu_start, imu_end = imu_timestamps[0], imu_timestamps[-1]
        
        img_duration = img_end - img_start
        imu_duration = imu_end - imu_start
        
        print(f"{'传感器':<10} | {'开始时间(s)':<15} | {'结束时间(s)':<15} | {'总时长(s)':<15} | {'消息数':<10}")
        print("-" * 80)
        print(f"{'Image':<10} | {img_start:<15.4f} | {img_end:<15.4f} | {img_duration:<15.4f} | {len(img_timestamps):<10}")
        print(f"{'IMU':<10} | {imu_start:<15.4f} | {imu_end:<15.4f} | {imu_duration:<15.4f} | {len(imu_timestamps):<10}")
        print("-" * 80)
        
        ratio = img_duration / imu_duration if imu_duration > 0 else float('inf')
        print(f"\n时间膨胀比率 (Image时长 / IMU时长): {ratio:.2f} 倍")
        
        # 频率稳定性分析
        print(f"\n{'传感器':<10} | {'平均频率(Hz)':<15} | {'平均间隔(ms)':<15} | {'最大间隔(ms)':<15} | {'间隔抖动(ms)':<15}")
        print("-" * 85)
        
        def analyze_freq(timestamps, name):
            if len(timestamps) < 2: return
            diffs = np.diff(timestamps)
            mean_dt = np.mean(diffs)
            std_dt = np.std(diffs)
            max_dt = np.max(diffs)
            freq = 1.0 / mean_dt if mean_dt > 0 else 0
            
            print(f"{name:<10} | {freq:<15.2f} | {mean_dt*1000:<15.2f} | {max_dt*1000:<15.2f} | {std_dt*1000:<15.2f}")
            
            if max_dt > mean_dt * 5:
                 print(f"  [警告] {name} 存在严重丢帧或卡顿! 最大间隔 {max_dt*1000:.1f}ms 是平均值的 {max_dt/mean_dt:.1f} 倍")

        analyze_freq(img_timestamps, "Image")
        analyze_freq(imu_timestamps, "IMU")
        print("-" * 85)

        if abs(ratio - 1.0) > 0.1:
            print("\n[严重的同步问题检测]")
            print("!!! 警告: 图像和IMU的时间流逝速度完全不同 !!!")
            print("这是一个典型的时钟域不匹配问题：")
            print("  - 图像可能使用了 wall-clock (time.time())")
            print("  - IMU 可能使用了 sim-clock (仿真时间)")
            print(f"  VINS 会认为机器人在 {img_duration:.2f} 秒内只走了 {imu_duration:.2f} 秒路程对应的位移。")
            print("  这会导致极大的尺度漂移 (Scale Drift)。")
        else:
            print("\n[通过] 两个传感器的时间流逝速度基本一致。")
            print("如果仍然出现严重尺度漂移，请考虑以下可能：")
            print("1. VINS参数中的 accelerometer_noise_density (acc_n) 设置过大，导致系统不信任IMU尺度。")
            print("2. 外部参数 body_T_cam 的平移部分与实际模型不符（检查基线 Baseline）。")
            print("3. 月球表面纹理过弱，导致特征点跟踪失败，立体匹配退化为单目（尺度不可观）。")
            
    except Exception as e:
        print(f"验证失败: {e}")

def interpolation_bag(input_path, output_path):
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

def downsample_bag(input_path, output_path, target_hz):
    """
    降采样图像频率，通过跳过指定时间间隔内的图像消息
    """
    print(f"正在进行图像降采样: {input_path}")
    print(f"目标频率: {target_hz} Hz")
    
    # 记录每个图像topic最后保留的时间戳 (ns)
    last_kept_ts = {} 
    min_interval_ns = int(1e9 / target_hz)

    with Reader(str(input_path)) as reader, Writer(str(output_path)) as writer:
        conn_map = {}
        for conn in reader.connections:
            # 这里的 typestore 已经全局定义
            new_conn = writer.add_connection(conn.topic, conn.msgtype, typestore=typestore)
            conn_map[conn.id] = new_conn
            
        count_total = 0
        count_kept_img = 0
        count_skipped_img = 0
        
        for connection, timestamp, rawdata in reader.messages():
            # 检查是否是图像消息
            is_image = 'Image' in connection.msgtype
            
            if is_image:
                topic = connection.topic
                # 如果间隔足够大，或者该topic从未记录过，则保留
                if topic not in last_kept_ts or (timestamp - last_kept_ts[topic]) >= min_interval_ns:
                    writer.write(conn_map[connection.id], timestamp, rawdata)
                    last_kept_ts[topic] = timestamp
                    count_kept_img += 1
                else:
                    count_skipped_img += 1
                    continue
            else:
                # 非图像消息直接写入（如IMU、里程计等）
                writer.write(conn_map[connection.id], timestamp, rawdata)
                
            count_total += 1
            if count_total % 2000 == 0:
                print(f"已处理 {count_total} 条消息...")

    print(f"\n降采样完成!")
    print(f"保留图像数: {count_kept_img}")
    print(f"丢弃图像数: {count_skipped_img}")
    print(f"输出文件: {output_path}")

def fix_bag(input_bag, output_bag=None):
    """
    修复损坏的ROS bag文件（索引丢失）
    通过重新读取所有消息并写入新bag文件来修复
    Args:
        input_bag: 输入的损坏bag文件
        output_bag: 输出的修复后bag文件（默认为input_bag.fixed.bag）
    """
    input_path = Path(input_bag)
    
    if output_bag is None:
        output_bag = input_path.parent / f"{input_path.stem}.fixed.bag"
    else:
        output_bag = Path(output_bag)
    
    print(f"正在修复bag文件")
    print(f"输入: {input_path}")
    print(f"输出: {output_bag}")
    
    typestore = get_typestore(Stores.ROS1_NOETIC)
    
    try:
        # 读取损坏的bag
        with Reader(str(input_path)) as reader:
            print(f"\n读取到 {len(reader.connections)} 个topic连接")
            
            # 创建新的bag文件
            with Writer(str(output_bag)) as writer:
                # 复制所有连接
                connection_map = {}
                for conn in reader.connections:
                    print(f"  - {conn.topic} [{conn.msgtype}]")
                    new_conn = writer.add_connection(
                        conn.topic,
                        conn.msgtype,
                        typestore=typestore
                    )
                    connection_map[conn.id] = new_conn
                
                print(f"\n开始复制消息")
                message_count = 0
                
                # 复制所有消息
                for connection, timestamp, rawdata in reader.messages():
                    new_conn = connection_map[connection.id]
                    writer.write(new_conn, timestamp, rawdata)
                    message_count += 1
                    
                    if message_count % 100 == 0:
                        print(f"  已复制 {message_count} 条消息")
                
                print(f"\n总共复制了 {message_count} 条消息")
        
        print(f"\n修复完成！")
        print(f"新bag文件: {output_bag}")
        
    except Exception as e:
        print(f"\n✗ 修复失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

def check_bag_integrity(bag_path):
    """
    检查bag文件的完整性和索引状态
    """
    bag_path = Path(bag_path)
    
    if not bag_path.exists():
        print(f"文件不存在: {bag_path}")
        return False
    
    print("=" * 70)
    print(f"检查 bag 文件: {bag_path.name}")
    print("=" * 70)
    
    # 检查文件大小
    file_size = bag_path.stat().st_size
    print(f"\n文件大小: {file_size / 1024 / 1024:.2f} MB ({file_size} bytes)")
    
    if file_size == 0:
        print("文件为空!")
        return False
    
    # 检查文件头和结构
    print("\n检查文件结构")
    try:
        with open(bag_path, 'rb') as f:
            # 读取文件头
            header = f.read(13)
            if len(header) < 13:
                print("文件头不完整")
                return False
            
            # 检查magic string
            if header[:13] != b'#ROSBAG V2.0\n':
                print(f"不是有效的ROS bag V2.0文件")
                print(f"文件头: {header[:20]}")
                return False
            
            print("文件头正常 (ROSBAG V2.0)")
            
            # 移到文件末尾检查
            f.seek(-min(4096, file_size), 2)  # 从末尾往前4KB或文件大小
            tail_data = f.read()
            
            # 查找索引数据记录（op=4）
            has_index_data = False
            pos = 0
            while pos < len(tail_data) - 8:
                try:
                    # 读取header长度
                    header_len = struct.unpack('<I', tail_data[pos:pos+4])[0]
                    if header_len > 0 and header_len < 10000:  # 合理的header长度
                        # 读取op code
                        op = struct.unpack('B', tail_data[pos+4:pos+5])[0]
                        if op == 4:  # INDEX_DATA record
                            has_index_data = True
                            break
                    pos += 1
                except:
                    pos += 1
            
            if has_index_data:
                print("文件末尾包含索引数据结构")
            else:
                print("文件末尾未找到索引数据结构（可能unindexed）")
    
    except Exception as e:
        print(f"文件结构检查失败: {e}")
        return False
    
    # 使用rosbags库检查
    print("\n使用rosbags库深度检查")
    try:
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        print("未安装rosbags库，无法进行深度检查")
        print("请运行: pip install rosbags")
        return None
    
    try:
        typestore = get_typestore(Stores.ROS1_NOETIC)
        
        print("  尝试打开bag文件")
        with Reader(str(bag_path)) as reader:
            # 检查连接
            num_connections = len(reader.connections)
            print(f"  成功打开bag文件")
            print(f"  找到 {num_connections} 个topic连接\n")
            
            if num_connections == 0:
                print("bag文件为空（没有任何topic）")
                return False
            
            # 列出所有topic
            print("Topic列表:")
            print("-" * 70)
            total_messages = 0
            for conn in reader.connections:
                print(f"  {conn.topic:<40} [{conn.msgtype}]")
                print(f"    消息数量: {conn.msgcount}")
                total_messages += conn.msgcount
            print("-" * 70)
            print(f"  总消息数: {total_messages}\n")
            
            if total_messages == 0:
                print("bag文件中没有消息")
                return False
            
            # 尝试读取消息
            try:
                message_count = 0
                first_timestamp = None
                last_timestamp = None
                
                for connection, timestamp, rawdata in reader.messages():
                    if first_timestamp is None:
                        first_timestamp = timestamp
                    last_timestamp = timestamp
                    message_count += 1
                    
                    if message_count >= 10:  # 读取前10条测试
                        break
                
                print(f"  成功读取消息 (测试了前{message_count}条)")
                if first_timestamp and last_timestamp:
                    duration = (last_timestamp - first_timestamp) / 1e9
                    print(f"  时间跨度: {duration:.2f} 秒")
                
            except Exception as e:
                print(f"  读取消息失败: {e}")
                print(f"\n这通常意味着bag文件索引损坏或不完整")
                return False
        
        print("\n" + "=" * 70)
        print("Bag文件完整，可以正常使用")
        print("=" * 70)
        return True
        
    except Exception as e:
        print(f"\nBag文件检查失败!")
        print(f"错误类型: {type(e).__name__}")
        print(f"错误信息: {e}")
        
        # 检查是否是索引相关错误
        error_str = str(e).lower()
        if 'index' in error_str or 'unindexed' in error_str:
            print("\nbag文件索引丢失或损坏")
        
        print("\n" + "=" * 70)
        print("Bag文件损坏，需要修复")
        print("请在args中修改task为fix进行修复")
        print("=" * 70)
        return False

if args['path_in'] is None:
    # 自动查找最新的bag
    base_dir = Path(r"D:\Codes\LunarSimulator\dataset")
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
elif args['path_out'] is None:
    input_bag = Path(args['path_in'])
    output_bag = input_bag.parent / f"{input_bag.stem}.fixed.bag"

# 执行不同处理任务
if args['task'] == 'fix':    
    fix_bag(str(input_bag), str(output_bag))
elif args['task'] == 'interpolation':
    interpolation_bag(str(input_bag), str(output_bag))
elif args['task'] == 'downsample':
    downsample_bag(str(input_bag), str(output_bag), args['target_hz'])
elif args['task'] == 'inspect':
    result = check_bag_integrity(str(input_bag))
elif args['task'] == 'extract':
    print("ROS Bag 轨迹提取工具")
    # 显示当前配置
    print(f"  Bag文件: {str(input_bag)}")
    print(f"  提取话题: {args['topic']}")
    print(f"  输出文件: {args['path_out'] if args['path_out'] else '自动生成'}")
    print()
    extractor = BagTrajectoryExtractor(str(input_bag))
    if not args['topic']:
        print("错误：MODE='extract' 时必须指定 TOPIC 变量")
        print("例：'/vins_estimator/odometry'")
        sys.exit(1)
    extractor.extract_trajectory(args['topic'], args['path_out'])
elif args['task'] == 'validate':
    validate_timestamps(str(input_bag))
