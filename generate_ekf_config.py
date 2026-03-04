import yaml
import os
import numpy as np

def generate_ekf_config():
    """
    生成 robot_localization (EKF) 的配置文件
    针对月球车低速场景：融合 VINS(姿态+角速度) + 轮速计(线速度) + 星敏感器(绝对姿态)
    
    注意：EKF 不需要相机外参，因为它融合的是 VINS 输出的已处理位姿，而不是原始图像。
    相机外参只在 VINS 配置中需要。
    """
    # ==========================================
    # 1. 基础参数配置
    # ==========================================
    config = {
        "frequency": 10,                 # EKF 运行频率 (Hz)
        "sensor_timeout": 0.1,           # 传感器超时时间 (s)，超过此时间未收到数据则认为传感器失效
        "two_d_mode": False,             # 是否为2D平面运动？(月球表面起伏，建议False)
        "transform_time_offset": 0.0,    # 变换发布的时间偏移
        "transform_timeout": 0.5,        # 增加等待时间 (0.1s)，解决 TF 延迟导致的报警
        "print_diagnostics": True,       # 是否打印诊断信息
        "debug": False,                  # Debug模式
        "publish_tf": False,             # 根据用户要求，由 VINS 发布 TF，EKF 不发布
        
        # 坐标系定义
        "map_frame": "map",              # 全局地图坐标系 (通常用于 GPS/Global Fusion)
        "odom_frame": "world",           # 修改：对于月球车，我们将世界坐标系命名为 world
        "base_link_frame": "body",       # 机器人基座坐标系命名为 body
        "world_frame": "world",          # EKF 输出结果所基于的世界坐标系
    }

    # ==========================================
    # 2. 传感器配置 (Fusion Strategy)
    # ==========================================
    # 矩阵掩码顺序 (15个状态量):
    # [x, y, z, 
    #  roll, pitch, yaw, 
    #  vx, vy, vz, 
    #  vroll, vpitch, vyaw, 
    #  ax, ay, az]
    
    # ------------------------------------------
    # 传感器 1: VINS-Fusion (主要提供平滑的姿态变化和角速度)
    # ------------------------------------------
    # 问题：低速下尺度漂移严重 (位置 xyz 和 线速度 vx/vy 不可信)
    # 策略：只用 姿态 (roll, pitch, yaw) 和 角速度
    config["odom0"] = "/vins_estimator/odometry"
    config["odom0_config"] = [
        False, False, False,    # x, y, z (禁用：尺度不对)
        True,  True,  True,     # roll, pitch, yaw (使用：VINS 姿态通常很准且平滑)
        False, False, False,    # vx, vy, vz (禁用：尺度不对)
        True,  True,  True,     # vroll, vpitch, vyaw (使用：角速度)
        False, False, False     # ax, ay, az
    ]
    config["odom0_differential"] = True  # 使用增量模式 (只融合变化量，不融合绝对值，防止跳变)
    config["odom0_relative"] = False
    config["odom0_queue_size"] = 10

    # ------------------------------------------
    # 传感器 2: 轮速计 (Wheel Odometry)
    # ------------------------------------------
    # 优势：机械结构决定了尺度绝对准确 (提供可靠的 path length), 并结合星敏感器姿态提供航位推算 (Dead Reckoning)
    # 策略：
    #   1. 使用 vx, vy (Body Frame Velocity) 约束 scale
    #   2. 使用 x, y (Odom Frame Position) 约束 position drift (这是一个漂移的odom frame，但短期内尺度是准的)
    #
    # 注意：我们现在输出的是 nav_msgs/Odometry，包含 Pose 和 Twist
    config["odom1"] = "/wheel_odometry"  # 改为 odom1 (robot_localization 支持多个 odom 输入)
    config["odom1_config"] = [
        True,  True,  False,    # x, y (使用轮速计推算的位置), z (2D平面假设则不用，或者看地形)
        False, False, False,    # roll, pitch, yaw (通常轮速计的姿态不可信，除非它融合了IMU。这里我们在生成时已经融合了StarTracker，但我们还是让EKF自己去融合StarTracker的Pose话题比较稳妥。或者设为False让EKF只信位置)
                                # 修正：我们在生成时已经用了StarTracker姿态去投影位置，所以这个位置包含了高质量姿态信息。
                                # 但是，为了避免双重计数(Double Counting)星敏感器信息（因为我们有 pose0 专门订阅 star tracker），
                                # 这里的姿态最好设为 False。
        True,  True,  False,    # vx, vy (关键！修正VINS的尺度), vz
        False, False, False,
        False, False, False
    ]
    # 轮速计位置是累积的(Odometric)，本身就是连续漂移的，所以 differential=False (它是绝对的Odom坐标系)
    # 但如果为了与 VINS 对齐，且避免初始原点问题，differential=False 是标准的 (EKF 会维护 odom->base_link)
    config["odom1_differential"] = False
    config["odom1_relative"] = False
    config["odom1_queue_size"] = 10

    # 移除旧的 twist0 配置
    # config["twist0"] = ... 

    # ------------------------------------------
    # 传感器 3: 星敏感器 (Star Tracker) (提供绝对姿态基准)
    # ------------------------------------------
    # 优势：无漂移的绝对姿态
    # 策略：修正 VINS 和 陀螺仪 的长期航向漂移
    config["pose0"] = "/star_tracker/pose" # 恢复为 pose 尝试兼容，或者需要修改 dataset_saver
    config["pose0_config"] = [
        False, False, False,
        True,  True,  True,     # roll, pitch, yaw (绝对姿态约束)
        False, False, False,
        False, False, False,
        False, False, False
    ]
    config["pose0_differential"] = False # 必须为False，因为我们要利用它的"绝对"准确性来消除积累计误差
    config["pose0_relative"] = False
    config["pose0_rejection_threshold"] = 2.0 # 拒绝异常突变

    # ==========================================
    # 3. 过程噪声协方差 (Process Noise Covariance - Q)
    # ==========================================
    # 用于告诉滤波器即使没有测量，系统状态本身的不确定性也会随时间增加
    # 对角线元素: [x, y, z, r, p, y, vx, vy, vz, vr, vp, vy, ax, ay, az]
    process_noise_matrix = np.diag([
        0.05, 0.05, 0.06,     # x, y, z 误差 (位置不确定性增长)
        0.03, 0.03, 0.06,     # r, p, y 误差
        0.025, 0.025, 0.04,   # vx, vy, vz 误差
        0.01, 0.01, 0.02,     # vr, vp, vy 误差
        0.01, 0.01, 0.015     # ax, ay, az 误差
    ])
    config["process_noise_covariance"] = process_noise_matrix.flatten().tolist()

    # ==========================================
    # 4. 初始状态协方差 (Initial Estimate Covariance - P)
    # ==========================================
    # 初始时刻我们对机器人状态有多大信心
    initial_covariance_matrix = np.diag([
        1e-9, 1e-9, 1e-9,     # x, y, z (假设从原点其实，非常确定)
        1e-9, 1e-9, 1e-9,     # r, p, y
        1e-9, 1e-9, 1e-9,     # vx, vy, vz
        1e-9, 1e-9, 1e-9,     # vr, vp, vy
        1e-9, 1e-9, 1e-9      # ax, ay, az
    ])
    config["initial_estimate_covariance"] = initial_covariance_matrix.flatten().tolist()

    # ==========================================
    # 保存文件
    # ==========================================
    output_path = os.path.join("ros_config", "ekf_lunar_config.yaml")
    
    # 确保目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    print(f"Generating EKF Config at: {os.path.abspath(output_path)}")
    
    with open(output_path, 'w') as f:
        # 手动写入注释，因为 PyYAML 不支持保留注释且很难控制格式
        f.write("# EKF Configuration for Lunar Rover Low-Speed Navigation\n")
        f.write("# Generated by generate_ekf_config.py\n\n")
        
        yaml.dump(config, f, default_flow_style=None, sort_keys=False)
        
    print("Done.")

def generate_ekf_launch():
    """
    生成启动文件 .launch
    """
    launch_content = """<launch>
    <!-- EKF Localization Node -->
    <node pkg="robot_localization" type="ekf_localization_node" name="ekf_se" clear_params="true">
        <!-- 加载配置文件 -->
        <!-- 使用 $(dirname) 获取当前 launch 文件所在目录。 -->
        <!-- 只要 launch 文件和 yaml 文件在同一个文件夹下，这种方式最稳妥，无论文件夹在哪里。 -->
        <rosparam command="load" file="$(dirname)/ekf_lunar_config.yaml" />

        <!-- 输出话题重映射 -->
        <!-- EKF 默认输出 /odometry/filtered，显式重映射一下以防万一 -->
        <remap from="odometry/filtered" to="/odometry/filtered"/>
    </node>
</launch>
"""
    output_path = os.path.join("ros_config", "ekf_lunar_run.launch")
    
    print(f"Generating EKF Launch at: {os.path.abspath(output_path)}")
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(launch_content)
    print("Done.")

if __name__ == "__main__":
    generate_ekf_config()
    generate_ekf_launch()
