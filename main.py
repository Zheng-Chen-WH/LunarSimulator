"""
主运行脚本：月球车数据采集
完整流程：生成轨迹 -> 采集数据 -> 保存数据集
"""

import argparse
from lunar_rover_env import LunarRoverEnv
from dataset_saver import DatasetSaver
import config as cfg

# parser = argparse.ArgumentParser(description='月球车VINS数据集采集')
# parser.add_argument('--seed', type=int, default=42, help='随机种子')
# parser.add_argument('--trajectory_type', type=str, default='random_walk',
#                     choices=['random_walk', 'figure_eight', 'spiral', 'grid'],
#                     help='轨迹类型')
# parser.add_argument('--save_format', type=str, default='euroc',
#                     choices=['euroc', 'custom', 'rosbag'],
#                     help='数据集保存格式')
# parser.add_argument('--num_runs', type=int, default=1,
#                     help='运行次数（生成多个数据集）')
args={'seed':50, # default=42
      'trajectory_type':'grid', # 随机轨迹类型：'random_walk', 'figure_eight', 'spiral', 'grid', None(随机选择)
      'save_format':'both', # 数据集保存格式：'euroc', 'custom', 'rosbag', 'both'
      'num_runs':1, # 运行次数（生成多个数据集）
      'mode': 'manual', # 采集模式：'manual'(手动驾驶) 或 'auto'(自动导航，暂不可用)
      'duration': 60, # 手动模式：采集时长(秒)，None表示不限制
      'distance': 50, # 手动模式：采集距离(米)，None表示不限制
      'max_frames': 200 # 手动模式：最大帧数，None表示不限制
      }
env_args = {'rover_params':cfg.rover_params,
        'nav_camera_params':cfg.nav_camera_params,
        'obstacle_camera_params':cfg.obstacle_camera_params,
        'imu_params': cfg.imu_params,
        'wheel_encoder_params': cfg.wheel_encoder_params,
        'trajectory_params': cfg.trajectory_params,
        'trajectory_type': args['trajectory_type']
        }
saver_args = {
        'dataset_params':cfg.dataset_params,
        'nav_camera_params':cfg.nav_camera_params,
        'obstacle_camera_params':cfg.obstacle_camera_params,
        'imu_params': cfg.imu_params,
        'wheel_encoder_params': cfg.wheel_encoder_params
        }

print("=" * 60)
print("月球车VINS数据集采集系统")
print("=" * 60)
print(f"配置信息：")
print(f"  - 随机种子: {args['seed']}")
print(f"  - 采集模式: {args['mode']} ({'手动驾驶' if args['mode']=='manual' else '自动导航'})")
if args['mode'] == 'manual':
    print(f"  - 采集时长: {args['duration']}秒" if args['duration'] else "  - 采集时长: 无限制")
    print(f"  - 采集距离: {args['distance']}米" if args['distance'] else "  - 采集距离: 无限制")
    print(f"  - 最大帧数: {args['max_frames']}" if args['max_frames'] else "  - 最大帧数: 无限制")
else:
    print(f"  - 轨迹类型: {args['trajectory_type']}")
print(f"  - 保存格式: {args['save_format']}")
print(f"  - 运行次数: {args['num_runs']}")
print(f"  - 活动区域: {cfg.trajectory_params['area_size']}x{cfg.trajectory_params['area_size']}m")
print(f"  - 巡航速度: {cfg.trajectory_params['speed']} m/s")
print(f"  - 控制频率: {1.0/cfg.trajectory_params['dt']:.1f} Hz")
print()
print(f"传感器配置：")
print(f"  - 导航相机: {cfg.nav_camera_params['resolution']}, FOV={cfg.nav_camera_params['fov']}°, {cfg.nav_camera_params['fps']} fps")
print(f"  - 避障相机: {cfg.obstacle_camera_params['resolution']}, FOV={cfg.obstacle_camera_params['fov']}°, {cfg.obstacle_camera_params['fps']} fps")
print(f"  - IMU采样率: {cfg.imu_params['sampling_rate']} Hz")
print(f"  - 轮速计采样率: {cfg.wheel_encoder_params['sampling_rate']} Hz")
print("=" * 60)
print()

# 初始化环境
print("初始化AirSim连接...")
env = LunarRoverEnv(env_args)

# 初始化数据保存器
saver = DatasetSaver(saver_args)

# 运行多次采集
for run_idx in range(args['num_runs']):
    print(f"\n{'=' * 60}")
    print(f"开始第 {run_idx + 1}/{args['num_runs']} 次数据采集")
    print(f"{'=' * 60}\n")
    
    # 重置环境
    seed = args['seed'] + run_idx if args['seed'] is not None else None

    env.reset(seed=seed)
    
    if args['mode'] == 'manual':
        print(f"\n=== 手动驾驶模式 ===")
        print(f"请使用键盘或手柄控制车辆：")
        print(f"  - 方向键/WASD：前进、后退、转向")
        print(f"  - 或使用手柄控制")
        print(f"\n程序将自动采集传感器数据，满足以下任一条件后停止：")
        if args['duration']:
            print(f"  ✓ 时长达到 {args['duration']} 秒")
        if args['distance']:
            print(f"  ✓ 距离达到 {args['distance']} 米")
        if args['max_frames']:
            print(f"  ✓ 帧数达到 {args['max_frames']} 帧")
        print()
        input("按回车键开始采集...")
        
        # 手动驾驶数据采集
        dataset = env.collect_data(
            duration=args['duration'],
            distance=args['distance'],
            max_frames=args['max_frames']
        )
    else:
        # 自动导航模式（保留，暂不可用）
        print(f"自动导航模式暂不可用，请使用手动模式")
        continue
    
    # 统计采集的数据
    try:
        n_frames = len(dataset['data'])
        n_imu = sum(1 for d in dataset['data'] if 'imu' in d)
        n_camera = sum(1 for d in dataset['data'] if 'nav_camera' in d)
        n_encoder = sum(1 for d in dataset['data'] if 'wheel_encoder' in d)
        
        print(f"\n数据采集统计：")
        print(f"  - 总帧数: {n_frames}")
        print(f"  - IMU数据: {n_imu} 帧")
        print(f"  - 相机数据: {n_camera} 帧")
        print(f"  - 轮速数据: {n_encoder} 帧")
        
        # 保存数据集
        print(f"\n保存数据集...")

        saver.save_dataset(dataset, format_type=args['save_format'])
        
        print(f"\n✓ 第 {run_idx + 1} 次采集完成")
        
    except KeyboardInterrupt:
        print("\n用户中断采集")
        break
    except Exception as e:
        print(f"\n错误：数据采集失败")
        print(f"异常信息: {str(e)}")
        import traceback
        traceback.print_exc()
        continue

print(f"\n{'=' * 60}")
print(f"所有数据采集完成！")
print(f"数据集保存在: {cfg.dataset_params['save_path']}")
print(f"{'=' * 60}\n")
