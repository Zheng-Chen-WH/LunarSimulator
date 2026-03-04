"""验证相机外参一致性: settings.json vs sensor.yaml vs generate_vins_config.py"""
import json
import math
import numpy as np
import yaml

# 1. 读取 settings.json
with open(r'C:\Users\zchenkf\Documents\AirSim\settings.json', 'r') as f:
    settings = json.load(f)

vehicle = settings['Vehicles']['LunarRover']
cameras = vehicle['Cameras']

print("=" * 70)
print("1. settings.json 原始值")
print("=" * 70)
cam_names = ['nav_camera_left', 'nav_camera_right', 'obstacle_camera_left', 'obstacle_camera_right']
for name in cam_names:
    c = cameras[name]
    cs = c['CaptureSettings'][0]
    print(f"  {name}:")
    print(f"    Position: X={c['X']}, Y={c['Y']}, Z={c['Z']}")
    print(f"    Rotation: Roll={c.get('Roll',0)}, Pitch={c.get('Pitch',0)}, Yaw={c.get('Yaw',0)}")
    print(f"    FOV: {cs['FOV_Degrees']}°, Resolution: {cs['Width']}x{cs['Height']}")

print()
print("=" * 70)
print("2. settings.json / 20 (期望的真实物理尺寸)")
print("=" * 70)
for name in cam_names:
    c = cameras[name]
    print(f"  {name}: X={c['X']/20:.4f}, Y={c['Y']/20:.4f}, Z={c['Z']/20:.4f}")

# 2. 读取 sensor.yaml
print()
print("=" * 70)
print("3. sensor.yaml 实际值 (dataset DOF)")
print("=" * 70)
with open(r'd:\Codes\LunarSimulator\dataset\DOF\mav0\sensor.yaml', 'r') as f:
    sensor_data = yaml.safe_load(f)

for cam_id in ['cam0', 'cam1', 'cam2', 'cam3']:
    T = sensor_data[cam_id]['T_cam_imu']
    T_mat = np.array(T).reshape(4, 4)
    trans = T_mat[:3, 3]
    intrinsics = sensor_data[cam_id]['intrinsics']
    resolution = sensor_data[cam_id]['resolution']
    print(f"  {cam_id}:")
    print(f"    Translation: X={trans[0]:.4f}, Y={trans[1]:.4f}, Z={trans[2]:.4f}")
    print(f"    Rotation matrix:\n      {T_mat[:3,:3].tolist()}")
    print(f"    Intrinsics: fx={intrinsics[0]:.2f}, fy={intrinsics[1]:.2f}, cx={intrinsics[2]:.1f}, cy={intrinsics[3]:.1f}")
    print(f"    Resolution: {resolution}")

# 3. 对比分析
print()
print("=" * 70)
print("4. 对比分析: 外参平移 (T)")
print("=" * 70)
mapping = {
    'cam0': 'nav_camera_left',
    'cam1': 'nav_camera_right',
    'cam2': 'obstacle_camera_left',
    'cam3': 'obstacle_camera_right',
}

for cam_id, settings_name in mapping.items():
    c = cameras[settings_name]
    expected = [c['X']/20, c['Y']/20, c['Z']/20]
    T = np.array(sensor_data[cam_id]['T_cam_imu']).reshape(4, 4)
    actual = T[:3, 3].tolist()
    
    match = all(abs(e - a) < 0.001 for e, a in zip(expected, actual))
    status = "✓" if match else "✗ 不匹配!"
    
    print(f"  {cam_id} ({settings_name}):")
    print(f"    期望 (settings/20): [{expected[0]:.4f}, {expected[1]:.4f}, {expected[2]:.4f}]")
    print(f"    实际 (sensor.yaml): [{actual[0]:.4f}, {actual[1]:.4f}, {actual[2]:.4f}]")
    if not match:
        diff = [a - e for a, e in zip(actual, expected)]
        print(f"    差值: [{diff[0]:.4f}, {diff[1]:.4f}, {diff[2]:.4f}]")
    print(f"    结果: {status}")

# 4. FOV 对比
print()
print("=" * 70)
print("5. 对比分析: FOV 与内参")
print("=" * 70)

from config import nav_camera_params, obstacle_camera_params

for cam_id, settings_name in mapping.items():
    cs = cameras[settings_name]['CaptureSettings'][0]
    fov_settings = cs['FOV_Degrees']
    w_settings = cs['Width']
    h_settings = cs['Height']
    
    if 'nav' in settings_name:
        fov_config = nav_camera_params['fov']
        res_config = nav_camera_params['resolution']
    else:
        fov_config = obstacle_camera_params['fov']
        res_config = obstacle_camera_params['resolution']
    
    fx_from_settings = (w_settings / 2.0) / math.tan(math.radians(fov_settings / 2.0))
    
    actual_intrinsics = sensor_data[cam_id]['intrinsics']
    actual_res = sensor_data[cam_id]['resolution']
    
    print(f"  {cam_id} ({settings_name}):")
    print(f"    settings.json: FOV={fov_settings}°, Res={w_settings}x{h_settings}, fx={fx_from_settings:.2f}")
    print(f"    config.py:     FOV={fov_config}°, Res={res_config}")
    print(f"    sensor.yaml:   fx={actual_intrinsics[0]:.2f}, Res={actual_res}")
    
    if abs(fov_settings - fov_config) > 0.1:
        print(f"    ✗ FOV不匹配! settings={fov_settings}° vs config={fov_config}°")

# 5. 检查 generate_vins_config.py 输出
print()
print("=" * 70)
print("6. generate_vins_config.py 预期输出 (body_T_cam)")
print("=" * 70)

def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

R_airsim_optical = np.array([[0,0,1],[1,0,0],[0,1,0]])
T_airsim_optical = np.eye(4)
T_airsim_optical[:3,:3] = R_airsim_optical

for settings_name in ['obstacle_camera_left', 'obstacle_camera_right']:
    c = cameras[settings_name]
    x, y, z = c['X']/20, c['Y']/20, c['Z']/20
    roll, pitch, yaw = c.get('Roll',0), c.get('Pitch',0), c.get('Yaw',0)
    
    T_v_cambody = np.eye(4)
    T_v_cambody[:3,:3] = euler_to_rotation_matrix(roll, pitch, yaw)
    T_v_cambody[:3, 3] = [x, y, z]
    
    # IMU at origin → T_imu_v = I
    T_imu_optical = T_v_cambody @ T_airsim_optical
    
    print(f"  {settings_name} (VINS body_T_cam):")
    print(f"    Translation: [{T_imu_optical[0,3]:.4f}, {T_imu_optical[1,3]:.4f}, {T_imu_optical[2,3]:.4f}]")
    print(f"    Rotation:")
    for row in T_imu_optical[:3,:3]:
        print(f"      [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}]")

# 6. dataset_saver.py 问题总结
print()
print("=" * 70)
print("7. dataset_saver.py 问题总结")  
print("=" * 70)
print("  [BUG 1] 右相机 Y 坐标双重计算:")
print("    代码: y = settings_Y / 20 + config_baseline")
print("    settings.json 已为左右相机分别设置了完整的 Y 坐标")
print("    除以 20 后就是正确值，不应再叠加 config.py 的 baseline")
print(f"    cam1: 0.06/20=0.003, 但实际输出 0.003+0.12=0.123")
print(f"    cam3: 1.0/20=0.05, 但实际输出 0.05+0.10=0.15")
print()
print("  [BUG 2] 避障相机 FOV 不匹配:")
print(f"    config.py: obstacle fov=70°")
print(f"    settings.json: obstacle FOV_Degrees=60°")
print("    sensor.yaml 内参基于 config.py 的 70° 计算 → 焦距错误")
print()
print("  [BUG 3] 缺少 AirSim→光学坐标系旋转:")
print("    dataset_saver.py 直接用 Euler 角构造旋转矩阵 (NED body frame)")
print("    未转换为光学坐标系 (X-right, Y-down, Z-forward)")
print("    generate_vins_config.py 正确应用了 T_airsim_optical 转换")
