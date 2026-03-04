import json
import numpy as np
import os
import math

# ======================== 可修改参数 ========================
# 数据集路径: 设为有效路径则自动估算 IMU 噪声，设为 None 则使用下方默认值
DATASET_PATH = "dataset/DOF"

# IMU 噪声默认值 (仅当 DATASET_PATH=None 或估算失败时使用)
DEFAULT_IMU_NOISE = {
    "acc_n": 0.03,
    "gyr_n": 0.005,
    "acc_w": 0.003,
    "gyr_w": 0.0005,
}
# ============================================================

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles (in degrees, AirSim convention) to Rotation Matrix.
    AirSim generally uses Z-Y-X rotation sequence (Yaw, Pitch, Roll) or similar math for NED.
    Here we build standard Rz * Ry * Rx
    """
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(r), -math.sin(r)],
        [0, math.sin(r), math.cos(r)]
    ])

    Ry = np.array([
        [math.cos(p), 0, math.sin(p)],
        [0, 1, 0],
        [-math.sin(p), 0, math.cos(p)]
    ])

    Rz = np.array([
        [math.cos(y), -math.sin(y), 0],
        [math.sin(y), math.cos(y), 0],
        [0, 0, 1]
    ])

    # R = Rz * Ry * Rx
    return Rz @ Ry @ Rx

def get_transform_matrix(x, y, z, roll, pitch, yaw):
    """
    Construct 4x4 Homogeneous Transformation Matrix from AirSim position/orientation.
    """
    T = np.eye(4)
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

def inverse_transform(T):
    """
    Calculate inverse of a rigid body transform.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv

def main():
    # 从数据集自动估算 IMU 噪声
    imu_noise_params = None
    if DATASET_PATH and os.path.exists(os.path.join(DATASET_PATH, "mav0", "imu0", "data.csv")):
        print(f"\n从数据集估算 IMU 噪声参数: {DATASET_PATH}")
        try:
            from estimate_imu_noise import estimate_from_dataset
            result = estimate_from_dataset(DATASET_PATH)
            imu_noise_params = {k: result[k] for k in ("acc_n", "gyr_n", "acc_w", "gyr_w")}
        except Exception as e:
            print(f"  估算失败 ({e})，使用默认值")
    else:
        if DATASET_PATH:
            print(f"数据集路径无效: {DATASET_PATH}，使用默认噪声参数")
        else:
            print("未指定数据集路径，使用默认噪声参数")

    # Paths - Prioritize the user's specific path
    target_path = r"c:\Users\zchenkf\Documents\AirSim\settings.json"
    local_path = os.path.join("Documents", "settings.json")
    
    # Check if target_path is accessible
    if os.path.exists(target_path):
        settings_path = target_path
    elif os.path.exists(local_path):
        settings_path = local_path
    else:
        # If neither exists (maybe permissions issue with absolute path), try reading local with a warning
        print(f"Warning: {target_path} not found. Trying local.")
        settings_path = local_path 

    print(f"Reading settings from: {settings_path}")
    
    try:
        with open(settings_path, 'r', encoding='utf-8') as f:
            settings_str = f.read()
            # Handle comments in JSON
            lines = settings_str.split('\n')
            clean_lines = [l for l in lines if not l.strip().startswith("//")]
            clean_json = '\n'.join(clean_lines)
            settings = json.loads(clean_json)
    except Exception as e:
        print(f"Error reading/parsing settings: {e}")
        return

    # Find LunarRover
    vehicle_settings = settings.get("Vehicles", {}).get("LunarRover", {})
    if not vehicle_settings:
        first_key = next(iter(settings.get("Vehicles", {})), None)
        if first_key:
            vehicle_settings = settings["Vehicles"][first_key]
            print(f"LunarRover not found, using vehicle: {first_key}")
        else:
            print("No vehicles found in settings.")
            return

    # 1. Get IMU Transform (Body Frame)
    imu_settings = vehicle_settings.get("Sensors", {}).get("imu", {})
    imu_x = imu_settings.get("X", 0)
    imu_y = imu_settings.get("Y", 0)
    imu_z = imu_settings.get("Z", 0)
    imu_roll = imu_settings.get("Roll", 0)
    imu_pitch = imu_settings.get("Pitch", 0)
    imu_yaw = imu_settings.get("Yaw", 0)
    
    T_v_imu = get_transform_matrix(imu_x, imu_y, imu_z, imu_roll, imu_pitch, imu_yaw)
    T_imu_v = inverse_transform(T_v_imu)

    # 2. Get Camera Transforms
    cameras = vehicle_settings.get("Cameras", {})
    
    left_cam_candidates = ["obstacle_left", "obstacle_camera_left", "haz_left", "cam2"]
    right_cam_candidates = ["obstacle_right", "obstacle_camera_right", "haz_right", "cam3"]
    
    found_left = None
    for name in left_cam_candidates:
        if name in cameras:
            found_left = name
            break
            
    found_right = None
    for name in right_cam_candidates:
        if name in cameras:
            found_right = name
            break
            
    cam_configs = []
    if found_left:
        cam_configs.append({"name": found_left, "id": 0})
    else:
        print("Warning: Could not find left camera")

    if found_right:
        cam_configs.append({"name": found_right, "id": 1})
    else:
        print("Warning: Could not find right camera")

    # Transform: AirSim Camera Frame -> Standard Optical Frame
    R_airsim_optical = np.array([
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0]
    ])
    T_airsim_optical = np.eye(4)
    T_airsim_optical[:3, :3] = R_airsim_optical

    vio_body_transforms = {}
    
    for cam in cam_configs:
        cam_name = cam["name"]
        cam_data = cameras[cam_name]
        
        # Intrinsics
        cap_settings = cam_data.get("CaptureSettings", [{}])
        if isinstance(cap_settings, list):
             if len(cap_settings) > 0:
                cap_settings = cap_settings[0]
             else:
                cap_settings = {}
                
        raw_width = cap_settings.get("Width", 840)
        raw_height = cap_settings.get("Height", 840)
        fov_deg = cap_settings.get("FOV_Degrees", 90)
        
        # 计算裁剪后的分辨率（避障相机从正方形裁剪为目标FOV）
        # 从 config.py 读取裁剪目标
        try:
            from config import obstacle_camera_params
            target_fov_h = obstacle_camera_params.get('target_fov_h', fov_deg)
            target_fov_v = obstacle_camera_params.get('target_fov_v', fov_deg)
            # 使用三角函数精确计算裁剪像素数: pixels = 2 * fx * tan(target_fov/2)
            _fx = (raw_width / 2.0) / math.tan(math.radians(fov_deg / 2.0))
            out_width = round(2 * _fx * math.tan(math.radians(target_fov_h / 2.0))) if target_fov_h < fov_deg else raw_width
            out_height = round(2 * _fx * math.tan(math.radians(target_fov_v / 2.0))) if target_fov_v < fov_deg else raw_height
        except ImportError:
            out_width, out_height = raw_width, raw_height
        
        # 焦距由采集FOV和采集宽度决定（裁剪不改变焦距）
        fx = (raw_width / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        fy = fx 
        cx = out_width / 2.0
        cy = out_height / 2.0

        cam_filename = f"cam{cam['id']}.yaml"
        with open(cam_filename, 'w') as f:
            f.write(f"%YAML:1.0\n---\n")
            f.write("model_type: PINHOLE\n")
            f.write(f"camera_name: camera\n")
            f.write(f"image_width: {out_width}\n")
            f.write(f"image_height: {out_height}\n")
            f.write("distortion_parameters:\n")
            f.write("   k1: 0.0\n   k2: 0.0\n   p1: 0.0\n   p2: 0.0\n")
            f.write("projection_parameters:\n")
            f.write(f"   fx: {fx:.5f}\n")
            f.write(f"   fy: {fy:.5f}\n")
            f.write(f"   cx: {cx:.1f}\n")
            f.write(f"   cy: {cy:.1f}\n")
        
        print(f"Generated {cam_filename}")

        # Extrinsics (除以20转换为实际物理尺寸)
        c_x = cam_data.get("X", 0) / 20.0
        c_y = cam_data.get("Y", 0) / 20.0
        c_z = cam_data.get("Z", 0) / 20.0
        c_r = cam_data.get("Roll", 0)
        c_p = cam_data.get("Pitch", 0)
        c_y_aw = cam_data.get("Yaw", 0)

        T_v_cambody = get_transform_matrix(c_x, c_y, c_z, c_r, c_p, c_y_aw)
        T_v_optical = T_v_cambody @ T_airsim_optical
        T_imu_optical = T_imu_v @ T_v_optical
        vio_body_transforms[cam['id']] = T_imu_optical

    if 0 in vio_body_transforms and 1 in vio_body_transforms:
        vio_filename = "VIO.yaml"
        with open(vio_filename, 'w') as f:
            f.write("%YAML:1.0\n\n")
            f.write("#common parameters\n")
            f.write("imu: 1\n")
            f.write("num_of_cam: 2\n\n")
            f.write('imu_topic: "/imu0"\n')
            f.write('image0_topic: "/cam0/image_raw"\n')
            f.write('image1_topic: "/cam1/image_raw"\n')
            f.write('output_path: "~/output/"\n\n')
            f.write('cam0_calib: "cam0.yaml"\n')
            f.write('cam1_calib: "cam1.yaml"\n')
            
            if found_left:
                 left_cam_data = cameras[found_left]
                 cap_s = left_cam_data.get("CaptureSettings", [{}])
                 if isinstance(cap_s, list) and len(cap_s) > 0: cap_s = cap_s[0]
                 elif isinstance(cap_s, list): cap_s = {}
                 raw_w = cap_s.get("Width", 840)
                 raw_h = cap_s.get("Height", 840)
                 raw_fov = cap_s.get("FOV_Degrees", 90)
                 # 使用裁剪后的分辨率
                 try:
                     from config import obstacle_camera_params as _obs_params
                     _tfh = _obs_params.get('target_fov_h', raw_fov)
                     _tfv = _obs_params.get('target_fov_v', raw_fov)
                     _fx = (raw_w / 2.0) / math.tan(math.radians(raw_fov / 2.0))
                     w = round(2 * _fx * math.tan(math.radians(_tfh / 2.0))) if _tfh < raw_fov else raw_w
                     h = round(2 * _fx * math.tan(math.radians(_tfv / 2.0))) if _tfv < raw_fov else raw_h
                 except ImportError:
                     w, h = raw_w, raw_h
                 f.write(f"image_width: {w}\n")
                 f.write(f"image_height: {h}\n\n")

            f.write("# Extrinsics\n")
            f.write("estimate_extrinsic: 1   # 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.\n")
            f.write("                    # 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.\n\n")

            for cam_id in [0, 1]:
                T = vio_body_transforms[cam_id]
                f.write(f"body_T_cam{cam_id}: !!opencv-matrix\n")
                f.write("   rows: 4\n   cols: 4\n   dt: d\n")
                f.write("   data: [")
                flat_data = T.flatten()
                for i, val in enumerate(flat_data):
                    f.write(f"{val:.10f}")
                    if i < 15:
                        f.write(", ")
                        if (i + 1) % 4 == 0:
                            f.write("\n          ")
                f.write("]\n\n")

            # IMU 噪声参数：优先从数据集估算，否则使用默认值
            imu_noise = dict(DEFAULT_IMU_NOISE)
            if imu_noise_params is not None:
                imu_noise.update(imu_noise_params)

            f.write("# Optimization & IMU Defaults\n")
            f.write("multiple_thread: 1\n")
            f.write("max_cnt: 150\n")
            f.write("min_dist: 30\n")
            f.write("freq: 10\n")
            f.write("F_threshold: 1.0\n")
            f.write("show_track: 1\n")
            f.write("flow_back: 1\n\n")
            f.write("max_solver_time: 0.04\n")
            f.write("max_num_iterations: 8\n")
            f.write("keyframe_parallax: 3.0\n\n")
            f.write(f"acc_n: {imu_noise['acc_n']:.6f}\n")
            f.write(f"gyr_n: {imu_noise['gyr_n']:.6f}\n")
            f.write(f"acc_w: {imu_noise['acc_w']:.6f}\n")
            f.write(f"gyr_w: {imu_noise['gyr_w']:.6f}\n")
            f.write("g_norm: 9.81\n\n")
            f.write("estimate_td: 0\n")
            f.write("td: 0.0\n")
        
        print(f"Generated {vio_filename}")
    else:
        print("Could not generate VIO.yaml.")

if __name__ == "__main__":
    main()