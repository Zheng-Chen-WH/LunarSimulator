import json
import numpy as np
import os
import math

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
                
        width = cap_settings.get("Width", 840)
        height = cap_settings.get("Height", 840)
        fov_deg = cap_settings.get("FOV_Degrees", 90)
        
        fx = (width / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        fy = fx 
        cx = width / 2.0
        cy = height / 2.0

        cam_filename = f"cam{cam['id']}.yaml"
        with open(cam_filename, 'w') as f:
            f.write(f"%YAML:1.0\n---\n")
            f.write("model_type: PINHOLE\n")
            f.write(f"camera_name: camera\n")
            f.write(f"image_width: {width}\n")
            f.write(f"image_height: {height}\n")
            f.write("distortion_parameters:\n")
            f.write("   k1: 0.0\n   k2: 0.0\n   p1: 0.0\n   p2: 0.0\n")
            f.write("projection_parameters:\n")
            f.write(f"   fx: {fx:.5f}\n")
            f.write(f"   fy: {fy:.5f}\n")
            f.write(f"   cx: {cx:.1f}\n")
            f.write(f"   cy: {cy:.1f}\n")
        
        print(f"Generated {cam_filename}")

        # Extrinsics
        c_x = cam_data.get("X", 0)
        c_y = cam_data.get("Y", 0)
        c_z = cam_data.get("Z", 0)
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
                 w = cap_s.get("Width", 840)
                 h = cap_s.get("Height", 840)
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
            f.write("keyframe_parallax: 3.0\n\n")  # 降低关键帧视差阈值，适应极低速运动
            f.write("acc_n: 0.004\n")
            f.write("gyr_n: 0.002\n")
            f.write("acc_w: 0.0004\n")
            f.write("gyr_w: 0.0002\n")
            f.write("g_norm: 9.81\n\n")
            f.write("estimate_td: 1\n")
            f.write("td: 0.0\n")
        
        print(f"Generated {vio_filename}")
    else:
        print("Could not generate VIO.yaml.")

if __name__ == "__main__":
    main()