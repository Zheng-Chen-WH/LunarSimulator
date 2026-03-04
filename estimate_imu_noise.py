"""
estimate_imu_noise.py — 从 EuRoC 数据集估算 IMU 噪声参数
==========================================================

功能:
  从数据集的 IMU CSV 中读取静止/低动态段数据，估算 VINS-Fusion 所需的四个噪声参数:
    acc_n  加速度计白噪声密度   (m/s²/√Hz)
    gyr_n  陀螺仪白噪声密度     (rad/s/√Hz)
    acc_w  加速度计偏置随机游走  (m/s³/√Hz)
    gyr_w  陀螺仪偏置随机游走   (rad/s²/√Hz)

原理:
  1. 白噪声密度 σ_c = σ_d / √f_s
     σ_d = 静止段离散时间标准差，f_s = 实际采样率
  2. 偏置随机游走 ≈ 长窗口均值的漂移标准差 × √(窗口时长)
     即 Allan Variance 的简化估计
"""

import csv
import os
import math
import numpy as np

# ======================== 可修改参数 ========================
DATASET_PATH = "dataset/DOF"          # 数据集路径 (包含 mav0/imu0/data.csv)
STATIC_SECONDS = 10.0                  # 使用前 N 秒数据估算白噪声
WINDOW_SECONDS = 5.0                   # 偏置随机游走估算的窗口时长
OUTPUT_YAML = None                     # 输出 YAML 路径, None 则不写文件
# ============================================================


def load_imu_data(dataset_path):
    """从 EuRoC 格式数据集加载 IMU 数据

    Args:
        dataset_path: 数据集根目录 (包含 mav0/)

    Returns:
        timestamps_ns: (N,) int64 纳秒时间戳
        gyro: (N, 3) 角速度 [wx, wy, wz] rad/s
        accel: (N, 3) 线加速度 [ax, ay, az] m/s²
    """
    imu_csv = os.path.join(dataset_path, "mav0", "imu0", "data.csv")
    if not os.path.exists(imu_csv):
        raise FileNotFoundError(f"IMU data not found: {imu_csv}")

    timestamps = []
    gyro = []
    accel = []

    with open(imu_csv, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            ts = int(row[0])
            gx, gy, gz = float(row[1]), float(row[2]), float(row[3])
            ax, ay, az = float(row[4]), float(row[5]), float(row[6])
            timestamps.append(ts)
            gyro.append([gx, gy, gz])
            accel.append([ax, ay, az])

    return np.array(timestamps, dtype=np.int64), np.array(gyro), np.array(accel)


def estimate_sampling_rate(timestamps_ns):
    """从时间戳估算实际采样率

    Args:
        timestamps_ns: (N,) 纳秒时间戳

    Returns:
        median_rate_hz: 基于中值间隔的采样率
        mean_rate_hz: 基于均值间隔的采样率
    """
    dts = np.diff(timestamps_ns) / 1e9  # 转为秒
    median_dt = np.median(dts)
    mean_dt = np.mean(dts)
    return 1.0 / median_dt, 1.0 / mean_dt


def estimate_white_noise(data, rate_hz):
    """从静止段数据估算白噪声密度

    σ_c = σ_d / √(f_s)

    Args:
        data: (N, 3) 传感器读数
        rate_hz: 采样率

    Returns:
        noise_density: 三轴均值的连续时间噪声密度
        per_axis_std: (3,) 每轴离散标准差
    """
    # 减去均值（去除重力/偏置），计算残差标准差
    residual = data - np.mean(data, axis=0)
    per_axis_std = np.std(residual, axis=0)
    mean_std = np.mean(per_axis_std)
    noise_density = mean_std / math.sqrt(rate_hz)
    return noise_density, per_axis_std


def estimate_bias_walk(data, timestamps_ns, window_seconds):
    """用滑动窗口均值法估算偏置随机游走

    将数据分为若干不重叠窗口，取各窗口均值，计算相邻窗口均值之差的标准差。
    偏置随机游走 ≈ std(Δmean) × √(f_window)
    其中 f_window = 1 / window_seconds

    Args:
        data: (N, 3) 传感器读数
        timestamps_ns: (N,) 纳秒时间戳
        window_seconds: 窗口时长（秒）

    Returns:
        bias_walk: 三轴均值的偏置随机游走
        per_axis_walk: (3,) 每轴偏置游走
    """
    total_duration = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    n_windows = max(2, int(total_duration / window_seconds))
    samples_per_window = len(data) // n_windows

    if samples_per_window < 10:
        # 数据太短，用粗略估计
        return np.nan, np.array([np.nan, np.nan, np.nan])

    window_means = []
    for i in range(n_windows):
        start = i * samples_per_window
        end = start + samples_per_window
        window_means.append(np.mean(data[start:end], axis=0))

    window_means = np.array(window_means)
    diffs = np.diff(window_means, axis=0)
    per_axis_walk = np.std(diffs, axis=0) * math.sqrt(1.0 / window_seconds)
    bias_walk = np.mean(per_axis_walk)
    return bias_walk, per_axis_walk


def estimate_from_dataset(dataset_path=DATASET_PATH, static_seconds=STATIC_SECONDS,
                          window_seconds=WINDOW_SECONDS, output_yaml=OUTPUT_YAML):
    """从数据集估算 IMU 噪声参数（主函数）

    Args:
        dataset_path: 数据集根目录
        static_seconds: 前 N 秒用于白噪声估算
        window_seconds: 偏置游走窗口时长
        output_yaml: 输出 YAML 路径, None 则不写文件

    Returns:
        dict: {acc_n, gyr_n, acc_w, gyr_w, sampling_rate_hz}
    """
    # 加载数据
    print(f"加载 IMU 数据: {dataset_path}")
    timestamps_ns, gyro, accel = load_imu_data(dataset_path)
    print(f"  总样本数: {len(timestamps_ns)}")

    total_duration = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    print(f"  总时长: {total_duration:.1f}s")

    # 采样率
    median_rate, mean_rate = estimate_sampling_rate(timestamps_ns)
    print(f"  采样率: {median_rate:.1f} Hz (中值), {mean_rate:.1f} Hz (均值)")
    rate_hz = median_rate  # 使用中值更鲁棒（不受 gap 影响）

    # 截取静止段
    static_end_ns = timestamps_ns[0] + int(static_seconds * 1e9)
    static_mask = timestamps_ns <= static_end_ns
    n_static = np.sum(static_mask)
    print(f"\n--- 白噪声估算 (前 {static_seconds}s, {n_static} 样本) ---")

    if n_static < 50:
        print(f"  警告: 静止段样本数不足 ({n_static} < 50)，结果可能不准确")

    accel_static = accel[static_mask]
    gyro_static = gyro[static_mask]

    acc_n, acc_std = estimate_white_noise(accel_static, rate_hz)
    gyr_n, gyr_std = estimate_white_noise(gyro_static, rate_hz)

    print(f"  加速度计每轴 σ_d: [{acc_std[0]:.6f}, {acc_std[1]:.6f}, {acc_std[2]:.6f}] m/s²")
    print(f"  陀螺仪每轴 σ_d:   [{gyr_std[0]:.6f}, {gyr_std[1]:.6f}, {gyr_std[2]:.6f}] rad/s")
    print(f"  acc_n = σ_d / √f_s = {np.mean(acc_std):.6f} / √{rate_hz:.0f} = {acc_n:.6f}")
    print(f"  gyr_n = σ_d / √f_s = {np.mean(gyr_std):.6f} / √{rate_hz:.0f} = {gyr_n:.6f}")

    # 偏置随机游走（用全段数据）
    print(f"\n--- 偏置随机游走估算 (全段, 窗口={window_seconds}s) ---")
    acc_w, acc_w_axis = estimate_bias_walk(accel, timestamps_ns, window_seconds)
    gyr_w, gyr_w_axis = estimate_bias_walk(gyro, timestamps_ns, window_seconds)

    if not np.isnan(acc_w):
        print(f"  加速度计每轴: [{acc_w_axis[0]:.6f}, {acc_w_axis[1]:.6f}, {acc_w_axis[2]:.6f}]")
        print(f"  陀螺仪每轴:   [{gyr_w_axis[0]:.8f}, {gyr_w_axis[1]:.8f}, {gyr_w_axis[2]:.8f}]")
        print(f"  acc_w = {acc_w:.6f}")
        print(f"  gyr_w = {gyr_w:.6f}")
    else:
        print("  数据不足，无法估算偏置游走，使用经验公式 acc_w ≈ acc_n/10")
        acc_w = acc_n / 10.0
        gyr_w = gyr_n / 10.0

    # 汇总
    print(f"\n{'='*50}")
    print(f"  VINS-Fusion 推荐噪声参数")
    print(f"{'='*50}")
    print(f"  acc_n: {acc_n:.6f}    # 加速度计白噪声密度")
    print(f"  gyr_n: {gyr_n:.6f}    # 陀螺仪白噪声密度")
    print(f"  acc_w: {acc_w:.6f}    # 加速度计偏置随机游走")
    print(f"  gyr_w: {gyr_w:.6f}    # 陀螺仪偏置随机游走")
    print(f"{'='*50}")

    # 可选: 写入 YAML
    if output_yaml:
        yaml_content = (
            f"# IMU 噪声参数 (由 estimate_imu_noise.py 自动生成)\n"
            f"# 数据集: {os.path.abspath(dataset_path)}\n"
            f"# 静止段: 前 {static_seconds}s, 窗口: {window_seconds}s\n"
            f"acc_n: {acc_n:.6f}\n"
            f"gyr_n: {gyr_n:.6f}\n"
            f"acc_w: {acc_w:.6f}\n"
            f"gyr_w: {gyr_w:.6f}\n"
        )
        with open(output_yaml, "w") as f:
            f.write(yaml_content)
        print(f"\n已写入: {output_yaml}")

    return {
        "acc_n": acc_n,
        "gyr_n": gyr_n,
        "acc_w": acc_w,
        "gyr_w": gyr_w,
        "sampling_rate_hz": rate_hz,
    }


if __name__ == "__main__":
    estimate_from_dataset()
