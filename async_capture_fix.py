"""
async_capture_fix.py — 基于性能分析的优化方案
===============================================

核心发现（profile_capture.py 结果）:
  - getImuData:          0.2ms
  - 2-cam RGB:           63ms
  - 2-cam DepthPlanar:   460ms  ← 88% 的时间都花在这里
  - simPause:            对 PhysXCar 模式无效

方案设计:
  A. 纯 IMU 基线                           → 预期 ~118Hz
  F. 2Hz RGB-only（不采深度）               → 预期 ~105Hz
  G. 2Hz RGB + 0.2Hz Depth（深度降频解耦）  → 预期 ~95Hz
  H. 2Hz RGB + 后台线程异步采深度            → 预期 ~105Hz
  C. 2Hz RGB+Depth 同步（对照组，当前方案）  → 预期 ~4.7Hz

运行方式:
  python async_capture_fix.py          # 运行所有场景
  python async_capture_fix.py F        # 只运行场景 F
  python async_capture_fix.py F G      # 运行场景 F 和 G
"""

import airsim
import numpy as np
import time
import sys
import threading
from collections import defaultdict

VEHICLE_NAME = "LunarRover"
TEST_DURATION = 30  # 每个场景运行秒数


def spin_wait(seconds):
    """精确忙等替代 time.sleep()，绕过 Windows 15.6ms 定时器粒度"""
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        pass

# ──────────────────────────────── 工具函数 ────────────────────────────────

def get_sim_time(client):
    """获取仿真时间（秒），基于 IMU 时间戳"""
    imu = client.getImuData(vehicle_name=VEHICLE_NAME)
    return imu.time_stamp / 1e9

def get_imu_sample(client):
    """获取一次 IMU 数据, 返回 (timestamp_ns, data)"""
    imu = client.getImuData(vehicle_name=VEHICLE_NAME)
    return imu.time_stamp, imu

def capture_rgb_only(client):
    """只采集双目 RGB，不采深度"""
    requests = [
        airsim.ImageRequest("obstacle_camera_left", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("obstacle_camera_right", airsim.ImageType.Scene, False, False),
    ]
    responses = client.simGetImages(requests, VEHICLE_NAME)
    return responses

def capture_depth_only(client):
    """只采集双目深度"""
    requests = [
        airsim.ImageRequest("obstacle_camera_left", airsim.ImageType.DepthPlanar, True, False),
        airsim.ImageRequest("obstacle_camera_right", airsim.ImageType.DepthPlanar, True, False),
    ]
    responses = client.simGetImages(requests, VEHICLE_NAME)
    return responses

def capture_rgb_and_depth(client):
    """采集双目 RGB + 深度（当前 lunar_rover_env 的做法：两次调用）"""
    rgb = [
        airsim.ImageRequest("obstacle_camera_left", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("obstacle_camera_right", airsim.ImageType.Scene, False, False),
    ]
    depth = [
        airsim.ImageRequest("obstacle_camera_left", airsim.ImageType.DepthPlanar, True, False),
        airsim.ImageRequest("obstacle_camera_right", airsim.ImageType.DepthPlanar, True, False),
    ]
    r1 = client.simGetImages(rgb, VEHICLE_NAME)
    r2 = client.simGetImages(depth, VEHICLE_NAME)
    return r1 + r2


# ──────────────────────────────── 统计分析 ────────────────────────────────

def analyze_results(name, imu_timestamps, image_times, wall_start, wall_end):
    """分析一个场景的采集结果"""
    wall_dur = wall_end - wall_start
    
    ts = sorted(imu_timestamps)
    n_imu = len(ts)
    
    if n_imu < 2:
        print(f"  [{name}] IMU 样本数不足: {n_imu}")
        return
    
    # IMU 频率
    sim_dur = (ts[-1] - ts[0]) / 1e9
    imu_freq = (n_imu - 1) / sim_dur if sim_dur > 0 else 0
    
    # 仿真速度比
    speed_ratio = sim_dur / wall_dur if wall_dur > 0 else 0
    
    # 间隔分析
    intervals = np.diff(ts) / 1e6  # ms
    
    # 去重（相同时间戳）
    unique_ts = len(set(ts))
    dup_rate = 1.0 - unique_ts / n_imu if n_imu > 0 else 0
    
    # 间隔分布
    normal_mask = intervals < 20  # < 20ms 视为正常（120Hz → 8.33ms）
    gap_mask = intervals >= 20
    
    n_img = len(image_times)
    img_wall_total = sum(image_times) if image_times else 0
    img_avg = (img_wall_total / n_img * 1000) if n_img > 0 else 0
    
    # 预期最大 IMU（假设 120Hz 无中断）
    max_imu = sim_dur * 120
    retention = n_imu / max_imu * 100 if max_imu > 0 else 0
    
    print(f"\n{'='*60}")
    print(f"  场景: {name}")
    print(f"{'='*60}")
    print(f"  实测时长:      {wall_dur:.1f}s (wall) / {sim_dur:.1f}s (sim)")
    print(f"  仿真速度比:    {speed_ratio:.3f}x")
    print(f"  IMU 样本数:    {n_imu} (去重: {unique_ts}, 重复率: {dup_rate:.1%})")
    print(f"  IMU 频率:      {imu_freq:.1f} Hz")
    print(f"  IMU 保留率:    {retention:.1f}% (vs 120Hz baseline)")
    print(f"  间隔中位数:    {np.median(intervals):.2f} ms")
    print(f"  间隔 P99:      {np.percentile(intervals, 99):.2f} ms")
    print(f"  正常间隔:      {normal_mask.sum()} ({normal_mask.mean():.1%})")
    print(f"  长间隔(>20ms): {gap_mask.sum()} ({gap_mask.mean():.1%})")
    if gap_mask.any():
        print(f"  长间隔最大值:  {intervals[gap_mask].max():.1f} ms")
        print(f"  长间隔平均值:  {intervals[gap_mask].mean():.1f} ms")
    print(f"  图像采集次数:  {n_img}")
    if n_img > 0:
        print(f"  图像总耗时:    {img_wall_total:.3f}s ({img_wall_total/wall_dur*100:.1f}% of wall time)")
        print(f"  图像单次耗时:  {img_avg:.1f} ms (avg)")
    print(f"{'='*60}")


# ──────────────────────────────── 场景 A: 纯 IMU ────────────────────────────

def scenario_A():
    """纯 IMU 基线 — 不采任何图像"""
    print("\n>>> 场景 A: 纯 IMU 基线")
    
    client = airsim.CarClient()
    client.confirmConnection()
    
    imu_timestamps = []
    wall_start = time.perf_counter()
    
    while time.perf_counter() - wall_start < TEST_DURATION:
        ts, _ = get_imu_sample(client)
        imu_timestamps.append(ts)
        spin_wait(0.005)  # ~200Hz 轮询（忙等，避免 Windows 15.6ms 粒度）
    
    wall_end = time.perf_counter()
    analyze_results("A_pure_imu", imu_timestamps, [], wall_start, wall_end)
    return imu_timestamps


# ──────────────────────────────── 场景 F: 2Hz RGB-only ────────────────────

def scenario_F():
    """2Hz 双目 RGB，不采深度"""
    print("\n>>> 场景 F: 2Hz RGB-only（不采深度）")
    
    client = airsim.CarClient()
    client.confirmConnection()
    
    imu_timestamps = []
    image_times = []
    last_img_wall = 0
    img_interval = 1.0 / 2  # 2Hz
    
    wall_start = time.perf_counter()
    
    while time.perf_counter() - wall_start < TEST_DURATION:
        # IMU 采样
        ts, _ = get_imu_sample(client)
        imu_timestamps.append(ts)
        
        # 图像采集（基于 wall time 间隔）
        now = time.perf_counter()
        if now - last_img_wall >= img_interval:
            t0 = time.perf_counter()
            capture_rgb_only(client)
            t1 = time.perf_counter()
            image_times.append(t1 - t0)
            last_img_wall = now
        
        spin_wait(0.005)
    
    wall_end = time.perf_counter()
    analyze_results("F_rgb_only_2hz", imu_timestamps, image_times, wall_start, wall_end)
    return imu_timestamps

# ──────────────────────────────── 场景 G: 2Hz RGB + 0.2Hz Depth ──────────

def scenario_G():
    """2Hz RGB + 0.2Hz Depth（深度降频，每 5s 采一次深度）"""
    print("\n>>> 场景 G: 2Hz RGB + 0.2Hz Depth（深度 5s 一次）")
    
    client = airsim.CarClient()
    client.confirmConnection()
    
    imu_timestamps = []
    image_times = []
    last_rgb_wall = 0
    last_depth_wall = 0
    rgb_interval = 1.0 / 2   # 2Hz
    depth_interval = 5.0      # 0.2Hz
    
    wall_start = time.perf_counter()
    
    while time.perf_counter() - wall_start < TEST_DURATION:
        ts, _ = get_imu_sample(client)
        imu_timestamps.append(ts)
        
        now = time.perf_counter()
        
        # RGB 采集
        if now - last_rgb_wall >= rgb_interval:
            t0 = time.perf_counter()
            capture_rgb_only(client)
            t1 = time.perf_counter()
            image_times.append(t1 - t0)
            last_rgb_wall = now
        
        # Depth 采集（低频）
        if now - last_depth_wall >= depth_interval:
            t0 = time.perf_counter()
            capture_depth_only(client)
            t1 = time.perf_counter()
            image_times.append(t1 - t0)
            last_depth_wall = now
        
        spin_wait(0.005)
    
    wall_end = time.perf_counter()
    analyze_results("G_rgb2hz_depth0.2hz", imu_timestamps, image_times, wall_start, wall_end)
    return imu_timestamps


# ──────────────────────────────── 场景 H: 2Hz RGB + 异步深度 ──────────────

def scenario_H():
    """2Hz RGB 在主线程，Depth 在后台线程异步采集"""
    print("\n>>> 场景 H: 2Hz RGB + 后台异步 Depth（2Hz）")
    
    # 主线程客户端
    client_main = airsim.CarClient()
    client_main.confirmConnection()
    
    # 后台线程专用客户端
    client_bg = airsim.CarClient()
    client_bg.confirmConnection()
    
    imu_timestamps = []
    image_times = []  # 只记录主线程 RGB 的耗时
    depth_count = [0]
    depth_times = []
    
    last_rgb_wall = 0
    rgb_interval = 1.0 / 2  # 2Hz
    
    # 后台深度采集线程
    stop_event = threading.Event()
    
    def depth_worker():
        """后台线程：每 0.5s 采集一次深度"""
        while not stop_event.is_set():
            t0 = time.perf_counter()
            try:
                capture_depth_only(client_bg)
                t1 = time.perf_counter()
                depth_times.append(t1 - t0)
                depth_count[0] += 1
            except Exception as e:
                print(f"  [depth_worker] error: {e}")
            # 深度采集本身很慢(~460ms)，再等一小段以达到约 2Hz
            stop_event.wait(0.05)
    
    depth_thread = threading.Thread(target=depth_worker, daemon=True)
    depth_thread.start()
    
    wall_start = time.perf_counter()
    
    while time.perf_counter() - wall_start < TEST_DURATION:
        ts, _ = get_imu_sample(client_main)
        imu_timestamps.append(ts)
        
        now = time.perf_counter()
        if now - last_rgb_wall >= rgb_interval:
            t0 = time.perf_counter()
            capture_rgb_only(client_main)
            t1 = time.perf_counter()
            image_times.append(t1 - t0)
            last_rgb_wall = now
        
        spin_wait(0.005)
    
    stop_event.set()
    depth_thread.join(timeout=3)
    
    wall_end = time.perf_counter()
    
    # 分析主线程表现
    analyze_results("H_rgb2hz_async_depth", imu_timestamps, image_times, wall_start, wall_end)
    
    # 额外打印后台深度统计
    if depth_times:
        print(f"  [后台Depth] 采集次数: {depth_count[0]}, 平均耗时: {np.mean(depth_times)*1000:.0f}ms")
        print(f"  [后台Depth] 有效频率: {depth_count[0]/(wall_end-wall_start):.2f} Hz")
    
    return imu_timestamps


# ──────────────────────────────── 场景 C: 2Hz RGB+Depth 同步 ──────────────

def scenario_C():
    """当前做法：2Hz 同步采集 RGB + Depth（对照组）"""
    print("\n>>> 场景 C: 2Hz RGB+Depth 同步（对照组）")
    
    client = airsim.CarClient()
    client.confirmConnection()
    
    imu_timestamps = []
    image_times = []
    last_img_wall = 0
    img_interval = 1.0 / 2
    
    wall_start = time.perf_counter()
    
    while time.perf_counter() - wall_start < TEST_DURATION:
        ts, _ = get_imu_sample(client)
        imu_timestamps.append(ts)
        
        now = time.perf_counter()
        if now - last_img_wall >= img_interval:
            t0 = time.perf_counter()
            capture_rgb_and_depth(client)
            t1 = time.perf_counter()
            image_times.append(t1 - t0)
            last_img_wall = now
        
        spin_wait(0.005)
    
    wall_end = time.perf_counter()
    analyze_results("C_rgb_depth_2hz_sync", imu_timestamps, image_times, wall_start, wall_end)
    return imu_timestamps


# ──────────────────────────────── 主程序 ────────────────────────────────

SCENARIOS = {
    'A': ('纯 IMU 基线', scenario_A),
    'F': ('2Hz RGB-only', scenario_F),
    'G': ('2Hz RGB + 0.2Hz Depth', scenario_G),
    'H': ('2Hz RGB + 异步 Depth', scenario_H),
    'C': ('2Hz RGB+Depth 同步(对照)', scenario_C),
}

def main():
    # 解析命令行参数
    if len(sys.argv) > 1:
        selected = [s.upper() for s in sys.argv[1:]]
        # 验证
        for s in selected:
            if s not in SCENARIOS:
                print(f"未知场景: {s}. 可选: {', '.join(SCENARIOS.keys())}")
                return
    else:
        selected = ['A', 'F', 'G', 'H', 'C']
    
    print("=" * 60)
    print("  AirSim 图像采集优化基准测试")
    print(f"  基于 profile_capture.py 性能分析结果")
    print(f"  核心发现: DepthPlanar = 460ms (88%)")
    print(f"  每场景运行 {TEST_DURATION}s")
    print("=" * 60)
    
    print(f"\n将运行以下场景:")
    for s in selected:
        name, _ = SCENARIOS[s]
        print(f"  {s}: {name}")
    
    print(f"\n请确保 AirSim 已启动且 LunarRover 在运行")
    print(f"等待 3 秒后开始...\n")
    time.sleep(3)
    
    results = {}
    for s in selected:
        name, func = SCENARIOS[s]
        try:
            results[s] = func()
        except Exception as e:
            print(f"\n!!! 场景 {s} ({name}) 失败: {e}")
            import traceback
            traceback.print_exc()
    
    # ──── 汇总对比 ────
    print("\n\n" + "=" * 70)
    print("  汇 总 对 比")
    print("=" * 70)
    print(f"{'场景':<30} {'IMU样本':>8} {'频率(Hz)':>10} {'保留率':>8}")
    print("-" * 70)
    
    for s in selected:
        if s not in results:
            continue
        ts_list = sorted(results[s])
        n = len(ts_list)
        if n < 2:
            continue
        sim_dur = (ts_list[-1] - ts_list[0]) / 1e9
        freq = (n - 1) / sim_dur if sim_dur > 0 else 0
        ret = n / (sim_dur * 120) * 100 if sim_dur > 0 else 0
        name, _ = SCENARIOS[s]
        print(f"  {s}: {name:<24} {n:>8} {freq:>10.1f} {ret:>7.1f}%")
    
    print("=" * 70)
    print("\n结论: 如果 F (RGB-only) 已接近 ~105Hz，")
    print("      而 C (RGB+Depth) 仍在 ~5Hz，")
    print("      则解决方案 = 从主循环去掉 DepthPlanar 或移到后台线程/降频。")
    print()

if __name__ == "__main__":
    main()
