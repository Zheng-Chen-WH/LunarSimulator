"""
数据集时间戳验证脚本
检测大幅超出均值的IMU间隔，将其时间戳与图像时间戳对比来判断是否图像保存导致堵塞

用法：
    修改下面的 DATASET_PATH 变量为你的数据集路径，然后直接运行
"""

# ============================================
# 配置参数（修改这里）
# ============================================

# 数据集路径（包含 mav0 目录的文件夹路径）
DATASET_PATH = r"./dataset/20260302_192827"

# 输出目录（None表示当前目录）
OUTPUT_DIR = r"./verification_output"

# 异常间隔判定阈值（n倍sigma，工业界常用3-sigma准则）
N_SIGMA = 3.0

# 时间对齐容差（判断图像是否在IMU间隔附近，单位：秒）
TIME_ALIGNMENT_TOLERANCE = 0.01  # 10ms

# 参考间隔（ms），用于在图中标记参考线
# 设为 None 时自动根据数据集实际频率计算（推荐）
# 手动指定时使用固定值，如 8.333 (120Hz), 16.667 (60Hz), 33.333 (30Hz)
REFERENCE_INTERVAL_MS = None

# 绘图时间范围（秒），设为None绘制全部，例如(50, 100)只绘制50-100秒
PLOT_TIME_RANGE =(30,40)  # (50, 100)

# ============================================
# 以下为脚本代码，一般不需要修改
# ============================================

import numpy as np
import pandas as pd
from pathlib import Path
from collections import defaultdict
import matplotlib.pyplot as plt


class DatasetVerifier:
    def __init__(self, dataset_path, output_dir):
        self.dataset_path = Path(dataset_path)
        self.mav0_path = self.dataset_path / "mav0"
        
        if output_dir:
            self.output_dir = Path(output_dir)
        else:
            self.output_dir = Path(".")
        
        if not self.mav0_path.exists():
            raise ValueError(f"找不到 mav0 目录: {self.mav0_path}")
        
        self.imu_data = None
        self.cam_data = {}
        self.gt_data = None
        
    def load_groundtruth_data(self):
        """加载真值数据"""
        gt_file = self.mav0_path / "state_groundtruth_estimate0" / "data.csv"
        if not gt_file.exists():
            print(f"警告: 找不到真值数据文件: {gt_file}")
            return None
        
        df = pd.read_csv(gt_file, comment='#')
        df.columns = ['timestamp', 'p_x', 'p_y', 'p_z', 'q_w', 'q_x', 'q_y', 'q_z',
                      'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z']
        df['timestamp_sec'] = df['timestamp'] / 1e9
        
        self.gt_data = df
        print(f"[真值] 加载了 {len(df)} 条记录")
        return df
    
    def integrate_velocity_from_groundtruth(self):
        """
        使用真值速度和时间戳进行积分，验证位置估计精度
        这是为了验证时间戳准确性（而非IMU精度）
        """
        if self.gt_data is None:
            print("[速度积分验证] 真值数据不可用")
            return None
        
        print("\n" + "="*70)
        print("速度积分验证（基于真值速度，验证时间戳准确性）")
        print("="*70)
        
        ts = self.gt_data['timestamp_sec'].values
        vel = self.gt_data[['v_x', 'v_y', 'v_z']].values
        pos_gt = self.gt_data[['p_x', 'p_y', 'p_z']].values
        
        # 数值积分（梯形法）
        n = len(ts)
        pos_int = np.zeros((n, 3))
        pos_int[0] = pos_gt[0]  # 使用真值初始位置
        
        for i in range(1, n):
            dt = ts[i] - ts[i-1]
            # 梯形积分: (v_i-1 + v_i) / 2 * dt
            avg_vel = (vel[i-1] + vel[i]) / 2
            pos_int[i] = pos_int[i-1] + avg_vel * dt
        
        # 计算误差
        error = pos_int - pos_gt
        error_norm = np.linalg.norm(error, axis=1)
        
        # 统计
        print(f"\n积分方法: 梯形法（速度平均）")
        print(f"数据点数: {n}")
        print(f"时间跨度: {ts[-1] - ts[0]:.3f}s")
        
        print(f"\n位置误差统计:")
        print(f"   均值: {np.mean(error_norm):.4f}m")
        print(f"   标准差: {np.std(error_norm):.4f}m")
        print(f"   最大值: {np.max(error_norm):.4f}m")
        print(f"   最小值: {np.min(error_norm):.4f}m")
        
        # 各轴误差
        for i, axis in enumerate(['X', 'Y', 'Z']):
            axis_error = np.abs(error[:, i])
            print(f"   {axis}轴误差: 均值={np.mean(axis_error):.4f}m, 最大={np.max(axis_error):.4f}m")
        
        # 误差分析
        print(f"\n误差分析:")
        if np.max(error_norm) < 0.01:
            print(f"   ✓ 时间戳非常准确（最大误差 < 1cm）")
        elif np.max(error_norm) < 0.1:
            print(f"   ○ 时间戳较准确（最大误差 < 10cm）")
        elif np.max(error_norm) < 1.0:
            print(f"   △ 时间戳有一定偏差（最大误差 < 1m）")
        else:
            print(f"   ✗ 时间戳可能存在较大问题（最大误差 > 1m）")
        
        # 如果误差很大，可能是时间戳不连续导致的
        max_error_idx = np.argmax(error_norm)
        max_error_time = ts[max_error_idx]
        print(f"\n最大误差出现在 t={max_error_time:.3f}s (idx={max_error_idx})")
        
        result = {
            'timestamps': ts,
            'error': error,
            'error_norm': error_norm,
            'mean_error': np.mean(error_norm),
            'max_error': np.max(error_norm)
        }
        
        # 生成图表
        self.plot_velocity_integration(result)
        
        return result
    
    def plot_velocity_integration(self, result):
        """绘制速度积分误差随时间变化的折线图"""
        if result is None:
            return
        
        fig, axes = plt.subplots(2, 1, figsize=(14, 8))
        
        ts = result['timestamps']
        error = result['error']
        error_norm = result['error_norm']
        
        # 上图: 总误差
        ax = axes[0]
        ax.plot(ts, error_norm * 1000, 'b-', linewidth=0.8, alpha=0.7, label='Position Error')
        ax.axhline(y=result['mean_error'] * 1000, color='g', linestyle='--', 
                  label=f"Mean: {result['mean_error']*1000:.2f}mm")
        ax.axhline(y=10, color='orange', linestyle=':', alpha=0.7, label='10mm threshold')
        ax.axhline(y=100, color='r', linestyle=':', alpha=0.7, label='100mm threshold')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (mm)')
        ax.set_title('Velocity Integration Error Over Time')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # 添加统计文本
        stats_text = f"Max: {result['max_error']*1000:.2f}mm | Mean: {result['mean_error']*1000:.2f}mm"
        ax.text(0.02, 0.95, stats_text, transform=ax.transAxes,
               verticalalignment='top', horizontalalignment='left',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
               fontsize=9, family='monospace')
        
        # 下图: 各轴误差
        ax = axes[1]
        colors = ['r', 'g', 'b']
        labels = ['X', 'Y', 'Z']
        for i in range(3):
            ax.plot(ts, error[:, i] * 1000, color=colors[i], linewidth=0.6, 
                   alpha=0.7, label=f'{labels[i]} axis')
        
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error per Axis (mm)')
        ax.set_title('Velocity Integration Error by Axis')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_path = self.output_dir / "velocity_integration_error.png"
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\n速度积分误差图已保存: {output_path}")
        plt.close()
    
    def load_imu_data(self):
        imu_file = self.mav0_path / "imu0" / "data.csv"
        if not imu_file.exists():
            print(f"警告: 找不到IMU数据文件: {imu_file}")
            return None
        
        df = pd.read_csv(imu_file, comment='#')
        df.columns = ['timestamp', 'w_x', 'w_y', 'w_z', 'a_x', 'a_y', 'a_z']
        df['timestamp_sec'] = df['timestamp'] / 1e9
        
        self.imu_data = df
        print(f"[IMU] 加载了 {len(df)} 条记录，时间范围: {df['timestamp_sec'].iloc[0]:.3f}s - {df['timestamp_sec'].iloc[-1]:.3f}s")
        return df
    
    def load_camera_data(self, cam_id):
        cam_file = self.mav0_path / f"cam{cam_id}" / "data.csv"
        if not cam_file.exists():
            print(f"警告: 找不到相机 {cam_id} 数据文件: {cam_file}")
            return None
        
        df = pd.read_csv(cam_file, comment='#')
        df.columns = ['timestamp', 'filename']
        df['timestamp_sec'] = df['timestamp'] / 1e9
        
        self.cam_data[cam_id] = df
        print(f"[相机 cam{cam_id}] 加载了 {len(df)} 条记录")
        return df
    
    def find_anomalous_imu_intervals(self, df, n_sigma):
        timestamps = df['timestamp_sec'].values
        dt = np.diff(timestamps)
        
        dt_mean = np.mean(dt)
        dt_std = np.std(dt)
        threshold = dt_mean + n_sigma * dt_std
        
        anomalies = []
        for i, interval in enumerate(dt):
            if interval > threshold:
                anomalies.append({
                    'idx': i,
                    'start_time': timestamps[i],
                    'end_time': timestamps[i + 1],
                    'duration': interval,
                    'excess': interval - dt_mean,
                    'n_sigma': (interval - dt_mean) / dt_std if dt_std > 0 else 0
                })
        
        return anomalies, dt_mean, dt_std, threshold, dt
    
    def check_camera_alignment(self, imu_anomalies, tolerance):
        alignment_results = []
        
        for anomaly in imu_anomalies:
            start_time = anomaly['start_time']
            end_time = anomaly['end_time']
            
            window_start = start_time - tolerance
            window_end = end_time + tolerance
            
            matched_cameras = defaultdict(list)
            
            for cam_id, cam_df in self.cam_data.items():
                if cam_df is None:
                    continue
                
                cam_ts = cam_df['timestamp_sec'].values
                
                for ts in cam_ts:
                    if window_start <= ts <= window_end:
                        if start_time <= ts <= end_time:
                            position = "during"
                        elif ts < start_time:
                            position = f"before_{(start_time - ts)*1000:.1f}ms"
                        else:
                            position = f"after_{(ts - end_time)*1000:.1f}ms"
                        
                        matched_cameras[cam_id].append({
                            'timestamp': ts,
                            'position': position
                        })
            
            alignment_results.append({
                'anomaly': anomaly,
                'matched_cameras': dict(matched_cameras)
            })
        
        return alignment_results
    
    def analyze_blocking(self, alignment_results, dt_mean, dt_std, threshold):
        print("\n" + "="*70)
        print("详细检测结果")
        print("="*70)
        
        stats = {
            'total_anomalies': len(alignment_results),
            'camera_related': 0,
            'camera_during': 0,
            'max_n_sigma': 0,
            'blocking_detected': False
        }
        
        if not alignment_results:
            print(f"\n未发现异常IMU间隔")
            return stats
        
        print(f"\n异常判定阈值: {threshold*1000:.2f}ms (均值 + {N_SIGMA}倍标准差)")
        
        for i, result in enumerate(alignment_results, 1):
            anomaly = result['anomaly']
            cameras = result['matched_cameras']
            
            has_camera_match = len(cameras) > 0
            has_camera_during = any(
                any(m['position'] == 'during' for m in matches)
                for matches in cameras.values()
            )
            
            if has_camera_match:
                stats['camera_related'] += 1
            if has_camera_during:
                stats['camera_during'] += 1
            
            stats['max_n_sigma'] = max(stats['max_n_sigma'], anomaly['n_sigma'])
            
            print(f"\n[{i}/{len(alignment_results)}] IMU异常间隔 #{anomaly['idx']}")
            print(f"  时间范围: {anomaly['start_time']:.6f}s - {anomaly['end_time']:.6f}s")
            print(f"  间隔长度: {anomaly['duration']*1000:.2f}ms ({anomaly['n_sigma']:.2f}倍标准差)")
            
            if not cameras:
                print(f"  相关相机: 无")
            else:
                print(f"  相关相机:")
                for cam_id, matches in cameras.items():
                    print(f"    [cam{cam_id}] {len(matches)} 帧")
                    for m in matches[:3]:
                        pos_str = m['position']
                        if pos_str == "during":
                            print(f"      - {m['timestamp']:.6f}s (在IMU间隔期间) <<< 阻塞证据")
                        else:
                            print(f"      - {m['timestamp']:.6f}s ({pos_str})")
                    if len(matches) > 3:
                        print(f"      ... 还有 {len(matches)-3} 帧")
        
        stats['blocking_detected'] = stats['camera_during'] > 0
        return stats
    
    def detect_nominal_interval(self, dt):
        """
        自动检测 IMU 的标称间隔（基于中位数，对异常值鲁棒）
        返回 (nominal_ms, nominal_hz)
        """
        dt_ms = dt * 1000
        median_ms = np.median(dt_ms)
        
        # 匹配已知的 AirSim 物理引擎 tick 率
        known_ticks = {
            8.333: 120,   # 120Hz (默认 PhysX)
            16.667: 60,   # 60Hz
            33.333: 30,   # 30Hz
        }
        
        # 找最接近的已知 tick
        best_tick = min(known_ticks.keys(), key=lambda t: abs(median_ms - t))
        if abs(median_ms - best_tick) < best_tick * 0.15:  # 15% 容差
            nominal_ms = best_tick
            nominal_hz = known_ticks[best_tick]
        else:
            # 未匹配已知 tick，使用中位数
            nominal_ms = median_ms
            nominal_hz = 1000.0 / median_ms
        
        return nominal_ms, nominal_hz
    
    def analyze_interval_distribution(self, dt):
        """分析IMU间隔的分布，自适应不同采样率"""
        dt_ms = dt * 1000
        nominal_ms, nominal_hz = self.detect_nominal_interval(dt)
        
        # 基于检测到的标称间隔动态生成分类 bin
        # 标称 ± 25% 为 "正常"，2x/3x/4x 标称为降频模式
        half_width = nominal_ms * 0.25
        bins_def = [
            (f'{nominal_ms:.2f}ms ({nominal_hz:.0f}Hz, 标称)',
             nominal_ms - half_width, nominal_ms + half_width),
            (f'{nominal_ms*2:.1f}ms ({nominal_hz/2:.0f}Hz, 半频)',
             nominal_ms * 2 - half_width, nominal_ms * 2 + half_width),
            (f'{nominal_ms*3:.1f}ms ({nominal_hz/3:.0f}Hz, 1/3频)',
             nominal_ms * 3 - half_width, nominal_ms * 3 + half_width),
        ]
        
        # 图像阻塞区间（> 3x 标称）
        blocking_threshold = nominal_ms * 3 + half_width
        
        bins = {}
        classified_mask = np.zeros(len(dt_ms), dtype=bool)
        
        for label, lo, hi in bins_def:
            mask = (dt_ms >= lo) & (dt_ms < hi)
            bins[label] = np.sum(mask)
            classified_mask |= mask
        
        # 大间隔（图像采集阻塞）
        blocking_mask = dt_ms >= blocking_threshold
        bins[f'>{blocking_threshold:.0f}ms (图像采集阻塞)'] = np.sum(blocking_mask)
        classified_mask |= blocking_mask
        
        bins['其他'] = np.sum(~classified_mask)
        
        print("\n" + "="*70)
        print(f"IMU间隔分布分析（自动检测标称频率: {nominal_hz:.0f}Hz / {nominal_ms:.2f}ms）")
        print("="*70)
        
        for name, count in bins.items():
            if count > 0:
                pct = count / len(dt_ms) * 100
                print(f"   {name}: {count} ({pct:.1f}%)")
        
        # 统计非标称间隔
        non_nominal_mask = dt_ms > (nominal_ms + half_width)
        if non_nominal_mask.any():
            non_nominal = dt_ms[non_nominal_mask]
            print(f"\n   非标称间隔统计:")
            print(f"     数量: {len(non_nominal)} ({len(non_nominal)/len(dt_ms)*100:.1f}%)")
            print(f"     最小: {np.min(non_nominal):.2f}ms")
            print(f"     最大: {np.max(non_nominal):.2f}ms")
            print(f"     均值: {np.mean(non_nominal):.2f}ms")
            
            # 找峰值
            if len(non_nominal) > 5:
                hist, edges = np.histogram(non_nominal, bins=20)
                max_bin_idx = np.argmax(hist)
                peak_center = (edges[max_bin_idx] + edges[max_bin_idx+1]) / 2
                print(f"     主要峰值: ~{peak_center:.1f}ms ({hist[max_bin_idx]} 个)")
        
        # VIO 适用性评估
        camera_fps = 2  # 可从 config 获取
        inter_frame_ms = 1000.0 / camera_fps
        nominal_per_frame = inter_frame_ms / nominal_ms
        blocking_count = np.sum(blocking_mask)
        lost_per_frame = blocking_count / (len(dt_ms) * nominal_ms / inter_frame_ms) if len(dt_ms) > 0 else 0
        actual_per_frame = nominal_per_frame - lost_per_frame
        
        print(f"\n   VIO 适用性评估 ({camera_fps}Hz 相机):")
        print(f"     帧间隔: {inter_frame_ms:.0f}ms")
        print(f"     理论 IMU 数/帧: {nominal_per_frame:.0f}")
        print(f"     阻塞导致的丢失: ~{lost_per_frame:.1f} 个/帧")
        actual_samples = nominal_per_frame - lost_per_frame
        if actual_samples >= 20:
            print(f"     实际 IMU 数/帧: ~{actual_samples:.0f} → ✓ 充足（VINS 预积分需 10-20 个）")
        elif actual_samples >= 10:
            print(f"     实际 IMU 数/帧: ~{actual_samples:.0f} → ○ 勉强足够")
        else:
            print(f"     实际 IMU 数/帧: ~{actual_samples:.0f} → ✗ 不足，可能影响 VIO")
        
        return nominal_ms, nominal_hz
    
    def print_final_summary(self, stats, dt_mean, dt_std, threshold, total_intervals):
        print("\n" + "="*70)
        print("【时间戳检验结果总结】")
        print("="*70)
        
        print(f"\n1. IMU采样统计")
        print(f"   总采样数: {len(self.imu_data)}")
        print(f"   间隔统计: 均值={dt_mean*1000:.3f}ms, 标准差={dt_std*1000:.3f}ms")
        print(f"   检测阈值: {threshold*1000:.2f}ms (均值 + {N_SIGMA}倍标准差)")
        
        print(f"\n2. 间隔分布模式")
        nominal_ms = getattr(self, 'nominal_ms', dt_mean * 1000)
        nominal_hz = getattr(self, 'nominal_hz', 1000.0 / (dt_mean * 1000))
        print(f"   检测到标称频率: {nominal_hz:.0f}Hz ({nominal_ms:.2f}ms)")
        print(f"   标称间隔: 正常采样")
        print(f"   半频 ({nominal_hz/2:.0f}Hz): 偶尔降频")
        print(f"   更大间隔: 图像采集阻塞")
        
        print(f"\n3. 异常检测")
        print(f"   总间隔数: {total_intervals}")
        print(f"   异常间隔: {stats['total_anomalies']} ({stats['total_anomalies']/total_intervals*100:.2f}%)")
        print(f"   最大偏离: {stats['max_n_sigma']:.2f}倍标准差")
        
        print(f"\n4. 相机关联分析")
        print(f"   与相机相关的异常: {stats['camera_related']}/{stats['total_anomalies']}")
        print(f"   相机在间隔期间: {stats['camera_during']}/{stats['total_anomalies']}")
        
        print(f"\n5. 可能原因分析")
        if stats['camera_during'] / max(stats['total_anomalies'], 1) > 0.8:
            print("   → 图像采集阻塞（相机帧在IMU间隔期间）")
        elif stats['camera_related'] / max(stats['total_anomalies'], 1) > 0.5:
            print("   → 图像采集相关延迟（但不同步）")
        else:
            print("   → 与图像采集无关（可能是系统调度、仿真性能等）")
        
        print(f"\n6. 结论")
        if stats['total_anomalies'] == 0:
            print("   ✓ IMU时间戳均匀，无明显堵塞")
        elif stats['camera_during'] > 0:
            print(f"   ⚠ 检测到 {stats['camera_during']} 次图像保存阻塞IMU的事件")
            print("   建议: 检查同步模式配置，确保图像采集不阻塞传感器")
        elif stats['camera_related'] > stats['total_anomalies'] * 0.5:
            print("   ⚠ 大多数异常与相机保存时间相关")
            print("   图像保存可能是主要影响因素")
        else:
            print("   ? 异常间隔与图像保存无明显关联")
            print("   可能由其他因素导致（系统负载、仿真性能等）")
        
        print("\n" + "="*70)
    
    def plot_imu_intervals(self, timestamps, dt, anomalies, threshold):
        """绘制IMU间隔时间序列图，并在相机帧位置标记时间戳"""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 根据时间范围过滤数据
        time_points = timestamps[1:]
        dt_ms = dt * 1000
        
        if PLOT_TIME_RANGE is not None:
            t_start, t_end = PLOT_TIME_RANGE
            mask = (time_points >= t_start) & (time_points <= t_end)
            time_points = time_points[mask]
            dt_ms = dt_ms[mask]
            print(f"绘图时间范围: {t_start}s - {t_end}s ({len(time_points)} 个数据点)")
        else:
            print(f"绘图时间范围: 全部 ({time_points[0]:.1f}s - {time_points[-1]:.1f}s, {len(time_points)} 个数据点)")
        
        fig, ax = plt.subplots(1, 1, figsize=(16, 6))
        
        # 绘制IMU间隔曲线
        ax.plot(time_points, dt_ms, 'b-', linewidth=0.8, alpha=0.6, label='IMU Interval')
        
        # 阈值线
        ax.axhline(y=threshold * 1000, color='r', linestyle='--', linewidth=1.5, 
                  label=f'Threshold ({threshold*1000:.1f}ms)')
        
        # 均值线
        ax.axhline(y=np.mean(dt) * 1000, color='g', linestyle=':', alpha=0.7,
                  label=f'Mean ({np.mean(dt)*1000:.1f}ms)')
        
        # 参考线（自动检测或手动指定）
        ref_ms = REFERENCE_INTERVAL_MS if REFERENCE_INTERVAL_MS is not None else getattr(self, 'nominal_ms', 8.333)
        ax.axhline(y=ref_ms, color='orange', linestyle='-.', alpha=0.8,
                  linewidth=1.5, label=f'Nominal: {ref_ms:.2f}ms ({1000/ref_ms:.0f}Hz)')
        
        # 标记异常点
        if anomalies:
            if PLOT_TIME_RANGE is not None:
                t_start, t_end = PLOT_TIME_RANGE
                visible_anomalies = [a for a in anomalies if t_start <= a['end_time'] <= t_end]
            else:
                visible_anomalies = anomalies
            
            if visible_anomalies:
                anomaly_times = [a['end_time'] for a in visible_anomalies]
                anomaly_dts = [a['duration'] * 1000 for a in visible_anomalies]
                ax.scatter(anomaly_times, anomaly_dts, color='red', s=50, zorder=5, 
                          label=f'Anomalies ({len(visible_anomalies)}/{len(anomalies)})')
        
        # 标记相机时间戳（竖线+时间戳文字）
        for cam_id, cam_df in self.cam_data.items():
            if cam_df is not None:
                cam_ts = cam_df['timestamp_sec'].values
                
                # 过滤时间范围
                if PLOT_TIME_RANGE is not None:
                    t_start, t_end = PLOT_TIME_RANGE
                    mask = (cam_ts >= t_start) & (cam_ts <= t_end)
                    cam_ts = cam_ts[mask]
                
                # 绘制竖线和时间戳
                y_base = 0
                y_height = 3 + cam_id * 2
                
                for ts in cam_ts:
                    # 画竖线
                    ax.plot([ts, ts], [y_base, y_height], color=f'C{cam_id+2}', 
                           alpha=0.6, linewidth=1.5)
                    
                    # 添加时间戳文字（旋转90度避免重叠）
                    ax.text(ts, y_height + 1, f'{ts:.2f}s', 
                           rotation=90, fontsize=7, ha='center', va='bottom',
                           color=f'C{cam_id+2}', alpha=0.9)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('IMU Interval (ms)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # 添加统计文本
        ref_ms = REFERENCE_INTERVAL_MS if REFERENCE_INTERVAL_MS is not None else getattr(self, 'nominal_ms', 8.333)
        stats_text = f"N={len(dt_ms)} | Mean={np.mean(dt_ms):.2f}ms | Std={np.std(dt_ms):.2f}ms | Median={np.median(dt_ms):.2f}ms | "
        stats_text += f">{ref_ms:.1f}ms: {np.sum(dt_ms > ref_ms * 1.5)}"
        if PLOT_TIME_RANGE is not None:
            stats_text += f" | Range: {PLOT_TIME_RANGE[0]}-{PLOT_TIME_RANGE[1]}s"
        
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
               verticalalignment='top', horizontalalignment='left',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
               fontsize=9, family='monospace')
        
        plt.tight_layout()
        output_path = self.output_dir / "imu_interval_timeline.png"
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\n图表已保存: {output_path}")
        plt.close()
    
    def run(self):
        print("="*70)
        print("数据集时间戳验证 - 图像阻塞检测")
        print("="*70)
        print(f"数据集路径: {self.dataset_path}")
        print(f"输出目录: {self.output_dir}")
        print()
        
        print("[1/3] 加载数据...")
        self.load_imu_data()
        self.load_camera_data(2)
        self.load_camera_data(3)
        self.load_groundtruth_data()
        
        if self.imu_data is None:
            print("错误: 无法加载IMU数据")
            return
        
        print("\n[2/3] 检测异常IMU间隔...")
        timestamps = self.imu_data['timestamp_sec'].values
        anomalies, dt_mean, dt_std, threshold, dt = self.find_anomalous_imu_intervals(
            self.imu_data, N_SIGMA
        )
        
        total_intervals = len(dt)
        
        print(f"IMU间隔统计: 均值={dt_mean*1000:.3f}ms, 标准差={dt_std*1000:.3f}ms")
        print(f"异常判定阈值 ({N_SIGMA}倍标准差): {threshold*1000:.2f}ms")
        print(f"发现 {len(anomalies)} 个异常间隔（共 {total_intervals} 个，占比 {len(anomalies)/total_intervals*100:.2f}%）")
        
        print("\n[3/3] 分析相机关联...")
        alignment_results = self.check_camera_alignment(anomalies, TIME_ALIGNMENT_TOLERANCE)
        
        stats = self.analyze_blocking(alignment_results, dt_mean, dt_std, threshold)
        
        # 分析间隔分布（会自动检测标称频率）
        nominal_ms, nominal_hz = self.analyze_interval_distribution(dt)
        self.nominal_ms = nominal_ms
        self.nominal_hz = nominal_hz
        
        print("\n生成图表...")
        self.plot_imu_intervals(timestamps, dt, anomalies, threshold)
        
        # 速度积分验证
        self.integrate_velocity_from_groundtruth()
        
        self.print_final_summary(stats, dt_mean, dt_std, threshold, total_intervals)
        
        print("验证完成！")
        print("="*70)


# ============================================
# 主程序入口
# ============================================

if __name__ == "__main__":
    try:
        verifier = DatasetVerifier(DATASET_PATH, OUTPUT_DIR)
        verifier.run()
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
