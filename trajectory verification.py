"""
轨迹生成测试和可视化脚本
用于验证各种轨迹类型的生成代码是否正确
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import matplotlib.patches as mpatches

def generate_trajectory(trajectory_type, area_range=(-5.0, 5.0), spacing=2.0, 
                       min_wp=10, max_wp=20):
    """
    生成轨迹（复制自lunar_rover_env.py）
    """
    n_waypoints = np.random.randint(min_wp, max_wp + 1)
    trajectory = []
    
    if trajectory_type == 'random_walk':
        current_pos = np.array([0.0, 0.0])
        trajectory.append((0.0, 0.0, 0.0))
        
        for i in range(n_waypoints - 1):
            angle = np.random.uniform(0, 2 * np.pi)
            step = np.array([spacing * np.cos(angle), spacing * np.sin(angle)])
            new_pos = current_pos + step
            new_pos = np.clip(new_pos, area_range[0], area_range[1])
            direction = new_pos - current_pos
            yaw = np.arctan2(direction[1], direction[0])
            trajectory.append((new_pos[0], new_pos[1], yaw))
            current_pos = new_pos
    
    elif trajectory_type == 'figure_eight':
        t_values = np.linspace(0, 4 * np.pi, n_waypoints)
        scale = area_range[1] * 0.8
        
        for t in t_values:
            x = scale * np.sin(t) / (1 + np.cos(t)**2)
            y = scale * np.sin(t) * np.cos(t) / (1 + np.cos(t)**2)
            
            # 计算切线方向
            dx = scale * np.cos(t) * (1 + np.cos(t)**2) - 2 * scale * np.sin(t) * np.cos(t) * (-np.sin(t))
            dy = scale * np.cos(2*t) * (1 + np.cos(t)**2) - 2 * scale * np.sin(t) * np.cos(t) * (-np.sin(t))
            yaw = np.arctan2(dy, dx)
            
            trajectory.append((x, y, yaw))
    
    elif trajectory_type == 'spiral':
        t_values = np.linspace(0, 4 * np.pi, n_waypoints)
        max_radius = area_range[1] * 0.9
        
        for t in t_values:
            r = max_radius * (t / (4 * np.pi))
            x = r * np.cos(t)
            y = r * np.sin(t)
            
            # 计算切线方向
            dx = np.cos(t) * max_radius / (4 * np.pi) - r * np.sin(t)
            dy = np.sin(t) * max_radius / (4 * np.pi) + r * np.cos(t)
            yaw = np.arctan2(dy, dx)
            
            trajectory.append((x, y, yaw))
    
    elif trajectory_type == 'grid':
        grid_size = int(np.sqrt(n_waypoints))
        x_points = np.linspace(area_range[0], area_range[1], grid_size)
        y_points = np.linspace(area_range[0], area_range[1], grid_size)
        
        for i, x in enumerate(x_points):
            if i % 2 == 0:
                y_iter = y_points
            else:
                y_iter = list(reversed(y_points))
            
            for y in y_iter:
                if len(trajectory) > 0:
                    prev_x, prev_y, _ = trajectory[-1]
                    yaw = np.arctan2(y - prev_y, x - prev_x)
                else:
                    yaw = 0.0
                trajectory.append((x, y, yaw))
    
    return trajectory


def plot_trajectory(trajectory, trajectory_type, ax=None, show_arrows=True):
    """
    绘制轨迹
    Args:
        trajectory: 轨迹列表 [(x, y, yaw), ...]
        trajectory_type: 轨迹类型名称
        ax: matplotlib axis对象
        show_arrows: 是否显示方向箭头
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 10))
    
    # 提取坐标和朝向
    x_coords = [p[0] for p in trajectory]
    y_coords = [p[1] for p in trajectory]
    yaws = [p[2] for p in trajectory]
    
    # 绘制轨迹路径
    ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.6, label='Trajectory Path')
    
    # 绘制航点
    ax.scatter(x_coords, y_coords, c=range(len(trajectory)), cmap='viridis', 
              s=100, zorder=5, edgecolors='black', linewidth=1, label='Waypoints')
    
    # 标记起点和终点
    ax.scatter(x_coords[0], y_coords[0], c='green', s=300, marker='*', 
              edgecolors='black', linewidth=2, zorder=10, label='Start')
    ax.scatter(x_coords[-1], y_coords[-1], c='red', s=300, marker='s', 
              edgecolors='black', linewidth=2, zorder=10, label='End')
    
    # 绘制方向箭头（每隔几个点绘制一个，避免太密集）
    if show_arrows:
        arrow_interval = max(1, len(trajectory) // 15)  # 大约显示15个箭头
        arrow_length = 0.5  # 箭头长度
        
        for i in range(0, len(trajectory), arrow_interval):
            x, y, yaw = trajectory[i]
            dx = arrow_length * np.cos(yaw)
            dy = arrow_length * np.sin(yaw)
            
            arrow = FancyArrowPatch((x, y), (x + dx, y + dy),
                                  arrowstyle='->', mutation_scale=20, 
                                  linewidth=2, color='red', alpha=0.7, zorder=3)
            ax.add_patch(arrow)
    
    # 设置图表属性
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_title(f'{trajectory_type} Trajectory\n(Total {len(trajectory)} waypoints)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend(loc='upper right')
    
    # 添加统计信息
    total_distance = 0
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i-1][0]
        dy = trajectory[i][1] - trajectory[i-1][1]
        total_distance += np.sqrt(dx**2 + dy**2)
    
    info_text = f'总路程: {total_distance:.2f} m\n航点数: {len(trajectory)}'
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
           verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
           fontsize=10)
    
    return ax


def test_all_trajectories():
    """
    测试所有轨迹类型
    """
    np.random.seed(42)  # 固定随机种子以便复现
    
    trajectory_types = ['random_walk', 'figure_eight', 'spiral', 'grid']
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 16))
    axes = axes.flatten()
    
    for idx, traj_type in enumerate(trajectory_types):
        print(f"生成 {traj_type} 轨迹...")
        trajectory = generate_trajectory(traj_type, area_range=(-5.0, 5.0), 
                                        spacing=2.0, min_wp=15, max_wp=20)
        
        plot_trajectory(trajectory, traj_type, ax=axes[idx], show_arrows=True)
        
        # 打印轨迹信息
        print(f"  航点数: {len(trajectory)}")
        print(f"  X范围: [{min([p[0] for p in trajectory]):.2f}, {max([p[0] for p in trajectory]):.2f}]")
        print(f"  Y范围: [{min([p[1] for p in trajectory]):.2f}, {max([p[1] for p in trajectory]):.2f}]")
        print()
    
    plt.tight_layout()
    plt.savefig('trajectory_test_all.png', dpi=150, bbox_inches='tight')
    print("所有轨迹图已保存为 trajectory_test_all.png")
    plt.show()


def test_figure_eight_detail():
    """
    详细测试figure_eight轨迹（用于验证数学公式）
    """
    print("详细测试 figure_eight 轨迹...")
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    # 测试不同航点数量
    waypoint_counts = [10, 20, 40]
    
    for idx, n_wp in enumerate(waypoint_counts):
        # 手动生成轨迹以控制航点数
        t_values = np.linspace(0, 4 * np.pi, n_wp)
        scale = 4.0
        trajectory = []
        
        for t in t_values:
            x = scale * np.sin(t) / (1 + np.cos(t)**2)
            y = scale * np.sin(t) * np.cos(t) / (1 + np.cos(t)**2)
            
            dx = scale * np.cos(t) * (1 + np.cos(t)**2) - 2 * scale * np.sin(t) * np.cos(t) * (-np.sin(t))
            dy = scale * np.cos(2*t) * (1 + np.cos(t)**2) - 2 * scale * np.sin(t) * np.cos(t) * (-np.sin(t))
            yaw = np.arctan2(dy, dx)
            
            trajectory.append((x, y, yaw))
        
        plot_trajectory(trajectory, f'figure_eight ({n_wp} points)', ax=axes[idx], show_arrows=True)
        print(f"  {n_wp}个航点时的轨迹范围: X[{min([p[0] for p in trajectory]):.2f}, {max([p[0] for p in trajectory]):.2f}], "
              f"Y[{min([p[1] for p in trajectory]):.2f}, {max([p[1] for p in trajectory]):.2f}]")
    
    plt.tight_layout()
    plt.savefig('trajectory_test_figure_eight.png', dpi=150, bbox_inches='tight')
    print("Figure-8轨迹详细图已保存为 trajectory_test_figure_eight.png")
    plt.show()


def test_spiral_detail():
    """
    详细测试spiral轨迹
    """
    print("详细测试 spiral 轨迹...")
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    # 测试不同航点数量
    waypoint_counts = [10, 20, 40]
    
    for idx, n_wp in enumerate(waypoint_counts):
        t_values = np.linspace(0, 4 * np.pi, n_wp)
        max_radius = 4.5
        trajectory = []
        
        for t in t_values:
            r = max_radius * (t / (4 * np.pi))
            x = r * np.cos(t)
            y = r * np.sin(t)
            
            dx = np.cos(t) * max_radius / (4 * np.pi) - r * np.sin(t)
            dy = np.sin(t) * max_radius / (4 * np.pi) + r * np.cos(t)
            yaw = np.arctan2(dy, dx)
            
            trajectory.append((x, y, yaw))
        
        plot_trajectory(trajectory, f'spiral ({n_wp} points)', ax=axes[idx], show_arrows=True)
        print(f"  {n_wp}个航点时的轨迹范围: X[{min([p[0] for p in trajectory]):.2f}, {max([p[0] for p in trajectory]):.2f}], "
              f"Y[{min([p[1] for p in trajectory]):.2f}, {max([p[1] for p in trajectory]):.2f}]")
    
    plt.tight_layout()
    plt.savefig('trajectory_test_spiral.png', dpi=150, bbox_inches='tight')
    print("Spiral轨迹详细图已保存为 trajectory_test_spiral.png")
    plt.show()


def test_yaw_continuity():
    """
    测试yaw角的连续性（验证朝向计算是否合理）
    """
    print("测试yaw角连续性...")
    
    trajectory_types = ['figure_eight', 'spiral']
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    
    for idx, traj_type in enumerate(trajectory_types):
        trajectory = generate_trajectory(traj_type, min_wp=30, max_wp=30)
        
        # 提取yaw角
        yaws = [p[2] for p in trajectory]
        yaws_deg = [np.degrees(y) for y in yaws]
        
        # 计算yaw角变化率
        yaw_changes = []
        for i in range(1, len(yaws)):
            dy = yaws[i] - yaws[i-1]
            # 处理角度跳变（-π到π的边界）
            if dy > np.pi:
                dy -= 2 * np.pi
            elif dy < -np.pi:
                dy += 2 * np.pi
            yaw_changes.append(np.degrees(dy))
        
        # 左图：轨迹图
        ax_traj = axes[idx, 0]
        plot_trajectory(trajectory, traj_type, ax=ax_traj, show_arrows=True)
        
        # 右图：yaw角变化
        ax_yaw = axes[idx, 1]
        ax_yaw.plot(range(len(yaws_deg)), yaws_deg, 'b-o', markersize=4, label='Yaw angle')
        ax_yaw.set_xlabel('Waypoint Index', fontsize=12)
        ax_yaw.set_ylabel('Yaw angle (degree)', fontsize=12)
        ax_yaw.set_title(f'{traj_type} - Yaw angle change', fontsize=12, fontweight='bold')
        ax_yaw.grid(True, alpha=0.3)
        ax_yaw.legend()
        
        # 添加yaw变化率信息
        if yaw_changes:
            max_change = max(abs(c) for c in yaw_changes)
            avg_change = np.mean([abs(c) for c in yaw_changes])
            info = f'Max yaw change: {max_change:.2f}°\nAverage yaw change: {avg_change:.2f}°'
            ax_yaw.text(0.02, 0.98, info, transform=ax_yaw.transAxes, 
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig('trajectory_test_yaw_continuity.png', dpi=150, bbox_inches='tight')
    print("Yaw连续性测试图已保存为 trajectory_test_yaw_continuity.png")
    plt.show()


if __name__ == "__main__":
    print("=" * 60)
    print("轨迹生成测试脚本")
    print("=" * 60)
    print()
    
    # 测试1：所有轨迹类型
    print("【测试1】生成并绘制所有轨迹类型...")
    test_all_trajectories()
    print()
    
    # 测试2：详细测试figure_eight
    print("【测试2】详细测试figure_eight轨迹...")
    test_figure_eight_detail()
    print()
    
    # 测试3：详细测试spiral
    print("【测试3】详细测试spiral轨迹...")
    test_spiral_detail()
    print()
    
    # 测试4：测试yaw连续性
    print("【测试4】测试yaw角连续性...")
    test_yaw_continuity()
    print()
    
    print("=" * 60)
    print("所有测试完成！")
    print("生成的图像文件:")
    print("  - trajectory_test_all.png (所有轨迹)")
    print("  - trajectory_test_figure_eight.png (figure_eight详细)")
    print("  - trajectory_test_spiral.png (spiral详细)")
    print("  - trajectory_test_yaw_continuity.png (yaw连续性)")
    print("=" * 60)
