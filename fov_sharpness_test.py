"""
FOV 梯度测试脚本
用于验证图像模糊是否与FOV/对焦有关
测试范围：10° - 120°，观察不同FOV下的图像清晰度

使用方法：
1. 确保 AirSim 和 UE4 场景已启动
2. 将车辆手动驾驶到距离纹理物体 2m 左右的位置
3. 运行：python fov_sharpness_test.py
4. 查看输出的清晰度报告和对比图像
"""

import airsim
import numpy as np
import cv2
import json
import time
import os
from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt

# 确保中文字体正常显示
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class FOVSharpnessTester:
    """FOV清晰度测试器"""
    
    def __init__(self, vehicle_name="LunarRover"):
        """初始化测试器"""
        self.vehicle_name = vehicle_name
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        
        # ============================================
        # 禁用 TAA 和启用 Super Sampling（通过 UE4 控制台命令）
        # ============================================
        print("\n【初始化】应用图像优化设置...")
        
        # 禁用 TAA（解决油画/涂抹感）
        self.client.simRunConsoleCommand("r.DefaultFeature.AntiAliasing 0")
        print("  ✓ TAA 已禁用")
        
        # 启用 Super Sampling（提高渲染分辨率）
        # 150 = 1.5x 超采样，200 = 2x（性能开销更大）
        self.client.simRunConsoleCommand("r.ScreenPercentage 150")
        print("  ✓ Super Sampling 已启用 (150%)")
        
        # 其他优化设置
        self.client.simRunConsoleCommand("r.MaxAnisotropy 16")
        print("  ✓ 各向异性过滤已设置为 16x")
        
        self.client.simRunConsoleCommand("r.MotionBlurQuality 0")
        print("  ✓ 运动模糊质量已设为 0")
        
        self.client.simRunConsoleCommand("r.BloomQuality 0")
        print("  ✓ 泛光质量已设为 0")
        
        # 等待设置生效
        print("  等待设置生效...")
        time.sleep(1.0)
        
        # 创建测试结果目录
        self.test_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.test_dir = Path(f"./fov_test_{self.test_timestamp}")
        self.test_dir.mkdir(parents=True, exist_ok=True)
        
        # 测试配置
        self.test_fov_values = [10, 35, 70, 100, 120]  # 测试的FOV值
        self.resolution = (840, 840)  # 固定分辨率
        self.camera_name = "obstacle_camera_left"  # 测试的相机名称
        
        # 记录每个FOV的测试结果
        self.results = []
        
        print(f"FOV清晰度测试初始化完成")
        print(f"测试结果将保存到: {self.test_dir}")
        print(f"测试FOV值: {self.test_fov_values}")
        print(f"\n请将车辆放置在距离纹理物体 2m 左右的位置，然后按 Enter 开始测试...")
        input()
    
    def measure_sharpness(self, image):
        """
        测量图像清晰度
        使用多种指标综合评估
        
        Args:
            image: numpy数组，RGB图像
        Returns:
            dict: 各种清晰度指标
        """
        # 转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        else:
            gray = image
        
        # 1. 拉普拉斯方差（越高越清晰）
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        
        # 2. Sobel 边缘强度（越高边缘越明显）
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        sobel_mag = np.sqrt(sobelx**2 + sobely**2).mean()
        
        # 3. 边缘密度（Canny边缘检测后的非零像素比例）
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.count_nonzero(edges) / (edges.shape[0] * edges.shape[1])
        
        # 4. 高频成分能量（FFT）
        fft = np.fft.fft2(gray)
        fft_shift = np.fft.fftshift(fft)
        
        # 计算中心区域的低频能量
        h, w = gray.shape
        cy, cx = h // 2, w // 2
        radius = min(h, w) // 8
        y, x = np.ogrid[:h, :w]
        mask_high_freq = ((x - cx)**2 + (y - cy)**2) > radius**2
        high_freq_energy = np.abs(fft_shift[mask_high_freq]).mean()
        
        return {
            'laplacian_var': laplacian_var,
            'sobel_mag': sobel_mag,
            'edge_density': edge_density,
            'high_freq_energy': high_freq_energy
        }
    
    def capture_with_fov(self, fov):
        """
        使用指定FOV采集图像
        注意：这会通过AirSim API临时修改相机FOV（如果支持）
        
        Args:
            fov: 视场角度数
        Returns:
            image: numpy数组，RGB图像
        """
        # 尝试通过API设置FOV（AirSim 1.8+支持）
        try:
            camera_name = f"{self.camera_name}"
            self.client.simSetCameraFov(camera_name, fov, self.vehicle_name)
            print(f"  已设置相机FOV为 {fov}°")
            time.sleep(0.5)  # 等待设置生效
        except Exception as e:
            print(f"  警告：无法通过API设置FOV ({e})")
            print(f"  请手动修改 settings.json 中的 FOV_Degrees 为 {fov} 后重启AirSim")
            print(f"  按 Enter 继续...")
            input()
        
        # 重新应用控制台命令（防止被重置）
        self.client.simRunConsoleCommand("r.DefaultFeature.AntiAliasing 0")
        self.client.simRunConsoleCommand("r.ScreenPercentage 150")
        self.client.simRunConsoleCommand("r.MaxAnisotropy 16")
        time.sleep(0.2)
        
        # 采集图像
        request = airsim.ImageRequest(
            self.camera_name, 
            airsim.ImageType.Scene, 
            pixels_as_float=False, 
            compress=False
        )
        responses = self.client.simGetImages([request], self.vehicle_name)
        
        if len(responses) == 0 or responses[0].width == 0:
            raise RuntimeError(f"无法获取图像，FOV={fov}")
        
        # 转换为numpy数组
        response = responses[0]
        img = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img = img.reshape(response.height, response.width, 3)
        
        # 如果采集分辨率高于 840x840，缩小到 840x840（软件超采样）
        target_size = 840
        if response.width > target_size or response.height > target_size:
            img = cv2.resize(img, (target_size, target_size), interpolation=cv2.INTER_AREA)
            print(f"  已缩小图像: {response.width}x{response.height} -> {target_size}x{target_size}")
        
        return img
    
    def analyze_regions(self, image):
        """
        分析图像不同区域的清晰度
        将图像分为中心、左上、右上、左下、右下五个区域
        
        Args:
            image: numpy数组
        Returns:
            dict: 各区域的清晰度
        """
        h, w = image.shape[:2]
        regions = {
            'center': image[h//4:3*h//4, w//4:3*w//4],
            'top_left': image[:h//2, :w//2],
            'top_right': image[:h//2, w//2:],
            'bottom_left': image[h//2:, :w//2],
            'bottom_right': image[h//2:, w//2:]
        }
        
        results = {}
        for name, region in regions.items():
            results[name] = self.measure_sharpness(region)
        
        return results
    
    def run_test(self):
        """运行完整测试"""
        print("\n" + "="*60)
        print("开始 FOV 梯度测试")
        print("="*60)
        
        for fov in self.test_fov_values:
            print(f"\n测试 FOV = {fov}°...")
            
            try:
                # 采集图像
                img = self.capture_with_fov(fov)
                
                # 保存原始图像
                img_path = self.test_dir / f"fov_{fov:03d}_raw.png"
                cv2.imwrite(str(img_path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                
                # 测量整体清晰度
                overall_sharpness = self.measure_sharpness(img)
                
                # 分析各区域清晰度
                region_sharpness = self.analyze_regions(img)
                
                # 计算等效焦距（假设8mm对应70°）
                equiv_focal_length = 8 * 70 / fov
                
                # 记录结果
                result = {
                    'fov': fov,
                    'equiv_focal_length_mm': equiv_focal_length,
                    'image_shape': img.shape,
                    'overall': overall_sharpness,
                    'regions': region_sharpness
                }
                self.results.append(result)
                
                # 打印结果
                print(f"  等效焦距: {equiv_focal_length:.1f}mm")
                print(f"  整体清晰度 (Laplacian): {overall_sharpness['laplacian_var']:.2f}")
                print(f"  边缘强度 (Sobel): {overall_sharpness['sobel_mag']:.2f}")
                print(f"  高频能量: {overall_sharpness['high_freq_energy']:.2f}")
                
                # 检查区域差异（判断是否是畸变导致的边缘模糊）
                center_lap = region_sharpness['center']['laplacian_var']
                corner_lap = (region_sharpness['top_left']['laplacian_var'] + 
                             region_sharpness['top_right']['laplacian_var'] +
                             region_sharpness['bottom_left']['laplacian_var'] +
                             region_sharpness['bottom_right']['laplacian_var']) / 4
                
                print(f"  中心清晰度: {center_lap:.2f}")
                print(f"  四角平均清晰度: {corner_lap:.2f}")
                print(f"  中心/四角比: {center_lap/corner_lap:.2f}" if corner_lap > 0 else "  中心/四角比: N/A")
                
            except Exception as e:
                print(f"  错误: {e}")
                continue
        
        # 生成报告
        self.generate_report()
        
        print("\n" + "="*60)
        print(f"测试完成！结果保存在: {self.test_dir}")
        print("="*60)
    
    def generate_report(self):
        """生成测试报告"""
        if len(self.results) == 0:
            print("没有有效的测试结果")
            return
        
        # 保存JSON报告
        report_path = self.test_dir / "sharpness_report.json"
        with open(report_path, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        # 生成可视化图表
        self._plot_sharpness_comparison()
        self._plot_region_comparison()
        
        # 生成文本分析报告
        self._generate_text_report()
    
    def _plot_sharpness_comparison(self):
        """绘制清晰度对比图"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('FOV vs Sharpness Analysis', fontsize=16)
        
        fovs = [r['fov'] for r in self.results]
        
        # 图1: Laplacian方差
        lap_vars = [r['overall']['laplacian_var'] for r in self.results]
        axes[0, 0].plot(fovs, lap_vars, 'o-', linewidth=2, markersize=8, color='blue')
        axes[0, 0].set_xlabel('FOV (degrees)')
        axes[0, 0].set_ylabel('Laplacian Variance')
        axes[0, 0].set_title('Sharpness (Laplacian) - Higher is Sharper')
        axes[0, 0].grid(True, alpha=0.3)
        
        # 图2: Sobel边缘强度
        sobel_mags = [r['overall']['sobel_mag'] for r in self.results]
        axes[0, 1].plot(fovs, sobel_mags, 'o-', linewidth=2, markersize=8, color='green')
        axes[0, 1].set_xlabel('FOV (degrees)')
        axes[0, 1].set_ylabel('Sobel Magnitude')
        axes[0, 1].set_title('Edge Strength (Sobel) - Higher is Sharper')
        axes[0, 1].grid(True, alpha=0.3)
        
        # 图3: 边缘密度
        edge_densities = [r['overall']['edge_density'] for r in self.results]
        axes[1, 0].plot(fovs, edge_densities, 'o-', linewidth=2, markersize=8, color='red')
        axes[1, 0].set_xlabel('FOV (degrees)')
        axes[1, 0].set_ylabel('Edge Density')
        axes[1, 0].set_title('Edge Density - Higher is Sharper')
        axes[1, 0].grid(True, alpha=0.3)
        
        # 图4: 高频能量
        high_freqs = [r['overall']['high_freq_energy'] for r in self.results]
        axes[1, 1].plot(fovs, high_freqs, 'o-', linewidth=2, markersize=8, color='purple')
        axes[1, 1].set_xlabel('FOV (degrees)')
        axes[1, 1].set_ylabel('High Frequency Energy')
        axes[1, 1].set_title('High Frequency Energy - Higher is Sharper')
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.test_dir / 'sharpness_comparison.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"  已保存清晰度对比图: sharpness_comparison.png")
    
    def _plot_region_comparison(self):
        """绘制区域清晰度对比（检查畸变影响）"""
        fig, ax = plt.subplots(figsize=(12, 6))
        
        fovs = [r['fov'] for r in self.results]
        
        # 绘制中心和四角的对比
        center_sharp = [r['regions']['center']['laplacian_var'] for r in self.results]
        tl_sharp = [r['regions']['top_left']['laplacian_var'] for r in self.results]
        tr_sharp = [r['regions']['top_right']['laplacian_var'] for r in self.results]
        bl_sharp = [r['regions']['bottom_left']['laplacian_var'] for r in self.results]
        br_sharp = [r['regions']['bottom_right']['laplacian_var'] for r in self.results]
        
        ax.plot(fovs, center_sharp, 'o-', linewidth=2, markersize=8, label='Center', color='red')
        ax.plot(fovs, tl_sharp, 's--', linewidth=1.5, markersize=6, label='Top-Left', alpha=0.7)
        ax.plot(fovs, tr_sharp, '^--', linewidth=1.5, markersize=6, label='Top-Right', alpha=0.7)
        ax.plot(fovs, bl_sharp, 'v--', linewidth=1.5, markersize=6, label='Bottom-Left', alpha=0.7)
        ax.plot(fovs, br_sharp, 'd--', linewidth=1.5, markersize=6, label='Bottom-Right', alpha=0.7)
        
        ax.set_xlabel('FOV (degrees)', fontsize=12)
        ax.set_ylabel('Laplacian Variance (Sharpness)', fontsize=12)
        ax.set_title('Center vs Corner Sharpness (Distortion Check)', fontsize=14)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.test_dir / 'region_comparison.png', dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"  已保存区域对比图: region_comparison.png")
    
    def _generate_text_report(self):
        """生成文本分析报告"""
        report_lines = []
        report_lines.append("="*70)
        report_lines.append("FOV 清晰度测试分析报告")
        report_lines.append(f"测试时间: {self.test_timestamp}")
        report_lines.append("="*70)
        report_lines.append("")
        
        # 分析结果
        report_lines.append("【测试结果摘要】")
        report_lines.append("-"*70)
        
        for r in self.results:
            fov = r['fov']
            lap = r['overall']['laplacian_var']
            sobel = r['overall']['sobel_mag']
            
            report_lines.append(f"\nFOV {fov}° (等效{r['equiv_focal_length_mm']:.1f}mm):")
            report_lines.append(f"  - Laplacian方差: {lap:.2f}")
            report_lines.append(f"  - Sobel边缘强度: {sobel:.2f}")
            
            # 区域对比
            center = r['regions']['center']['laplacian_var']
            corners = [
                r['regions']['top_left']['laplacian_var'],
                r['regions']['top_right']['laplacian_var'],
                r['regions']['bottom_left']['laplacian_var'],
                r['regions']['bottom_right']['laplacian_var']
            ]
            avg_corner = np.mean(corners)
            ratio = center / avg_corner if avg_corner > 0 else float('inf')
            report_lines.append(f"  - 中心/四角清晰度比: {ratio:.2f}")
            
            if ratio > 2.0:
                report_lines.append(f"    ⚠️  中心明显比四角清晰，可能存在严重畸变或景深问题")
            elif ratio > 1.3:
                report_lines.append(f"    ℹ️  中心略比四角清晰，轻微畸变（广角正常）")
            else:
                report_lines.append(f"    ✅ 中心和四角清晰度一致")
        
        # 结论判断
        report_lines.append("\n" + "="*70)
        report_lines.append("【问题诊断】")
        report_lines.append("-"*70)
        
        # 检查是否所有FOV都模糊
        lap_values = [r['overall']['laplacian_var'] for r in self.results]
        if max(lap_values) < 100:  # 阈值可根据实际情况调整
            report_lines.append("\n⚠️  所有FOV的清晰度都很低，可能原因：")
            report_lines.append("   1. Motion Blur 未关闭（检查 settings.json 中的 MotionBlurAmount）")
            report_lines.append("   2. 图像压缩开启（检查 compress=False）")
            report_lines.append("   3. 纹理流送问题（UE4控制台执行 r.MipMap.LodBias -2）")
            report_lines.append("   4. 场景中纹理本身分辨率低")
        else:
            # 检查FOV与清晰度的关系
            sorted_by_fov = sorted(self.results, key=lambda x: x['fov'])
            small_fov_sharp = sorted_by_fov[0]['overall']['laplacian_var']
            large_fov_sharp = sorted_by_fov[-1]['overall']['laplacian_var']
            
            if small_fov_sharp > large_fov_sharp * 2:
                report_lines.append("\nℹ️  小FOV（长焦）比大FOV（广角）清晰很多，可能原因：")
                report_lines.append("   1. 广角透视畸变导致边缘拉伸（正常现象）")
                report_lines.append("   2. 存在景深效果，长焦景深更浅但对焦在远处")
            elif small_fov_sharp < large_fov_sharp * 0.5:
                report_lines.append("\n⚠️  大FOV（广角）比小FOV（长焦）更清晰，这不符合光学规律！")
                report_lines.append("   可能原因：对焦距离设置不当，长焦时失焦")
            else:
                report_lines.append("\n✅ 不同FOV的清晰度差异在合理范围内，没有明显的对焦问题")
        
        report_lines.append("\n" + "="*70)
        report_lines.append("【建议】")
        report_lines.append("-"*70)
        report_lines.append("1. 如果所有FOV都模糊，先检查 MotionBlurAmount=0 和 compress=False")
        report_lines.append("2. 如果只有广角模糊，可能是透视畸变，属于正常现象")
        report_lines.append("3. 如果需要精确对焦控制，考虑改用 CineCameraActor")
        report_lines.append("4. 当前测试使用相机: obstacle_camera_left")
        report_lines.append("="*70)
        
        # 保存报告
        report_text = "\n".join(report_lines)
        report_path = self.test_dir / "analysis_report.txt"
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(report_text)
        
        print(f"  已保存分析报告: analysis_report.txt")
        
        # 同时在控制台打印
        print("\n" + report_text)


if __name__ == "__main__":
    # 运行测试
    tester = FOVSharpnessTester()
    tester.run_test()
