"""
FOV 梯度测试脚本（手动模式）
用于 AirSim API 不支持动态修改 FOV 的情况

使用方法：
1. 修改 settings.json 中的 FOV_Degrees 为第一个测试值（如 10）
2. 重启 AirSim + UE4
3. 运行：python fov_test_manual.py
4. 按提示修改 FOV 并重启，直到测试完所有 FOV 值
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

plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class FOVTestManual:
    """手动FOV测试器"""
    
    def __init__(self, vehicle_name="LunarRover"):
        self.vehicle_name = vehicle_name
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        
        # 测试配置
        self.test_fov_values = [10, 35, 70, 100, 120]
        self.camera_name = "obstacle_camera_left"
        
        # 创建保存目录
        self.test_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.test_dir = Path(f"./fov_test_manual_{self.test_timestamp}")
        self.test_dir.mkdir(parents=True, exist_ok=True)
        
        self.results = []
        
        print("="*70)
        print("FOV 手动测试模式")
        print("="*70)
        print(f"\n将要测试的 FOV 值: {self.test_fov_values}")
        print("\n操作步骤：")
        print("1. 修改 Documents/AirSim/settings.json 中的 FOV_Degrees")
        print("2. 重启 AirSim 和 UE4")
        print("3. 将车辆放置在距离纹理物体 2m 处")
        print("4. 按 Enter 采集当前 FOV 的图像")
        print("="*70)
    
    def measure_sharpness(self, image):
        """测量清晰度"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        else:
            gray = image
        
        # Laplacian 方差
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        
        # Sobel 边缘强度
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        sobel_mag = np.sqrt(sobelx**2 + sobely**2).mean()
        
        # 边缘密度
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.count_nonzero(edges) / (edges.shape[0] * edges.shape[1])
        
        return {
            'laplacian_var': laplacian_var,
            'sobel_mag': sobel_mag,
            'edge_density': edge_density
        }
    
    def capture_image(self):
        """采集单张图像"""
        request = airsim.ImageRequest(
            self.camera_name,
            airsim.ImageType.Scene,
            pixels_as_float=False,
            compress=False
        )
        responses = self.client.simGetImages([request], self.vehicle_name)
        
        if len(responses) == 0 or responses[0].width == 0:
            raise RuntimeError("无法获取图像")
        
        response = responses[0]
        img = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img = img.reshape(response.height, response.width, 3)
        return img
    
    def run_test(self):
        """运行手动测试"""
        for fov in self.test_fov_values:
            print(f"\n{'='*70}")
            print(f"请设置 FOV = {fov}°")
            print(f"1. 修改 settings.json: \"FOV_Degrees\": {fov}")
            print(f"2. 重启 AirSim + UE4")
            print(f"3. 将车辆放在距离纹理 2m 处")
            print(f"4. 按 Enter 采集图像 (或输入 'skip' 跳过，'quit' 退出)")
            
            user_input = input("> ").strip().lower()
            
            if user_input == 'quit':
                print("退出测试")
                break
            if user_input == 'skip':
                print(f"跳过 FOV {fov}°")
                continue
            
            try:
                # 采集多张图像取平均
                print("  正在采集 3 张图像...")
                sharpness_list = []
                
                for i in range(3):
                    img = self.capture_image()
                    time.sleep(0.3)
                    sharpness = self.measure_sharpness(img)
                    sharpness_list.append(sharpness)
                
                # 计算平均清晰度
                avg_sharpness = {
                    k: np.mean([s[k] for s in sharpness_list])
                    for k in sharpness_list[0].keys()
                }
                
                # 保存图像（第一张）
                img_path = self.test_dir / f"fov_{fov:03d}.png"
                cv2.imwrite(str(img_path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                
                # 记录结果
                equiv_focal = 8 * 70 / fov  # 假设 8mm = 70°
                result = {
                    'fov': fov,
                    'equiv_focal_length_mm': equiv_focal,
                    'sharpness': avg_sharpness,
                    'image_path': str(img_path)
                }
                self.results.append(result)
                
                print(f"  ✅ 采集完成")
                print(f"  等效焦距: {equiv_focal:.1f}mm")
                print(f"  Laplacian方差: {avg_sharpness['laplacian_var']:.2f}")
                print(f"  Sobel边缘强度: {avg_sharpness['sobel_mag']:.2f}")
                
            except Exception as e:
                print(f"  ❌ 错误: {e}")
                continue
        
        # 生成报告
        if len(self.results) > 0:
            self.generate_report()
        
        print(f"\n{'='*70}")
        print(f"测试完成！结果保存在: {self.test_dir}")
        print(f"{'='*70}")
    
    def generate_report(self):
        """生成报告"""
        # 保存JSON
        report_path = self.test_dir / "results.json"
        with open(report_path, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        # 绘制图表
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        fig.suptitle('FOV vs Sharpness Analysis (Manual Mode)', fontsize=14)
        
        fovs = [r['fov'] for r in self.results]
        
        # Laplacian
        lap_vars = [r['sharpness']['laplacian_var'] for r in self.results]
        axes[0].plot(fovs, lap_vars, 'o-', linewidth=2, markersize=8)
        axes[0].set_xlabel('FOV (degrees)')
        axes[0].set_ylabel('Laplacian Variance')
        axes[0].set_title('Sharpness (Higher is Better)')
        axes[0].grid(True, alpha=0.3)
        
        # Sobel
        sobel_mags = [r['sharpness']['sobel_mag'] for r in self.results]
        axes[1].plot(fovs, sobel_mags, 'o-', linewidth=2, markersize=8, color='green')
        axes[1].set_xlabel('FOV (degrees)')
        axes[1].set_ylabel('Sobel Magnitude')
        axes[1].set_title('Edge Strength (Higher is Better)')
        axes[1].grid(True, alpha=0.3)
        
        # Edge Density
        edge_dens = [r['sharpness']['edge_density'] for r in self.results]
        axes[2].plot(fovs, edge_dens, 'o-', linewidth=2, markersize=8, color='red')
        axes[2].set_xlabel('FOV (degrees)')
        axes[2].set_ylabel('Edge Density')
        axes[2].set_title('Edge Density (Higher is Better)')
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.test_dir / 'sharpness_plot.png', dpi=150)
        plt.close()
        
        # 文本报告
        with open(self.test_dir / 'report.txt', 'w', encoding='utf-8') as f:
            f.write("FOV 手动测试报告\n")
            f.write("="*70 + "\n\n")
            
            for r in self.results:
                f.write(f"FOV {r['fov']}° (等效{r['equiv_focal_length_mm']:.1f}mm):\n")
                f.write(f"  Laplacian: {r['sharpness']['laplacian_var']:.2f}\n")
                f.write(f"  Sobel: {r['sharpness']['sobel_mag']:.2f}\n")
                f.write(f"  Edge Density: {r['sharpness']['edge_density']:.4f}\n\n")
            
            # 简单分析
            lap_values = [r['sharpness']['laplacian_var'] for r in self.results]
            if max(lap_values) < 100:
                f.write("结论: 所有FOV都较模糊，可能存在运动模糊或压缩问题\n")
            elif max(lap_values) / min(lap_values) > 2:
                f.write("结论: 不同FOV清晰度差异较大，可能是畸变或景深影响\n")
            else:
                f.write("结论: 各FOV清晰度差异不大，无明显对焦问题\n")
        
        print(f"\n  已保存报告到: {self.test_dir}")


if __name__ == "__main__":
    tester = FOVTestManual()
    tester.run_test()
