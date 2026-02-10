"""
纹理质量诊断脚本
用于检查 UE4 场景中的纹理质量和过滤设置

使用方法：
1. 确保 AirSim 和 UE4 场景已启动
2. 在 UE4 编辑器中打开场景（非运行模式）
3. 运行此脚本获取诊断建议
4. 根据建议调整 UE4 纹理设置
"""

import airsim
import numpy as np
import cv2
import time
from pathlib import Path
from datetime import datetime


class TextureQualityChecker:
    """纹理质量检查器"""
    
    def __init__(self, vehicle_name="LunarRover"):
        self.vehicle_name = vehicle_name
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        
        # 创建保存目录
        self.test_dir = Path(f"./texture_check_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        self.test_dir.mkdir(parents=True, exist_ok=True)
        
        print("="*70)
        print("UE4 纹理质量诊断")
        print("="*70)
    
    def capture_test_images(self):
        """采集测试图像"""
        print("\n【步骤 1】采集测试图像...")
        print("请确保车辆静止在距离地面纹理 2m 处")
        input("按 Enter 开始采集...")
        
        # 采集 obstacle_camera_left 的图像
        request = airsim.ImageRequest(
            "obstacle_camera_left",
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
        
        # 保存原始图像
        cv2.imwrite(str(self.test_dir / "original.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        
        print(f"  图像分辨率: {img.shape[1]}x{img.shape[0]}")
        return img
    
    def analyze_texture_quality(self, img):
        """分析纹理质量"""
        print("\n【步骤 2】分析纹理质量...")
        
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        h, w = gray.shape
        
        # 提取不同尺度的区域进行分析
        regions = {
            'center': gray[h//3:2*h//3, w//3:2*w//3],
            'top_left': gray[:h//2, :w//2],
            'top_right': gray[:h//2, w//2:],
            'bottom_left': gray[h//2:, :w//2],
            'bottom_right': gray[h//2:, w//2:]
        }
        
        results = {}
        for name, region in regions.items():
            # 1. 计算局部方差（纹理丰富度）
            local_var = np.var(region)
            
            # 2. 计算梯度方向一致性（判断是否为人工/低质量纹理）
            sobelx = cv2.Sobel(region, cv2.CV_64F, 1, 0, ksize=3)
            sobely = cv2.Sobel(region, cv2.CV_64F, 0, 1, ksize=3)
            gradient_direction = np.arctan2(sobely, sobelx)
            direction_consistency = 1 - np.std(gradient_direction) / (np.pi / 2)
            
            # 3. 检测块状伪影（DCT系数分析）
            blocky_score = self._detect_blockiness(region)
            
            # 4. 频谱分析
            fft = np.fft.fft2(region)
            fft_shift = np.fft.fftshift(fft)
            magnitude = np.abs(fft_shift)
            
            # 计算高频成分比例
            cy, cx = region.shape[0] // 2, region.shape[1] // 2
            radius = min(cy, cx) // 4
            y, x = np.ogrid[:region.shape[0], :region.shape[1]]
            mask_high = ((x - cx)**2 + (y - cy)**2) > radius**2
            high_freq_ratio = np.sum(magnitude[mask_high]) / np.sum(magnitude)
            
            results[name] = {
                'local_var': local_var,
                'direction_consistency': direction_consistency,
                'blocky_score': blocky_score,
                'high_freq_ratio': high_freq_ratio
            }
        
        return results
    
    def _detect_blockiness(self, img, block_size=8):
        """检测块状伪影（JPEG压缩或低质量纹理特征）"""
        h, w = img.shape
        blocky_scores = []
        
        # 检查水平和垂直方向的块边界
        for i in range(0, h - block_size, block_size):
            for j in range(0, w - block_size, block_size):
                # 水平边界
                if i + block_size < h:
                    diff_h = np.abs(img[i + block_size - 1, j:j+block_size].astype(float) - 
                                   img[i + block_size, j:j+block_size].astype(float))
                    blocky_scores.append(np.mean(diff_h))
                
                # 垂直边界
                if j + block_size < w:
                    diff_v = np.abs(img[i:i+block_size, j + block_size - 1].astype(float) - 
                                   img[i:i+block_size, j + block_size].astype(float))
                    blocky_scores.append(np.mean(diff_v))
        
        return np.mean(blocky_scores) if blocky_scores else 0
    
    def generate_heatmap(self, img):
        """生成纹理质量热力图"""
        print("  生成纹理质量热力图...")
        
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        h, w = gray.shape
        
        # 使用滑动窗口计算局部清晰度
        window_size = 64
        heatmap = np.zeros((h // window_size, w // window_size))
        
        for i in range(0, h - window_size, window_size):
            for j in range(0, w - window_size, window_size):
                window = gray[i:i+window_size, j:j+window_size]
                laplacian_var = cv2.Laplacian(window, cv2.CV_64F).var()
                heatmap[i // window_size, j // window_size] = laplacian_var
        
        # 归一化并上色
        heatmap_norm = cv2.normalize(heatmap, None, 0, 255, cv2.NORM_MINMAX)
        heatmap_colored = cv2.applyColorMap(heatmap_norm.astype(np.uint8), cv2.COLORMAP_JET)
        
        # resize 到原图大小
        heatmap_resized = cv2.resize(heatmap_colored, (w, h), interpolation=cv2.INTER_LINEAR)
        
        # 叠加到原图
        overlay = cv2.addWeighted(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), 0.6, heatmap_resized, 0.4, 0)
        
        cv2.imwrite(str(self.test_dir / "texture_quality_heatmap.png"), heatmap_resized)
        cv2.imwrite(str(self.test_dir / "texture_quality_overlay.png"), overlay)
        
        print(f"  热力图已保存到: {self.test_dir / 'texture_quality_heatmap.png'}")
        
        return heatmap
    
    def print_diagnosis(self, results, img):
        """打印诊断报告"""
        print("\n" + "="*70)
        print("【诊断报告】")
        print("="*70)
        
        # 分析各区域
        print("\n1. 区域纹理质量分析:")
        print("-"*70)
        for name, data in results.items():
            print(f"\n{name}:")
            print(f"  局部方差: {data['local_var']:.2f} (越高越丰富)")
            print(f"  块状伪影评分: {data['blocky_score']:.2f} (越低越好)")
            print(f"  高频比例: {data['high_freq_ratio']:.4f} (越高细节越多)")
        
        # 判断纹理类型
        avg_blocky = np.mean([d['blocky_score'] for d in results.values()])
        avg_high_freq = np.mean([d['high_freq_ratio'] for d in results.values()])
        
        print("\n2. 纹理质量判断:")
        print("-"*70)
        
        if avg_blocky > 5.0:
            print("⚠️  检测到明显块状伪影，可能是:")
            print("   - 纹理分辨率过低")
            print("   - JPEG 压缩质量差")
            print("   - Mipmap 过滤设置不当")
        else:
            print("✅ 块状伪影不明显")
        
        if avg_high_freq < 0.1:
            print("⚠️  高频细节较少，可能是:")
            print("   - 纹理本身分辨率低")
            print("   - 使用了过度模糊的材料")
        else:
            print("✅ 高频细节丰富")
        
        # 检查是否存在质量差异大的区域
        variances = [d['local_var'] for d in results.values()]
        if max(variances) / min(variances) > 3:
            print("⚠️  不同区域纹理质量差异大，可能是:")
            print("   - Texture Streaming 未完全加载")
            print("   - 远处纹理分辨率更低")
        
        print("\n" + "="*70)
        print("【UE4 优化建议】")
        print("="*70)
        
        print("\n在 UE4 控制台输入以下命令（按 ~ 键打开控制台）:")
        print("-"*70)
        
        print("\n1. 强制加载最高分辨率纹理:")
        print("   r.Streaming.FramesForFullUpdate 1")
        print("   r.Streaming.MaxTempMemoryAllowed 1024")
        print("   r.Streaming.PoolSize 8192")
        print("   r.Streaming.ForceAllFullyLoad 1")
        
        print("\n2. 禁用 Mipmap 自动调整（强制最高质量）:")
        print("   r.MipMap.LodBias -10")
        print("   r.MaxAnisotropy 16")
        
        print("\n3. 改善纹理过滤质量:")
        print("   r.TextureStreaming.UsePerTextureBias 0")
        print("   r.TextureStreaming.MipBias -2")
        
        print("\n4. 检查纹理本身分辨率:")
        print("   在 Content Browser 中找到地面纹理")
        print("   查看 Details 面板中的 'Imported Size'")
        print("   建议地面纹理至少 2048x2048 或 4096x4096")
        
        print("\n5. 如果是运行时模糊，检查抗锯齿设置:")
        print("   r.DefaultFeature.AntiAliasing 2  (TAA)")
        print("   r.TemporalAA.Algorithm 1")
        print("   r.TemporalAA.Upsampling 1")
        print("   r.TemporalAA.HistoryScreenPercentage 200")
        
        print("\n6. 检查材质设置:")
        print("   - 打开地面材质")
        print("   - 确保 Texture Sample 节点的 'MipValueMode' 是 'Default'")
        print("   - 检查 'Sampler Source' 是 'From texture asset'")
        
        print("\n" + "="*70)
        print("【如果以上都无效】")
        print("="*70)
        print("\n可能需要更换更高分辨率的月球地面纹理:")
        print("- 推荐分辨率: 4096x4096 或 8192x8192")
        print("- 推荐格式: PNG 无损或 TGA")
        print("- 避免过度压缩的 JPG")
        print("\n或使用细节纹理（Detail Texturing）:")
        print("- 在材质中添加 Detail Normal Map")
        print("- 使用 Detail Diffuse 增加近距离细节")
    
    def run_check(self):
        """运行完整检查"""
        # 采集图像
        img = self.capture_test_images()
        
        # 分析纹理质量
        results = self.analyze_texture_quality(img)
        
        # 生成热力图
        self.generate_heatmap(img)
        
        # 打印诊断报告
        self.print_diagnosis(results, img)
        
        print(f"\n检查完成！结果保存在: {self.test_dir}")
        print("="*70)


if __name__ == "__main__":
    checker = TextureQualityChecker()
    checker.run_check()
