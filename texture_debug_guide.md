# 图像油画/涂抹感问题排查指南

## 问题确认

即使关闭了抗锯齿，图像仍有油画感，可能原因：

1. **TAA 未完全关闭** - 全局设置可能未应用到 SceneCapture
2. **纹理流送/质量问题** - 纹理本身分辨率低或加载了低质量 Mipmap
3. **渲染分辨率未生效** - settings.json 的 1260x1260 可能没有正确应用

## 排查步骤

### 步骤 1：验证渲染分辨率

在 UE4 控制台输入：
```
stat streaming
```

查看输出中是否有纹理流送问题。

### 步骤 2：强制禁用所有抗锯齿

在 UE4 控制台输入以下命令（按顺序）：
```
r.DefaultFeature.AntiAliasing 0
r.PostProcessAAQuality 0
r.TemporalAAQuality 0
r.TemporalAACurrentFrameWeight 1
r.TemporalAAHistoryScreenPercentage 100
showflag.postprocessing 0
```

然后截图测试，看油画感是否消失。

### 步骤 3：验证采集分辨率

在 Python 中打印实际采集的分辨率：
```python
import airsim
client = airsim.CarClient()
client.confirmConnection()

request = airsim.ImageRequest("obstacle_camera_left", airsim.ImageType.Scene, False, False)
responses = client.simGetImages([request])

if responses:
    print(f"实际采集分辨率: {responses[0].width} x {responses[0].height}")
```

如果输出是 840x840 而不是 1260x1260，说明 settings.json 的修改没有生效。

### 步骤 4：检查纹理本身质量

在 UE4 编辑器中：
1. 找到地面材质使用的纹理
2. 双击打开，查看 Details 面板中的 "Imported Size"
3. 如果显示 1024x1024 或更低，说明纹理本身分辨率不足

## 可能的解决方案

### 方案 A：提高纹理分辨率（如果纹理本身质量差）

联系场景美术，更换更高分辨率的月球地面纹理（≥4096x4096）。

### 方案 B：强制加载最高质量 Mipmap

在 UE4 控制台：
```
r.MipMap.LodBias -10
r.Streaming.ForceAllFullyLoad 1
```

### 方案 C：使用细节纹理（Detail Texturing）

在材质中添加近距离细节纹理，增加表面细节。

### 方案 D：接受现状，优化算法

如果以上都无效，考虑：
- 调整特征点提取算法的阈值
- 使用对模糊更鲁棒的特征点检测器（如 SIFT 替代 ORB）
- 增加图像对比度预处理

## 快速验证

在 Python 中测试不同的图像预处理：

```python
import cv2
import numpy as np

# 读取图像
img = cv2.imread("captured_image.png")

# 方法 1：增加锐化
kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
img_sharpened = cv2.filter2D(img, -1, kernel)

# 方法 2：增加对比度
alpha = 1.5  # 对比度增益
beta = 0     # 亮度偏移
img_contrast = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

# 方法 3：双边滤波（去噪但保留边缘）
img_bilateral = cv2.bilateralFilter(img, 9, 75, 75)

# 保存对比
cv2.imwrite("original.png", img)
cv2.imwrite("sharpened.png", img_sharpened)
cv2.imwrite("contrast.png", img_contrast)
cv2.imwrite("bilateral.png", img_bilateral)
```

看看哪种处理对 VINS/SLAM 的效果最好。

## 结论

如果关闭抗锯齿后仍有油画感，**最可能的原因是纹理本身的分辨率或 Mipmap 设置问题**，而不是渲染设置问题。

建议优先排查：
1. 实际采集分辨率是否为 1260x1260
2. 纹理本身的 Imported Size 是否 ≥4096
3. Mipmap 是否加载了低分辨率版本
