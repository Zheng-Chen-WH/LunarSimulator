# UE4 纹理质量问题修复指南

## 问题描述
**"圆形放大之后是多边形"** — 这是典型的纹理分辨率不足或过滤设置不当的表现。

## 快速诊断

### 1. 确认问题类型

在 UE4 编辑器中打开场景：

1. **靠近地面**（2m 距离）观察
2. **按 ` 键** 打开控制台
3. 输入 `stat streaming` 查看纹理流送状态

如果看到 `Over Budget` 或 `Waiting For IO`，说明是 **Texture Streaming** 问题。

### 2. 强制最高质量纹理

在控制台输入：

```
r.Streaming.FramesForFullUpdate 1
r.Streaming.MaxTempMemoryAllowed 2048
r.Streaming.PoolSize 8192
r.Streaming.ForceAllFullyLoad 1
r.MipMap.LodBias -10
r.MaxAnisotropy 16
```

然后观察地面纹理是否变清晰。

---

## 常见原因与修复

### 原因 1: 纹理本身分辨率太低

**检查方法**：
1. 在 Content Browser 中找到地面材质使用的纹理
2. 双击打开，查看 Details 面板中的 **"Imported Size"**

**标准**：
- 月球地面纹理至少应 **2048x2048**
- 推荐 **4096x4096** 或 **8192x8192**
- 如果显示 512x512 或 1024x1024，**必须更换纹理**

**修复**：
- 联系场景美术，更换高分辨率纹理
- 或使用 Quixel Megascans 的高清月球/岩石纹理

---

### 原因 2: Mipmap 设置不当

Mipmap 是纹理的逐级缩小版本，用于远距离显示。但如果设置不当，近距离也会加载低分辨率版本。

**检查方法**：
1. 双击地面纹理打开
2. 查看 Details 面板中的 **MIP Gen Settings**

**修复**：
```
MIP Gen Settings: FromTextureGroup
Texture Group: World
LODBias: -1 或 -2 (强制加载更高一级Mipmap)
```

或者在控制台临时禁用 Mipmap：
```
r.MipMap.LodBias -10
```

---

### 原因 3: 纹理过滤模式错误

**问题**：使用了 **Nearest Neighbor**（最近邻）过滤，导致放大时出现块状。

**修复**：
1. 双击地面纹理
2. 在 Details 面板中找到 **Filter**
3. 设置为 **Default (from texture group)** 或 **Trilinear**

**推荐设置**：
```
Filter: Default
Texture Group: World (or WorldNormalMap for normal maps)
```

---

### 原因 4: 材质中的 UV 缩放问题

如果材质中 UV 被过度放大，会导致纹理被拉伸，看起来模糊。

**检查方法**：
1. 双击地面材质
2. 查找 Texture Coordinate 节点
3. 检查 UTiling 和 VTiling

**修复**：
- 确保 UTiling/VTiling 不会过度放大纹理
- 如果地面很大，使用 **World Position Based UV** 而非 Object UV

---

### 原因 5: Texture Streaming 预算不足

AirSim 运行时，UE4 会根据 VRAM 预算动态调整纹理分辨率。

**修复 settings.json**：
```json
{
  "CameraDefaults": {
    "CaptureSettings": [{
      "ImageType": 0,
      "Width": 840,
      "Height": 840,
      "FOV_Degrees": 70,
      "MotionBlurAmount": 0
    }]
  }
}
```

**修复 UE4 项目设置**：
1. Edit → Project Settings → Engine - Rendering
2. 找到 Texture Streaming
3. 将 **Pool Size** 设置为 **4000** 或更高（根据你的显卡显存）

---

### 原因 6: 抗锯齿导致的模糊

如果使用了 **TAA（Temporal Anti-Aliasing）**，在静止画面时可能会过度模糊。

**检查与修复**：

在控制台输入：
```
r.DefaultFeature.AntiAliasing 2  (确保是TAA)
r.TemporalAA.HistoryScreenPercentage 200  (提高TAA历史缓冲区分辨率)
r.TemporalAA.Upsampling 1
r.TemporalAA.Algorithm 1
```

或者切换到 **MSAA**（如果显卡支持）：
```
r.DefaultFeature.AntiAliasing 1
r.MSAACount 4
```

---

## 推荐配置（一键复制）

在 UE4 控制台输入：

```
# 1. 纹理流送（强制最高质量）
r.Streaming.FramesForFullUpdate 1
r.Streaming.MaxTempMemoryAllowed 2048
r.Streaming.PoolSize 8192
r.Streaming.ForceAllFullyLoad 1
r.Streaming.LimitPoolSizeToVRAM 0

# 2. Mipmap（强制最高级别）
r.MipMap.LodBias -10
r.MaxAnisotropy 16

# 3. 纹理过滤
r.TextureStreaming.UsePerTextureBias 0
r.TextureStreaming.MipBias -2

# 4. 抗锯齿优化
r.TemporalAA.HistoryScreenPercentage 200
r.TemporalAA.Upsampling 1
r.TemporalAA.Algorithm 1

# 5. 显示纹理流送状态（确认生效）
stat streaming
```

---

## 如果以上都无效

### 方案 A: 使用 Detail Texturing（细节纹理）

在材质中添加近距离细节：

1. 打开地面材质
2. 添加 **Detail Normal Map** 和 **Detail Diffuse**
3. 使用 **Distance Blend** 节点，在近距离显示细节纹理
4. 远距离淡出，保持性能

### 方案 B: 更换月球场景

如果当前场景纹理质量本身就不高：

- 使用 **Moon Environment** 插件的高质量月球场景
- 或使用 **Brushify** 的 Moon/Lunar 场景包
- 或使用 **Quixel Megascans** 的 Moon Surface 材质

### 方案 C: 程序化生成细节

在材质中使用 **Procedural Noise** 增加近距离细节：

1. 打开地面材质
2. 添加 **Noise** 节点
3. 使用 **World Position** 作为 UV
4. 在近距离叠加 Noise，增加表面细节

---

## 验证修复效果

修改后，运行测试脚本验证：

```bash
python texture_quality_check.py
```

如果清晰度提升，说明修复有效。

---

## 关键参数总结

| 问题 | 关键参数 | 推荐值 |
|-----|---------|-------|
| 纹理分辨率低 | 纹理 Imported Size | ≥4096x4096 |
| Mipmap 问题 | r.MipMap.LodBias | -10 |
| 过滤问题 | Filter | Trilinear |
| Texture Streaming | r.Streaming.PoolSize | 8192 |
| 抗锯齿模糊 | r.TemporalAA.HistoryScreenPercentage | 200 |

---

## 联系美术/场景制作

如果以上都无法解决，需要联系场景美术：

1. **提供证据**：截图显示"圆形变多边形"
2. **明确要求**：
   - 地面纹理分辨率 ≥4096x4096
   - 使用无损格式（PNG/TGA）
   - 正确设置 Mipmap 和 Filter
3. **替代方案**：使用 Quixel Megascans 高清纹理
