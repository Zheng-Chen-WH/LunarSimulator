# FOV 梯度测试 Settings.json 配置指南

## 快速测试配置

将以下配置复制到 `Documents/AirSim/settings.json`，每次修改 `FOV_Degrees` 值后重启 AirSim：

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Car",
  "ClockSpeed": 1,
  "CameraDefaults": {
    "CaptureSettings": [
      {
        "ImageType": 0,
        "Width": 840,
        "Height": 840,
        "FOV_Degrees": 70,
        "AutoExposureSpeed": 100,
        "AutoExposureBias": 0,
        "AutoExposureMaxBrightness": 0.64,
        "AutoExposureMinBrightness": 0.03,
        "MotionBlurAmount": 0,
        "TargetGamma": 1.0
      }
    ]
  },
  "Vehicles": {
    "LunarRover": {
      "VehicleType": "PhysXCar",
      "Cameras": {
        "obstacle_camera_left": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 840,
              "Height": 840,
              "FOV_Degrees": 70
            }
          ],
          "X": 0.2,
          "Y": -0.10,
          "Z": -0.3,
          "Pitch": 15,
          "Roll": 0,
          "Yaw": 0
        },
        "obstacle_camera_right": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 840,
              "Height": 840,
              "FOV_Degrees": 70
            }
          ],
          "X": 0.2,
          "Y": 0.10,
          "Z": -0.3,
          "Pitch": 15,
          "Roll": 0,
          "Yaw": 0
        }
      }
    }
  }
}
```

## 测试步骤

### 方法 1：使用自动脚本（推荐）

```bash
python fov_sharpness_test.py
```

如果 AirSim 版本支持 `simSetCameraFov` API，脚本会自动修改 FOV 并采集图像。

### 方法 2：手动测试（如果 API 不支持）

```bash
python fov_test_manual.py
```

按照提示修改 `settings.json` 中的 `FOV_Degrees`，重启 AirSim 后继续测试。

### 方法 3：纯手动操作

| 步骤 | 操作 |
|-----|------|
| 1 | 修改 `settings.json` 中的 `FOV_Degrees` 为测试值（10, 35, 70, 100, 120） |
| 2 | 重启 AirSim 和 UE4 |
| 3 | 将车辆驾驶到距离纹理物体 2m 处 |
| 4 | 使用 `dataset_saver.py` 或 `hello_car.py` 采集单张图像 |
| 5 | 记录图像清晰度（可用 Windows 照片查看器放大观察） |
| 6 | 重复步骤 1-5，直到测试完所有 FOV 值 |

## 结果解读

### 情况 A：所有 FOV 都模糊

**诊断**：不是对焦问题！

**可能原因**：
- `MotionBlurAmount` 未设为 0
- 图像压缩开启（`compress=true`）
- 纹理流送预算不足
- UE4 场景中纹理本身分辨率低

**解决方案**：
```json
"MotionBlurAmount": 0  // 确保是 0
```

UE4 控制台命令：
```
r.Streaming.PoolSize 4096
r.MipMap.LodBias -2
```

### 情况 B：小 FOV（10°-35°）清晰，大 FOV（70°+）模糊

**诊断**：广角透视畸变（正常现象）

**解释**：
- 广角镜头近距离拍摄时，边缘像素被"拉伸"
- 这不是真正的"模糊"，而是几何畸变
- 8mm 镜头 + 2m 距离属于广角近距离，边缘拉伸明显

**解决方案**：
- 如果用于 SLAM/VINS，通常只使用图像中心区域
- 可以在 `config.py` 中调整 `target_fov_h` 和 `target_fov_v` 进行裁剪

### 情况 C：小 FOV 模糊，大 FOV 清晰

**诊断**：可能存在景深/对焦问题

**解释**：
- 长焦（小 FOV）景深浅，对焦不准时更明显
- 广角（大 FOV）景深深，即使对焦不准也相对清晰

**解决方案**：
- 检查 AirSim 是否启用了景深效果（默认不启用）
- 如果需要精确对焦控制，改用 `CineCameraActor`

### 情况 D：所有 FOV 都清晰

**诊断**：当前没有对焦问题

AirSim 默认使用无限景深，所有内容都在焦点内。

## 附加测试：景深验证

如果你想确认 AirSim 是否有景深效果，可以设置以下场景：

1. 放置三个物体：
   - A 在 0.5m（前景）
   - B 在 2m（中景，假设对焦这里）
   - C 在 10m（背景）

2. 用不同 FOV 拍摄

3. 观察：
   - 如果 A、B、C 都清晰 → 没有景深效果（AirSim 默认）
   - 如果只有 B 清晰，A 和 C 模糊 → 有景深效果

## 参考：FOV 与焦距对照表

假设你的镜头是 8mm 焦距，传感器能支持到 120° FOV：

| FOV | 等效焦距 | 视角类型 |
|-----|---------|---------|
| 10° | 56mm | 长焦 |
| 35° | 16mm | 广角 |
| 70° | 8mm | 超广角（你的镜头） |
| 100° | 5.6mm | 鱼眼边缘 |
| 120° | 4.7mm | 全鱼眼 |

## 测试记录表

打印此表格手动记录：

```
| FOV | 等效焦距 | 整体清晰度 | 中心清晰度 | 边缘清晰度 | 备注 |
|-----|---------|-----------|-----------|-----------|------|
| 10° | 56mm    | 1-10分    | 1-10分    | 1-10分    |      |
| 35° | 16mm    | 1-10分    | 1-10分    | 1-10分    |      |
| 70° | 8mm     | 1-10分    | 1-10分    | 1-10分    | 当前设置 |
| 100°| 5.6mm   | 1-10分    | 1-10分    | 1-10分    |      |
| 120°| 4.7mm   | 1-10分    | 1-10分    | 1-10分    |      |
```

## 快速检查清单

测试前确认：
- [ ] `settings.json` 中 `MotionBlurAmount` 设为 0
- [ ] 采集代码中 `compress` 设为 `False`
- [ ] 车辆静止（不要移动）
- [ ] 距离纹理物体约 2m
- [ ] 光线充足（不要太暗）

运行测试：
```bash
# 自动模式
python fov_sharpness_test.py

# 或手动模式
python fov_test_manual.py
```
