# VINS-Fusion Docker 使用指南

本文档提供从打开终端到运行 VINS-Fusion 的完整操作流程。

---

## 1. 启动 Docker 容器

### 1.1 打开终端
```bash
# 在 Ubuntu 中右键打开终端，或按 Ctrl+Alt+T
```

### 1.2 启动 VINS-Fusion 容器（基础模式）
```bash
# 启动容器（无 GUI 支持）
docker run -it \
    --name vins_fusion \
    --network host \
    -v ~/data:/root/data \
    vins_fusion:melodic \
    /bin/bash
```

### 1.3 启动 VINS-Fusion 容器（带 GUI 支持 - 推荐）
```bash
# 允许 X 服务器连接
xhost +local:docker

# 启动容器（支持 RViz 可视化）
docker run -it \
    --name vins_fusion \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/data:/root/data \
    vins_fusion:melodic \
    /bin/bash
```

成功后，提示符会变为：
```
root@MEE2440:~/catkin_ws#
```

---

## 2. 验证安装

### 2.1 检查工作空间
```bash
# 查看工作空间结构
ls -la

# 查看 VINS-Fusion 源码
ls src/VINS-Fusion/

# 输出应包含：
# camera_models  config  docker  global_fusion  loop_fusion  vins_estimator
```

### 2.2 加载 ROS 环境
```bash
source devel/setup.bash

# 验证 VINS 包
rospack find vins
# 输出：/root/catkin_ws/src/VINS-Fusion/vins_estimator
```

---

## 3. 准备数据集

### 3.1 下载 EuRoC 数据集（示例）

**在主机上操作**（打开新终端）：

#### 方法 1：使用 wget（标准方法）
```bash
# 创建数据目录
mkdir -p ~/data/euroc

# 进入数据目录
cd ~/data/euroc

数据集链接：https://www.research-collection.ethz.ch/handle/20.500.11850/690084

### 3.2 验证数据挂载

**在容器内操作**：
```bash
# 查看挂载的数据目录
ls /root/data/euroc/

# 应该能看到下载的 .bag 文件
```

---

## 4. 运行 VINS-Fusion

### 4.1 方法一：使用三个终端窗口（推荐）

#### 终端 1：启动 ROS Master
```bash
# 在容器内（已经进入）
roscore
```

保持此终端运行。

#### 终端2：录制输出话题（先启动这个）
```bash
docker exec -it vins_fusion /bin/bash
source /root/catkin_ws/devel/setup.bash

# 录制轨迹话题（两个话题用空格分隔）
rosbag record -O ~/vins_output.bag /vins_estimator/odometry /vins_estimator/path
```
**注意**：rosbag record会等待话题出现，这是正常的。启动后继续执行终端3和终端4。

**然后**在其他终端运行 VINS 和播放数据，完成后：
```bash
# 停止录制（按 Ctrl+C）
# 查看录制的文件
rosbag info ~/vins_output.bag
# 退出容器
exit
# 复制到主机
docker cp vins_fusion:/root/vins_output.bag ~/data/
```

<!-- #### 终端 2：启动 RViz 可视化（可选）

**在主机上打开新终端**：
```bash
# 进入同一个运行中的容器
docker exec -it vins_fusion /bin/bash

# 加载环境
source /root/catkin_ws/devel/setup.bash

# 启动 RViz
roslaunch vins vins_rviz.launch
``` -->

#### 终端 3：运行 VINS 估计器

**在主机上再打开一个新终端**：
```bash
# 进入容器
docker exec -it vins_fusion /bin/bash

# 加载环境
source /root/catkin_ws/devel/setup.bash

# 运行 VINS 节点（双目+IMU 配置）
rosrun vins vins_node /root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
```

或使用单目+IMU 配置：
```bash
rosrun vins vins_node /root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml
```

#### 终端 4：配置EKF
```bash
docker exec -it vins_fusion bash
# 安装EKF（假如没安装
apt-get update
apt-get install -y ros-melodic-robot-localization ros-melodic-topic-tools
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash

# 启动EKF
roslaunch /root/data/ekf_only.launch
```
**在主机上再打开一个终端**：
```bash
# 进入容器
docker exec -it vins_fusion /bin/bash

# 加载环境
source /root/catkin_ws/devel/setup.bash

# 播放 rosbag
rosbag play /root/data/euroc/V2_01_easy.bag
```


#### 终端 5：播放数据集（可选）

**在主机上再打开一个终端**：
```bash
# 进入容器
docker exec -it vins_fusion /bin/bash

# 加载环境
source /root/catkin_ws/devel/setup.bash

# 播放 rosbag
rosbag play /root/data/euroc/V2_01_easy.bag
```

---

### 4.2 方法二：使用一体化启动文件

#### 启动完整系统（包括可视化）
```bash
# 在容器内
source /root/catkin_ws/devel/setup.bash

# 启动 VINS + RViz
roslaunch vins euroc.launch

# 在另一个终端播放数据
docker exec -it vins_fusion /bin/bash
source /root/catkin_ws/devel/setup.bash
rosbag play /root/data/euroc/MH_01_easy.bag
```

---

## 5. 配置文件说明

### 5.1 可用配置文件路径
```
/root/catkin_ws/src/VINS-Fusion/config/
├── euroc/
│   ├── euroc_mono_imu_config.yaml          # 单目+IMU
│   ├── euroc_stereo_imu_config.yaml        # 双目+IMU
│   └── euroc_stereo_config.yaml            # 双目（无IMU）
├── realsense_d435i/
│   └── realsense_stereo_imu_config.yaml    # RealSense D435i
└── vi_car/
    └── vi_car.yaml                          # 自定义配置
```

### 5.2 关键配置参数

编辑配置文件：
```bash
nano /root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
```

主要参数：
- `imu_topic`: IMU 数据话题
- `image0_topic`: 左相机话题
- `image1_topic`: 右相机话题（双目）
- `output_path`: 输出结果路径
- `estimate_extrinsic`: 是否估计外参（0/1/2）
- `max_solver_time`: 最大求解时间（秒）

---

## 6. 输出结果

### 6.1 查看输出文件
```bash
# VINS 会将结果保存到配置文件指定的路径
# 默认在 ~/.ros/ 目录
# 在第三个终端中查看

ls -lh ~/.ros/

# 常见输出文件：
# vins_result_loop.csv     - 带回环的轨迹
# vins_result_no_loop.csv  - 不带回环的轨迹
# vio.csv                  - VIO 轨迹
```

### 6.2 复制结果到主机
```bash
# 在主机上执行
docker cp vins_fusion:/root/.ros/vins_result_loop.csv ~/data/
```

---

## 7. 常用 Docker 命令

### 7.1 容器管理
```bash
# 查看运行中的容器
docker ps

# 查看所有容器（包括停止的）
docker ps -a

# 停止容器
docker stop vins_fusion

# 启动已停止的容器
docker start vins_fusion

# 进入运行中的容器
docker exec -it vins_fusion /bin/bash

# 删除容器
docker rm vins_fusion

# 强制删除运行中的容器
docker rm -f vins_fusion

# 在容器中退出容器
exit
```

### 7.2 镜像管理
```bash
# 查看镜像
docker images

# 删除镜像
docker rmi vins_fusion:melodic

# 清理未使用的资源
docker system prune -a
```

### 7.3 数据传输
```bash
# 从容器复制到主机
docker cp vins_fusion:/root/catkin_ws/results.txt ~/data/

# 从主机复制到容器
docker cp ~/data/dataset.bag vins_fusion:/root/data/
```

---

## 8. 故障排除

### 8.1 RViz 无法显示

**问题**：RViz 窗口无法打开或显示错误

**解决**：
```bash
# 在主机上执行
xhost +local:docker

# 重启容器时添加 X11 支持
docker rm -f vins_fusion
docker run -it \
    --name vins_fusion \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/data:/root/data \
    vins_fusion:melodic \
    /bin/bash
```

### 8.2 找不到 ROS 包

**问题**：`rospack find vins` 报错

**解决**：
```bash
# 重新加载环境
source /opt/ros/melodic/setup.bash
source /root/catkin_ws/devel/setup.bash

# 或添加到 .bashrc（永久生效）
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 8.3 容器名称冲突

**问题**：`docker: Error response from daemon: Conflict. The container name "/vins_fusion" is already in use`

**解决**：
```bash
# 删除旧容器
docker rm vins_fusion

# 或使用不同名称
docker run -it --name vins_fusion_new ... vins_fusion:melodic /bin/bash
```

### 8.4 IMU 数据不同步

**问题**：VINS 无法初始化或跟踪失败

**解决**：
- 检查 IMU 话题频率：`rostopic hz /imu0`
- 确认时间戳对齐
- 调整 `td` 参数（时间偏移）

### 8.5 内存不足

**问题**：容器运行缓慢或崩溃

**解决**：
```bash
# 限制容器内存
docker run -it \
    --name vins_fusion \
    --memory="8g" \
    --memory-swap="8g" \
    ... 其他参数 ...
```

### 8.6 RViz 无法连接到 ROS Master

**问题**：`Could not contact ROS master at [http://localhost:11311]`

**原因**：roscore 未运行或网络配置问题

**解决**：
```bash
# 在任一容器终端检查roscore是否运行
rostopic list

# 如果报错，说明roscore未启动，返回终端1检查
# 如果能看到话题列表，说明roscore正常

# 检查ROS_MASTER_URI环境变量
echo $ROS_MASTER_URI
# 应该输出：http://localhost:11311 或 http://MEE2440:11311
```

### 8.7 RViz OpenGL 渲染错误

**问题**：`OGRE EXCEPTION: Unable to create a suitable GLXContext`

**原因**：容器内OpenGL渲染环境不完整

**解决**：
```bash
# 方案1：安装软件渲染库（在容器内）
apt-get update
apt-get install -y libgl1-mesa-dri libgl1-mesa-glx mesa-utils

export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# 方案2：跳过RViz，仅运行算法（推荐）
# 按照第11节的流程，跳过终端2，直接运行终端3和终端4
# VINS算法仍会正常运行并输出结果到~/.ros/目录
```

### 8.8 数据集下载失败或超时

**问题**：`Connection timed out` 或下载速度极慢

**原因**：网络问题或服务器限制

**解决方案：**

#### 方案1：使用断点续传
```bash
# wget 断点续传
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag

# curl 断点续传
curl -L -O -C - http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
```

#### 方案2：使用多线程下载
```bash
# 安装 axel
sudo apt-get install axel

# 使用 10 个线程下载
axel -n 10 http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
```

#### 方案3：下载较小的数据集
```bash
# V1_01_easy 只有 1.4GB
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag

# V2_01_easy 只有 1.2GB
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.bag
```

#### 方案4：使用已有数据集
如果已经从其他地方获得了数据集：
```bash
# 将数据集移动到正确位置
mkdir -p ~/data/euroc
cp /path/to/your/MH_01_easy.bag ~/data/euroc/

# 或使用 USB 设备
cp /media/usb/MH_01_easy.bag ~/data/euroc/
```

#### 方案5：验证已下载文件
```bash
# 检查文件大小（MH_01_easy 应该约 2GB）
ls -lh ~/data/euroc/MH_01_easy.bag

# 验证是否为有效的 rosbag
rosbag info ~/data/euroc/MH_01_easy.bag
```

---

## 9. 使用自定义数据集

### 9.1 准备 rosbag

确保你的 rosbag 包含以下话题：
```bash
# 查看 bag 信息
rosbag info your_dataset.bag

# 必需话题：
# - /imu0 (sensor_msgs/Imu)
# - /cam0/image_raw (sensor_msgs/Image)  # 单目
# - /cam1/image_raw (sensor_msgs/Image)  # 双目（可选）
```

### 9.2 创建配置文件

复制现有配置：
```bash
cd /root/catkin_ws/src/VINS-Fusion/config
cp -r euroc my_robot

# 编辑配置
nano my_robot/my_robot_config.yaml
```

修改关键参数：
- 话题名称
- 相机内参
- IMU 噪声参数
- 相机-IMU 外参

### 9.3 运行自定义配置
```bash
rosrun vins vins_node /root/catkin_ws/src/VINS-Fusion/config/my_robot/my_robot_config.yaml
```

---

## 10. 性能优化

### 10.1 调整线程数
```bash
# 在配置文件中设置
max_solver_time: 0.04  # 减少求解时间
max_num_iterations: 8  # 减少迭代次数
```

### 10.2 降低图像分辨率
```yaml
# 在配置文件中
image_width: 752  # 降低到 376
image_height: 480 # 降低到 240
```

### 10.3 减少特征点数量
```yaml
max_cnt: 150  # 降低到 100 或 80
```

---

## 11. 完整操作流程示例

### 单次运行完整命令序列

```bash
# ============ 主机操作 ============
# 1. 允许 X11 转发
xhost +local:docker

# 2. 启动容器（带 GUI）
docker run -it \
    --name vins_fusion \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/data:/root/data \
    vins_fusion:melodic \
    /bin/bash

# 2.5 如果已有名称重复容器：
docker rm vins_fusion

# ============ 容器内操作 - 终端1 ============
# 此时你应该已经在容器内（提示符：root@MEE2440:~/catkin_ws#）
roscore

# 保持此终端运行，不要关闭！

# ============ 主机 - 打开新终端2（Ctrl+Alt+T 或右键新建终端）============
# 注意：这是在主机上打开的新终端，不是容器内
# 如果没有权限需要在docker命令前加sudo

# 进入同一个运行中的容器
docker exec -it vins_fusion /bin/bash

# 现在你在容器内，执行以下命令（逐行执行）：
source /root/catkin_ws/devel/setup.bash

# 验证能否连接到roscore（可选）
rostopic list
# 应该能看到 /rosout 等话题，说明roscore正在运行

# 方案A：尝试启动RViz（可能遇到OpenGL错误）
roslaunch vins vins_rviz.launch

# 如果遇到OpenGL错误，按Ctrl+C停止，使用方案B

# 方案B：跳过RViz，直接运行算法（推荐）
# 如果RViz启动失败，跳过此终端，直接进行终端3的操作

# ============ 主机 - 打开新终端3 ============
docker exec -it vins_fusion /bin/bash
source /root/catkin_ws/devel/setup.bash

# 运行VINS算法节点（双目+IMU）
rosrun vins vins_node /root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml

# 等待看到 "waiting for image and imu..." 提示

# ============ 主机 - 打开新终端4 ============
docker exec -it vins_fusion /bin/bash
source /root/catkin_ws/devel/setup.bash

# 播放数据集（确保数据集已下载到~/data/euroc/目录）
rosbag play /root/data/euroc/MH_01_easy.bag

# 此时终端3应该开始输出VINS处理信息

# 运行完成后，在主机上复制结果
docker cp vins_fusion:/root/.ros/vins_result_loop.csv ~/data/

# 退出容器
exit

# 在主机停止容器
docker stop vins_fusion
```

---

## 12. 快速启动脚本

创建一个启动脚本方便使用：

```bash
# 在主机上创建脚本
nano ~/start_vins.sh
```

脚本内容：
```bash
#!/bin/bash

# 允许 X11
xhost +local:docker

# 检查容器是否存在
if [ "$(docker ps -aq -f name=vins_fusion)" ]; then
    echo "容器已存在，正在启动..."
    docker start vins_fusion
    docker exec -it vins_fusion /bin/bash
else
    echo "创建新容器..."
    docker run -it \
        --name vins_fusion \
        --network host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v ~/data:/root/data \
        vins_fusion:melodic \
        /bin/bash
fi
```

赋予执行权限并使用：
```bash
chmod +x ~/start_vins.sh
~/start_vins.sh
```

---

## 13. 参考资源

- **VINS-Fusion GitHub**: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- **EuRoC 数据集**: http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- **ROS Wiki**: http://wiki.ros.org/vins_estimator
- **论文**: VINS-Fusion: An Optimization-Based Multi-Sensor State Estimator

---

## 附录：常见数据集链接

### EuRoC MAV Dataset
```bash
# Machine Hall (室内)
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_02_easy/MH_02_easy.bag

# Vicon Room (小空间)
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.bag
```

### 其他数据集
- **KITTI**: http://www.cvlibs.net/datasets/kitti/
- **TUM VI**: https://vision.in.tum.de/data/datasets/visual-inertial-dataset
- **UZH-FPV**: http://rpg.ifi.uzh.ch/uzh-fpv.html

---

**祝您使用愉快！如有问题，请查看故障排除章节或访问 GitHub Issues。**
