# 环境配置与安装
## 1. 虚拟机配置
在windows系统中：     
1. 控制面板-程序和功能-启用或关闭Windows功能-开启：
    - 适用于linux的windows子系统
    - Hyper-V
    - 虚拟机平台
2. 以管理员身份启动powershell：
```bash
# 先设置WSL2为默认版本
wsl --set-default-version 2
# 更新
wsl update
# 再安装Ubuntu 20.04
wsl --install -d Ubuntu-20.04
# 验证安装
wsl -l -v
# 输出示例：
#   NAME            STATE           VERSION
# * Ubuntu-20.04    Stopped         2
```

## 2. VINS-Fusion在WSL原生安装（适配20.04）
打开WSL（企鹅头）：

### 安装ROS
```bash
# 1. 设置软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. 设置密钥
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 3. 安装 ROS Noetic 桌面完整版 (包含 Rviz 等工具)
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# 4. 配置环境变量
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. 安装构建工具
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

# 6. 初始化 rosdep
sudo rosdep init
rosdep update
```

### 安装VINS-Fusion

```bash
# 安装 Ceres Solver 和相关数学库
sudo apt install libceres-dev libeigen3-dev -y
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# 克隆适配 ROS Noetic 的代码：
# 这里使用社区维护的适配 OpenCV 4 的版本，可以避免大量的报错。
git clone https://github.com/cyp4x141/VINS-Fusion-noetic-Opencv4.git
# 重命名文件夹以符合习惯 (可选)
mv VINS-Fusion-noetic-Opencv4 VINS-Fusion
# 编译，等待100%
cd ~/catkin_ws
catkin_make
# 刷新环境
source ~/catkin_ws/devel/setup.bash
```

## 3. 安装 robot_localization (EKF融合)
针对低速月球车场景，我们需要引入轮速计和星敏感器来消除VIO的尺度漂移。

```bash
# 直接安装二进制包
sudo apt install ros-noetic-robot-localization -y
```

# 使用教程
## 移动/查看虚拟机文件

1. **在 WSL 的终端里查看/移动文件**   
```bash
# 将windows主机的文件的复制到 WSL 主目录，例如：
cp /mnt/c/Users/zchenkf/Downloads/V1_01_easy.bag ~/
# 弹出一个 Windows 资源管理器窗口，显示当前的 Linux 目录
explorer.exe .
```

2. **通过地址栏访问**
```bash
# 在任何 Windows 文件夹的地址栏里输入以下路径并回车：
\\wsl$
# 或者具体发行版：
\\wsl$\Ubuntu-20.04
# 这样可以像访问网络共享文件夹一样访问你的 Linux 系统根目录。home 目录通常在：
\\wsl$\Ubuntu-20.04\home\dendrobium
```

3. **直接通过文件管理器访问**
文件管理器侧边栏最底下有个Linux，点进去就是WSL根目录     

4. **VS Code集成**
```bash
# 可以在 WSL 终端里输入以下命令，直接用 VS Code 打开当前文件夹进行编辑
code .
```
或者直接在vscode窗口左侧一排列表的“远程资源管理器”，点开直接就有WSL


## 启动VINS-Fusion（OpenCV4版本）+EKF

```bash
# 第零个终端运行roscore
roscore

# 第一个终端运行rviz：
source ~/catkin_ws/devel/setup.bash
# 强制刷新 ROS 的包索引（这步是关键！）
rospack profile
rosparam set use_sim_time true
roslaunch vins vins_rviz.launch

# 第二个终端播放数据（数据集为例）：
source ~/catkin_ws/devel/setup.bash
rospack profile
rosparam set use_sim_time true
# 运行 VINS 节点，参数是数据集的路径，发布时钟
rosbag play --clock --pause xxx.bag
# 再启动VINS和EKF，然后按空格开始播放

# 第三个终端启动VINS
source ~/catkin_ws/devel/setup.bash
rospack profile
# 所有ROS节点等待bag包时间戳。
rosparam set use_sim_time true
# 运行 VINS 节点，参数是配置文件的路径【记得修改！】
rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/LunarSimulator/VIO.yaml

# 第四个终端启动核心启动 EKF (融合轮速计和星敏感器)
source ~/catkin_ws/devel/setup.bash
rosparam set use_sim_time true
# 启动launch文件
# roslaunch 的标准格式是：roslaunch <包名(package_name)> <启动文件名(.launch)>
# 在 ROS 系统中，~/catkin_ws/src/VINS-Fusion文件夹对应的包名被定义为 vins
# 【重要】绝对路径启动方式（推荐，最稳妥）：
# 直接指定 .launch 文件的完整路径，不需要关心它在哪个包的哪个目录下
roslaunch ~/catkin_ws/src/VINS-Fusion/config/LunarSimulator/ekf_lunar_run.launch

# 第五个终端收集数据
source ~/catkin_ws/devel/setup.bash
rospack profile
rosparam set use_sim_time true
# 指定端口、文件保存位置进行录制；这里需要先手动创建output文件夹；这里是录制vins的结果
rosbag record -O ~/output/vins_result.bag /vins_estimator/odometry /vins_estimator/path /vins_estimator/key_poses
# 这里是录制EKF的结果(EKF 输出的/odometry/filtered)和VINS 原始数据进行对比
rosbag record -O ~/output/fusion_result.bag /vins_estimator/odometry /vins_estimator/path /vins_estimator/key_poses /odometry/filtered /ground_truth/pose
# 结束录制
ctrl+C
# 查看录制结果
rosbag info ~/output/fusion_result.bag

# 第六个终端检查话题数据（可选）
source ~/catkin_ws/devel/setup.bash
rospack profile
# 检查list
rostopic list
# 检查数据频率（imu0为例）
rostopic hz /imu0
# 接受端口全部传出信息（EKF输出为例，包含了协方差矩阵）
rostopic echo /odometry/filtered
# 屏蔽协方差
rostopic echo /odometry/filtered --noarr
```

## 后处理
### rviz可视化分析
1. **track_image两张图像**
+ 左图：时域追踪(Temporal Tracking)
  + 画面内容：左眼相机(cam0)拍摄的实时画面。
  + 红色点代表能够被追踪的特征点，VINS 正在通过光流法（Optical Flow）不断锁定这些点。
    + 如果红点很密，说明月球表面纹理丰富，利于定位。
    + 如果红点在快速移动，说明相机在转动或移动。
    + 作用：主要负责计算 “我相对于上一时刻移动了多少”（旋转 + 平移的方向），也就是提供连续的运动估计。
+ 右图：空域匹配(Stereo Matching)
+ 画面内容：右眼相机(cam1)拍摄的实时画面。
  + 绿色点代表成功与左图红点配对的特征点，VINS在右眼中寻找左眼看到的同一个特征点。
    + 只有变成了绿色的点，才能通过双目原理（三角测量）算出深度。
    + 作用：VINS 获取“绝对尺度”的唯一来源。

### 使用evo进行真值与仿真结果的对齐
1. **安装evo**
```bash
# 1. 确保你有 pip (Ubuntu 默认可能没有带 pip)
sudo apt update
sudo apt install python3-pip -y

# 2. 安装 evo，no-binary evo 是为了避免一些 numpy 版本依赖冲突，是官方推荐的安装方式
pip3 install evo --upgrade --no-binary evo

# 如果找不到命令，把安装的文件夹添加到PATH:
export PATH=$PATH:/home/dendrobium/.local/bin
# 执行以下命令，把配置写入 .bashrc 文件
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc
source ~/.bashrc

# 3. 安装完成后验证
evo_ape --help
```
2. **使用evo**
```bash
# 用 evo 自带工具从 bag 包里提取标准tum格式（输出逻辑定死）
evo_traj bag ~/output/fusion_result.bag /vins_estimator/odometry --save_as_tum
evo_traj bag ~/output/fusion_result.bag /odometry/filtered --save_as_tum
evo_traj bag ~/output/fusion_result.bag /ground_truth/pose --save_as_tum
# -vas: 对齐原点(a)和尺度(s)；
# 评估 VINS (会有尺度漂移，需要 -s 对齐才能看清形状)
evo_ape tum ground_truth_pose.tum vins_estimator_odometry.tum -vas --plot --save_plot vins_drift.png
# 评估 EKF (应该不需要尺度对齐，直接 -va)
evo_ape tum ground_truth_pose.tum odometry_filtered.tum -va --plot --save_plot ekf_fusion.png

# 如果需要仅分析xoy平面内投影精度：
# 使用 awk 将 TUM 文件的第 4 列（Z轴）强制清零，生成平底轨迹
awk '{$4=0; print $0}' ground_truth_pose.tum > gt_xy.tum
awk '{$4=0; print $0}' odometry_filtered.tum > odom_xy.tum
# 然后分析这两个平面的轨迹
evo_ape tum gt_xy.tum odom_xy.tum -va --plot --save_plot ekf_fusion_xy.png
```
**EVO结果解释：**
+ **map图像**
  + **灰色虚线：**真值轨迹，AirSim 的真实路径。
  + **彩色实线 (从蓝色到红色)：**VINS 估计轨迹，颜色代表误差 (APE) 的大小。
+ **APE (Absolute Pose Error, 绝对位姿误差) 分析图表**
  + **灰线 (APE (m))：**瞬时误差曲线。它代表在每一个时间戳上，估算的轨迹点与真值轨迹点之间的欧氏距离误差。
  + **蓝线 (RMSE):**均方根误差 (Root Mean Square Error)：评估轨迹精度的最常用指标，它对大误差（异常值）比较敏感。
  + **红线 (mean):**平均误差 (Mean Error)，整个轨迹误差的算术平均值。
  + **绿线 (median):**将所有误差从小到大排序，位于中间的那个值。如果系统中偶尔有巨大的跳变（Outliers），RMSE 和 Mean 会被拉高，但 Median 相对不受影响，更能代表系统“大多数时候”的表现。
  + **紫色阴影区域 (std):**标准差 (Standard Deviation)，以红线（均值）为中心，上下覆盖的区域通常表示误差的分布范围（Mean±Std）
+ **EVO对齐：**Sim(3) 变换（包含旋转、平移和尺度）将VINS轨迹对齐到了真值轨迹，这通常用于单目VIO系统，因为单目系统往往存在尺度漂移或尺度不确定的问题，需要通过对齐来评估其轨迹形状的准确性。

## VINS-Fusio调参
通过yaml实现，目前的config在`/home/dendrobium/catkin_ws/src/VINS-Fusion/config/LunarSimulator`
+ 包括一个整体的`VIO.yaml`和分别控制两个相机参数的`cam0.yaml`和`cam1.yaml`
+ `imu_topic`, `image0_topic`, `image1_topic`改成符合bag话题的
+ `cam0_calib`和`cam1_calib`指向两个自建的相机文件.yaml
+ **已经编写了`generate_vins_config.py`，直接运行即可自动生成yaml配置文件**

## EKF多传感器融合 (VINS + Wheel + StarTracker)
通过`config.yaml`和`.launch`实现，同样放在`~/catkin_ws/src/VINS-Fusion/config/LunarSimulator/`
+ 配置文件为`ekf_lunar_config.yaml`，启动文件为`ekf_lunar_run.launch`。
+ **编写了`generate_ekf_config.py`，直接运行即可自动生成yaml配置文件**
