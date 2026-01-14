#!/bin/bash
# robot_localization 安装脚本
# 在Docker容器内运行此脚本

echo "============================================"
echo "robot_localization 安装脚本"
echo "============================================"

# 检查是否在容器内
if [ ! -f /.dockerenv ]; then
    echo "[警告] 请在Docker容器内运行此脚本"
    echo "使用: docker exec -it vins_fusion bash"
    echo "然后: bash /root/data/install_robot_localization.sh"
    exit 1
fi

echo ""
echo "[步骤1] 配置ROS软件源..."
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

echo ""
echo "[步骤2] 添加ROS GPG密钥..."
# 尝试多种方法添加密钥
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654 2>/dev/null || \
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 2>/dev/null || \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - 2>/dev/null || \
wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - 2>/dev/null

if [ $? -eq 0 ]; then
    echo "  ✓ GPG密钥添加成功"
else
    echo "  ✗ GPG密钥添加失败，尝试继续..."
fi

echo ""
echo "[步骤3] 更新包列表..."
apt-get update
if [ $? -ne 0 ]; then
    echo "  ✗ 更新失败，请检查网络连接"
    exit 1
fi
echo "  ✓ 包列表更新成功"

echo ""
echo "[步骤4] 安装robot_localization..."
apt-get install -y ros-melodic-robot-localization

if [ $? -eq 0 ]; then
    echo "  ✓ robot_localization安装成功"
else
    echo "  ✗ 安装失败，尝试修复..."
    apt-get install -y --fix-broken
    apt-get install -y ros-melodic-robot-localization
    
    if [ $? -ne 0 ]; then
        echo ""
        echo "============================================"
        echo "安装失败，可能的原因："
        echo "1. 网络问题 - 无法访问ROS软件源"
        echo "2. 磁盘空间不足"
        echo "3. Docker镜像版本不兼容"
        echo ""
        echo "请尝试以下解决方案："
        echo ""
        echo "方案1: 使用国内镜像源"
        echo "  echo 'deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ bionic main' > /etc/apt/sources.list.d/ros-latest.list"
        echo "  apt-get update"
        echo "  apt-get install -y ros-melodic-robot-localization"
        echo ""
        echo "方案2: 从源码编译（见文档）"
        echo ""
        echo "方案3: 使用Python EKF方案（备选）"
        echo "  python fusion_pipeline.py --dataset ./dataset --no-vins"
        echo "============================================"
        exit 1
    fi
fi

echo ""
echo "[步骤5] 安装其他依赖包..."
apt-get install -y ros-melodic-topic-tools
apt-get install -y ros-melodic-tf2-ros

echo ""
echo "[步骤6] 验证安装..."
source /opt/ros/melodic/setup.bash
if rospack find robot_localization > /dev/null 2>&1; then
    INSTALL_PATH=$(rospack find robot_localization)
    echo "  ✓ robot_localization安装成功！"
    echo "  安装路径: $INSTALL_PATH"
    echo ""
    echo "============================================"
    echo "安装完成！现在可以运行融合管道："
    echo "  python ros_fusion_pipeline.py --bag /root/data/dataset.bag"
    echo "============================================"
else
    echo "  ✗ 验证失败，robot_localization未找到"
    exit 1
fi
