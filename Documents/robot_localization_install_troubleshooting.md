# robot_localization 安装故障排除指南

## 问题：Unable to locate package ros-melodic-robot-localization

### 原因分析

1. **ROS软件源未配置** - Docker镜像中可能没有配置ROS软件源
2. **GPG密钥缺失** - 无法验证ROS软件包
3. **网络问题** - 无法访问ROS官方软件源
4. **包列表未更新** - apt缓存过期

---

## 解决方案

### 方案1：使用安装脚本（推荐）

```bash
# 在主机上，将脚本复制到容器
docker cp install_robot_localization.sh vins_fusion:/root/

# 进入容器
docker exec -it vins_fusion bash

# 运行安装脚本
cd /root
chmod +x install_robot_localization.sh
./install_robot_localization.sh
```

### 方案2：手动逐步安装

```bash
# 进入容器
docker exec -it vins_fusion bash

# 1. 添加ROS软件源
echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# 2. 添加ROS GPG密钥（尝试多种方法）
# 方法A：使用apt-key
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# 如果方法A失败，尝试方法B
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654

# 如果方法B也失败，尝试方法C
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# 3. 更新包列表
apt-get update

# 4. 安装robot_localization
apt-get install -y ros-melodic-robot-localization

# 5. 验证安装
source /opt/ros/melodic/setup.bash
rospack find robot_localization
```

### 方案3：使用国内镜像源（中国用户）

如果访问官方源速度慢或超时：

```bash
# 进入容器
docker exec -it vins_fusion bash

# 使用清华大学镜像
echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ bionic main" > /etc/apt/sources.list.d/ros-latest.list

# 添加密钥
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

# 更新并安装
apt-get update
apt-get install -y ros-melodic-robot-localization
```

其他国内镜像：
- 中科大：`http://mirrors.ustc.edu.cn/ros/ubuntu/`
- 阿里云：`http://mirrors.aliyun.com/ros/ubuntu/`

### 方案4：从源码编译

如果包管理器方式都失败，可以从源码编译：

```bash
# 进入容器
docker exec -it vins_fusion bash

# 进入工作空间
cd /root/catkin_ws/src

# 克隆robot_localization源码
git clone https://github.com/cra-ros-pkg/robot_localization.git

# 或使用gitee镜像（国内用户）
git clone https://gitee.com/mirrors/robot_localization.git

# 切换到melodic分支
cd robot_localization
git checkout melodic-devel

# 安装依赖
cd /root/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
catkin_make

# 更新环境变量
source devel/setup.bash

# 验证
rospack find robot_localization
```

### 方案5：使用Python EKF方案（备选）

如果以上方案都无法解决，使用我们的Python EKF实现：

```bash
# 在Windows主机上运行
python fusion_pipeline.py --dataset ./dataset/20260113_164908
```

优点：
- 不依赖ROS
- 纯Python实现
- 功能完整

---

## 验证安装

### 检查robot_localization是否安装成功

```bash
# 方法1：使用rospack
source /opt/ros/melodic/setup.bash
rospack find robot_localization

# 应该输出类似：
# /opt/ros/melodic/share/robot_localization

# 方法2：检查可执行文件
which ekf_localization_node

# 应该输出类似：
# /opt/ros/melodic/lib/robot_localization/ekf_localization_node

# 方法3：查看包信息
rospack depends robot_localization
```

### 测试运行

```bash
# 启动roscore
roscore &

# 等待几秒
sleep 3

# 测试启动EKF节点
rosrun robot_localization ekf_localization_node

# 如果没有报错，按Ctrl+C停止
```

---

## 常见错误及解决

### 错误1：GPG error: NO_PUBKEY

```
W: GPG error: http://packages.ros.org/ros/ubuntu bionic InRelease: 
The following signatures couldn't be verified because the public key is not available: 
NO_PUBKEY F42ED6FBAB17C654
```

**解决**：
```bash
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
apt-get update
```

### 错误2：Failed to fetch

```
Err:1 http://packages.ros.org/ros/ubuntu bionic InRelease
  Could not connect to packages.ros.org:80
```

**解决**：
1. 检查网络连接
2. 使用国内镜像源（方案3）
3. 配置代理（如果需要）

### 错误3：依赖问题

```
The following packages have unmet dependencies:
 ros-melodic-robot-localization : Depends: ros-melodic-geographic-msgs but it is not going to be installed
```

**解决**：
```bash
apt-get install -y --fix-broken
apt-get install -y ros-melodic-robot-localization
```

---

## 诊断命令

```bash
# 检查ROS版本
rosversion -d

# 检查软件源配置
cat /etc/apt/sources.list.d/ros-latest.list

# 搜索可用的robot_localization包
apt-cache search robot-localization

# 查看包信息
apt-cache show ros-melodic-robot-localization

# 检查网络连接
ping -c 3 packages.ros.org

# 检查磁盘空间
df -h
```

---

## 联系支持

如果以上方案都无法解决问题，请提供以下信息：

1. Docker镜像版本
   ```bash
   docker images | grep vins_fusion
   ```

2. ROS版本
   ```bash
   docker exec -it vins_fusion rosversion -d
   ```

3. 错误日志
   ```bash
   apt-get install -y ros-melodic-robot-localization 2>&1 | tee install_error.log
   ```

4. 网络诊断
   ```bash
   ping -c 3 packages.ros.org
   curl -I http://packages.ros.org/ros/ubuntu/
   ```

---

## 替代方案总结

| 方案 | 难度 | 推荐度 | 说明 |
|------|-----|--------|------|
| 方案1：安装脚本 | ⭐ | ⭐⭐⭐⭐⭐ | 自动化，最简单 |
| 方案2：手动安装 | ⭐⭐ | ⭐⭐⭐⭐ | 了解每一步 |
| 方案3：国内镜像 | ⭐ | ⭐⭐⭐⭐⭐ | 网络问题首选 |
| 方案4：源码编译 | ⭐⭐⭐⭐ | ⭐⭐⭐ | 需要时间 |
| 方案5：Python EKF | ⭐ | ⭐⭐⭐⭐ | 完全不依赖ROS |

建议顺序：方案1 → 方案3 → 方案5
