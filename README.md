# ROS2飞控导航控制节点

## 📌 功能概述
实现ROS2与飞控系统的串口通信，包含：
- 20Hz周期发送控制指令（导引→飞控）
- 10Hz周期接收状态反馈（飞控→导引）
- 支持距离模式/悬停模式/GPS模式

## 🛠️ 安装依赖
ROS2 (推荐Humble或Foxy)
>sudo apt install ros-$ROS_DISTRO-serial-driver

>pip install pyserial

## 🚀 快速开始
1. **构建项目**
bash
cd ~/fc-guide-ros2
colcon build --symlink-install
source install/setup.bash

2. **运行节点**
bash
ros2 run fc_comm fc_comm_node
ros2 run fc_comm guidance_simulator


## 📡 通信协议
### 消息格式
- **导引→飞控** (`FCCommand`)
  yaml
  control_mode: 1       # 1=距离模式, 2=悬停模式
  x_offset: 100         # X轴偏移(cm)
  target_longitude: 116.404  # 目标经度(度)


- **飞控→导引** (`FCStatus`)
  yaml
  battery: 80          # 电量(%)
  longitude: 116.403   # 当前经度(度)
  roll: 15.2           # 横滚角(度)


### 串口配置
python
port: '/dev/ttyS0'
baudrate: 115200
timeout: 0.02  # 20ms


## 🔧 测试指令
发送测试命令：
bash
ros2 topic pub /guidance_command fc_comm/msg/FCCommand "
{
  control_mode: 1,
  x_offset: 100,
  target_longitude: 116.404
}" -1


查看飞控状态：
bash
ros2 topic echo /fc_status


## 🛠️ 硬件连接
| 飞控引脚 | 主机接口 |
|----------|----------|
| TX       | RX (Pin2)|
| RX       | TX (Pin3)|
| GND      | GND      |

## 📜 协议文档
完整协议见：[PROTOCOL.md](docs/PROTOCOL.md)

## 💡 常见问题
### Q1: 权限不足
bash
sudo usermod -a -G dialout $USER
newgrp dialout


### Q2: 消息未识别
bash
重新加载环境

source ~/fc_comm_ws/install/setup.bash
ros2 interface list | grep FCCommand
