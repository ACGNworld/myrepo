#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fc_comm.msg import FCCommand
import random
import math

class GuidanceCommandSimulator(Node):
    def __init__(self):
        super().__init__('guidance_command_simulator')
        
        # 创建发布器
        self.publisher = self.create_publisher(FCCommand, 'guidance_command', 10)
        
        # 设置发布频率(Hz)
        self.publish_frequency = 20.0  # 20Hz
        self.timer = self.create_timer(1.0/self.publish_frequency, self.timer_callback)
        
        # 初始化随机种子
        random.seed()
        
        # 模拟参数
        self.simulation_time = 0.0
        self.base_longitude = 116.3912  # 基准经度(北京附近)
        self.base_latitude = 39.9075    # 基准纬度(北京附近)
        
        self.get_logger().info("Guidance Command Simulator Started")

    def timer_callback(self):
        """定时生成并发布模拟控制指令"""
        msg = FCCommand()
        
        # 更新时间
        self.simulation_time += 1.0/self.publish_frequency
        
        # 随机生成控制模式(0-3)
        # msg.control_mode = random.randint(0, 3)
        msg.control_mode = 1
        
        # 生成随机偏移量(-100到100cm)
        msg.x_offset = random.randint(-100, 100)
        msg.y_offset = random.randint(-100, 100)
        msg.z_offset = random.randint(-100, 100)
        
        # 随机最大速度(100-500 cm/s)
        msg.max_speed = random.randint(100, 500)
        
        # 随机航向角(0-359度)
        msg.heading = random.randint(0, 359)
        
        # 在基准位置附近生成随机目标位置(±0.001度 ≈ ±111米)
        msg.target_longitude = self.base_longitude + random.uniform(-0.001, 0.001)
        msg.target_latitude = self.base_latitude + random.uniform(-0.001, 0.001)
        
        # 随机高度(50-500米)
        msg.target_height = random.randint(50, 500)
        
        # 随机目标航向(0-359度)
        msg.target_heading = random.randint(0, 359)
        
        # 随机目标速度(100-300 cm/s)
        msg.target_speed = random.randint(100, 300)
        
        # 随机最大角速度(10-50度/秒)
        msg.max_angular_speed = random.randint(10, 50)
        
        # 发布消息
        self.publisher.publish(msg)
        
        # 记录日志(降低频率避免刷屏)
        if self.simulation_time % 5.0 < 1.0/self.publish_frequency:
            self.get_logger().info(f"Published command: mode={msg.control_mode}, "
                                 f"target=({msg.target_longitude:.6f}, {msg.target_latitude:.6f})")

def main(args=None):
    rclpy.init(args=args)
    node = GuidanceCommandSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()