#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
import struct
from fc_comm.msg import FCCommand, FCStatus

DATA_LENGTH=0x1B

def calculate_checksum(data_bytes):
    """
    计算校验和(简单累加和,取低8位)
    :param data_bytes: 要计算的数据字节序列
    :return: 校验和字节
    """
    return sum(data_bytes) & 0xFF

class FCCommNode(Node):
    def __init__(self):
        super().__init__('fc_comm_node')
        # 串口初始化
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=0.02  # 读取超时时间
        )
        # 存储最新的控制指令
        self.latest_command = FCCommand()
        self.command_lock = threading.Lock()
        
        # 订阅控制指令话题
        self.subscription = self.create_subscription(
            FCCommand,
            'guidance_command',
            self.command_callback,
            10
        )
        
        # 发布飞控状态消息
        self.publisher = self.create_publisher(FCStatus, 'fc_status', 10)
        
        # 创建定时器(20Hz发送周期)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 50ms = 20Hz
        
        # 启动串口接收线程
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_serial_data)
        self.receive_thread.start()
        
        self.get_logger().info("FC Communication Node Started")

    def command_callback(self, msg):
        """存储最新的控制指令"""
        with self.command_lock:
            self.latest_command = msg

    def timer_callback(self):
        """定时发送控制指令给飞控"""
        with self.command_lock:
            cmd = self.latest_command
        
        # 打包消息
        try:
            data = self.pack_guidance_message(cmd)
            self.ser.write(data)
            # self.get_logger().info(f"Send:{cmd}")
        except Exception as e:
            self.get_logger().error(f"Packing error: {str(e)}")

    def pack_guidance_message(self, cmd):   #TODO: 打包的不对
        """
        打包导引->飞控消息(0x01指令)
        :param cmd: FCCommand消息
        :return: 打包后的字节序列
        """
        # 创建字节缓冲区
        buf = bytearray()
        
        # 添加固定头部
        buf.extend([0xA5, 0x5A, 0xAA, 0x01, DATA_LENGTH])  # 同步头+来源+类型+数据长度(26字节)
        
        # 添加数据部分
        buf.append(cmd.control_mode)  # 控制模式
        
        # 添加所有带符号的2字节字段(小端序)
        buf.extend(struct.pack('<h', cmd.x_offset))   # X轴偏移
        buf.extend(struct.pack('<h', cmd.y_offset))   # Y轴偏移
        buf.extend(struct.pack('<h', cmd.z_offset))   # Z轴偏移
        
        # 添加所有无符号的2字节字段(小端序)
        buf.extend(struct.pack('<H', cmd.max_speed))   # 最大速度
        buf.extend(struct.pack('<H', cmd.heading))     # 航向角
        
        # 处理经度/纬度(转换为协议要求的无符号32位)
        buf.extend(struct.pack('<i', int(cmd.target_longitude*1e7)))
        buf.extend(struct.pack('<i', int(cmd.target_latitude*1e7)))
        
        # 添加高度/航向/速度字段(小端序)
        buf.extend(struct.pack('<H', cmd.target_height))          # 目标高度
        buf.extend(struct.pack('<H', cmd.target_heading))        # 目标航向角
        buf.extend(struct.pack('<H', cmd.target_speed))          # 目标速度
        buf.extend(struct.pack('<H', cmd.max_angular_speed))    # 最大角速度限制
        
        # 计算并添加校验和
        checksum = calculate_checksum(buf)
        buf.append(checksum)
        
        # 添加结束符
        buf.append(0xFF)
        return bytes(buf)

    def parse_fc_message(self, data):
        """
        解析飞控->导引消息(0x01指令)
        :param data: 包含完整消息的字节序列(27字节)
        :return: FCStatus消息对象
        """
        status = FCStatus()
        
        # 解析固定字段（索引从0开始）
        status.control_mode = data[5]  # 控制模式
        status.battery = data[6]       # 电池电量
        
        # 解析坐标（转换为有符号浮点数）
        status.longitude = struct.unpack('<i', data[7:11])[0] / 1e7  # 经度（索引7-10）
        status.latitude = struct.unpack('<i', data[11:15])[0] / 1e7  # 纬度（索引11-14）
        
        # 解析其他字段（小端序）
        status.height = struct.unpack('<h', data[15:17])[0]           # 高度（索引15-16）
        status.speed = struct.unpack('<H', data[17:19])[0] * 0.01    # 速度（索引17-18）
        status.roll = struct.unpack('<h', data[19:21])[0] * 0.01     # 滚转角（索引19-20）
        status.pitch = struct.unpack('<h', data[21:23])[0] * 0.01    # 俯仰角（索引21-22）
        status.yaw = struct.unpack('<h', data[23:25])[0] * 0.01      # 偏航角（索引23-24）
        
        return status

    def receive_serial_data(self):
        """串口数据接收线程"""
        buffer = bytearray()
        while self.running and rclpy.ok():
            # 读取串口数据
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                continue
                
            buffer.extend(data)
            
            # 处理缓冲区中的所有完整帧
            while len(buffer) >= 27:  # 飞控返回消息固定为27字节
                # 查找帧头 (0xA5 0x5A)
                start_idx = -1
                for i in range(len(buffer) - 1):
                    if buffer[i] == 0xA5 and buffer[i+1] == 0x5A:
                        start_idx = i
                        break
                        
                if start_idx == -1:  # 没有找到帧头
                    buffer.clear()
                    break
                    
                # 检查是否包含完整帧
                if len(buffer) < start_idx + 27:
                    buffer = buffer[start_idx:]  # 保留剩余数据
                    break
                    
                # 提取完整帧
                frame = buffer[start_idx:start_idx+27]
                
                # 验证帧尾
                if frame[-1] != 0xFF:
                    buffer = buffer[start_idx+2:]  # 跳过错误帧头
                    continue
                    
                # 验证校验和
                computed_checksum = calculate_checksum(frame[:-2])  # 排除校验和和结束符
                if computed_checksum != frame[-2]:
                    buffer = buffer[start_idx+2:]  # 跳过错误帧头
                    self.get_logger().warn("Checksum mismatch")
                    continue
                    
                # 解析有效帧
                try:
                    if frame[2] == 0xBB and frame[3] == 0x01:  # 验证来源和指令类型
                        status_msg = self.parse_fc_message(frame)
                        self.publisher.publish(status_msg)
                        self.get_logger().info(str(status_msg))
                except Exception as e:
                    self.get_logger().error(f"Parsing error: {str(e)}")
                    
                # 移除已处理帧
                buffer = buffer[start_idx+27:]
                
    def destroy_node(self):
        self.running = False
        if self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FCCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
