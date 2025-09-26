import socket
import time
import argparse
import threading
import sys
import binascii
from datetime import datetime
import select

class ArduPilotTCPBridge:
    def __init__(self, ip, port, data=None, rate=1.0, repeat=False, log_file=None):
        self.tcp_ip = ip
        self.tcp_port = port
        self.raw_data = data
        self.rate = rate
        self.repeat = repeat
        self.interval = 1.0 / rate if rate > 0 else 0
        self.log_file = log_file
        self.running = False
        self.sock = None
        self.threads = []
        self.lock = threading.Lock()
        self.last_activity = time.time()
        self.connection_active = False
        
    def connect(self):
        """连接到TCP服务器"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)  # 连接超时
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.sock.settimeout(0.5)  # 接收超时
            print(f"已连接到 {self.tcp_ip}:{self.tcp_port}")
            self.connection_active = True
            return True
        except ConnectionRefusedError:
            print(f"连接被拒绝，请确保ArduPilot SITL正在监听 {self.tcp_ip}:{self.tcp_port}")
        except socket.timeout:
            print(f"连接超时，无法连接到 {self.tcp_ip}:{self.tcp_port}")
        except Exception as e:
            print(f"连接错误: {e}")
        return False
    
    def send_data(self, data=None):
        """发送原始数据"""
        if data is None:
            data = self.raw_data
            
        if not data:
            return False
            
        try:
            with self.lock:
                self.sock.sendall(data)
                self.last_activity = time.time()
                
            hex_data = binascii.hexlify(data).decode('utf-8').upper()
            ascii_repr = self.format_ascii(data)
            print(f"[发送] {len(data)} 字节")
            print(f"HEX: {hex_data}")
            print(f"ASCII: {ascii_repr}")
            self.log_data(f"SEND: {hex_data}")
            return True
        except (BrokenPipeError, ConnectionResetError):
            print("连接已断开")
            self.connection_active = False
        except Exception as e:
            print(f"发送错误: {e}")
        return False
    
    def format_ascii(self, data):
        """格式化数据为ASCII表示，非打印字符用点号代替"""
        return ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data])
    
    def receive_data(self):
        """接收数据"""
        try:
            data = self.sock.recv(4096)
            if data:
                with self.lock:
                    self.last_activity = time.time()
                    
                hex_data = binascii.hexlify(data).decode('utf-8').upper()
                # ascii_repr = self.format_ascii(data)
                print(f"[接收] {len(data)} 字节")
                print(f"HEX: {hex_data}")
                # print(f"ASCII: {ascii_repr}")
                self.log_data(f"RECV: {hex_data}")
                return data
            elif data == b'':  # 空数据表示连接关闭
                print("连接已关闭")
                self.connection_active = False
            return None
        except socket.timeout:
            return None  # 超时是正常的
        except (ConnectionResetError, BrokenPipeError):
            print("连接已断开")
            self.connection_active = False
        except Exception as e:
            print(f"接收错误: {e}")
        return None
    
    def log_data(self, message):
        """记录数据到日志文件"""
        if self.log_file:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            try:
                with open(self.log_file, "a") as f:
                    f.write(f"[{timestamp}] {message}\n")
            except Exception as e:
                print(f"日志记录错误: {e}")
    
    def start_receiver(self):
        """启动接收线程"""
        print("开始接收数据...")
        while self.running and self.connection_active:
            self.receive_data()
        print("接收线程退出")
    
    def start_auto_sender(self):
        """启动自动发送线程"""
        if not self.raw_data:
            return
            
        print(f"开始自动发送数据 (长度: {len(self.raw_data)} 字节)")
        
        try:
            # 单次发送模式
            if not self.repeat:
                if self.send_data():
                    print("数据发送完成")
                return
            
            # 循环发送模式
            while self.running and self.connection_active:
                start_time = time.time()
                
                if not self.send_data():
                    break
                
                # 计算并补偿时间误差
                elapsed = time.time() - start_time
                sleep_time = max(0, self.interval - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n发送被用户中断")
        print("自动发送线程退出")

    def interactive_sender(self):
        """交互式发送模式"""
        print("进入交互式发送模式 (输入十六进制数据或命令)")
        print("命令: exit, quit, status, help")
        
        while self.running and self.connection_active:
            try:
                user_input = input("> ").strip()
                if not user_input:
                    continue
                    
                if user_input.lower() in ['exit', 'quit']:
                    break
                    
                if user_input.lower() == 'status':
                    self.show_status()
                    continue
                    
                if user_input.lower() == 'help':
                    print("命令:")
                    print("  exit/quit - 退出交互模式")
                    print("  status    - 显示连接状态")
                    print("  help      - 显示帮助")
                    print("十六进制数据格式示例:")
                    print("  B5 62 01 07 3C 00")
                    print("  B56201073C00")
                    continue
                    
                data = parse_hex_data(user_input)
                if data:
                    self.send_data(data)
                    
            except KeyboardInterrupt:
                print("\n退出交互模式")
                break
            except Exception as e:
                print(f"错误: {e}")
        print("交互式发送模式退出")
    
    def show_status(self):
        """显示当前连接状态"""
        status = "连接状态: "
        status += "活跃" if self.connection_active else "断开"
        
        if self.connection_active:
            idle_time = time.time() - self.last_activity
            status += f", 空闲时间: {idle_time:.1f}秒"
            
        print(status)
    
    def connection_monitor(self):
        """监控连接状态"""
        while self.running:
            time.sleep(1)
            if self.connection_active:
                # 检查连接是否仍然活跃
                try:
                    # 使用select检查socket是否可读（有数据或关闭）
                    r, _, _ = select.select([self.sock], [], [], 0)
                    if r:
                        # 如果有数据可读，但recv返回空数据，说明连接关闭
                        if self.sock.recv(1, socket.MSG_PEEK) == b'':
                            self.connection_active = False
                            print("检测到连接已关闭")
                except:
                    self.connection_active = False
                    print("检测到连接异常")
    
    def start(self):
        """启动桥接器"""
        if not self.connect():
            return
        
        self.running = True
        
        # 启动连接监控线程
        monitor_thread = threading.Thread(target=self.connection_monitor, daemon=True)
        monitor_thread.start()
        self.threads.append(monitor_thread)
        
        # 启动接收线程
        receiver_thread = threading.Thread(target=self.start_receiver, daemon=True)
        receiver_thread.start()
        self.threads.append(receiver_thread)
        
        # 如果有自动发送数据，启动自动发送线程
        if self.raw_data:
            sender_thread = threading.Thread(target=self.start_auto_sender, daemon=True)
            sender_thread.start()
            self.threads.append(sender_thread)
        else:
            # 否则进入交互式发送模式
            self.interactive_sender()
        
        # 保持主线程运行
        try:
            while any(t.is_alive() for t in self.threads):
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        finally:
            self.stop()
    
    def stop(self):
        """停止并关闭连接"""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            print("TCP连接已关闭")
        
        # 等待所有线程结束
        for t in self.threads:
            t.join(timeout=1.0)
        print("程序已停止")

def parse_hex_data(hex_str):
    """将十六进制字符串转换为字节数据"""
    try:
        # 移除所有空格和0x前缀
        cleaned = hex_str.replace(" ", "").replace("0x", "").replace(",", "").replace(":", "")
        if not cleaned:
            return None
            
        # 检查长度是否为偶数
        if len(cleaned) % 2 != 0:
            print("警告: 十六进制字符串长度应为偶数，自动补零")
            cleaned = '0' + cleaned
            
        return bytes.fromhex(cleaned)
    except ValueError as e:
        print(f"错误: 无效的十六进制数据: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(
        description='ArduPilot TCP 双向数据工具',
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('--ip', default='127.0.0.1', help='TCP服务器IP (默认: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9998, help='TCP端口 (默认: 9998)')
    parser.add_argument('--send', metavar='HEX_DATA', 
                        help='要发送的原始数据 (十六进制格式)\n示例: "B5 62 01 07 3C 00"')
    parser.add_argument('--rate', type=float, default=1.0, 
                        help='发送速率 (Hz) (仅重复模式有效, 默认: 1.0)')
    parser.add_argument('--repeat', action='store_true', 
                        help='重复发送数据 (默认: 单次发送)')
    parser.add_argument('--log', metavar='FILE', 
                        help='记录所有收发数据到文件')
    
    args = parser.parse_args()
    
    # 解析发送数据
    send_data = parse_hex_data(args.send) if args.send else None
    
    if send_data:
        print(f"准备发送数据: {binascii.hexlify(send_data).decode('utf-8').upper()}")
    
    bridge = ArduPilotTCPBridge(
        ip=args.ip,
        port=args.port,
        data=send_data,
        rate=args.rate,
        repeat=args.repeat,
        log_file=args.log
    )
    
    bridge.start()

if __name__ == "__main__":
    main()