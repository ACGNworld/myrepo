#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import time
import argparse
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import struct
import binascii
from datetime import datetime

# -------------------- 工具函数 --------------------
def parse_hex_data(hex_str):
    cleaned = hex_str.replace(" ", "").replace("0x", "").replace(",", "").replace(":", "")
    if len(cleaned) % 2 != 0:
        cleaned = '0' + cleaned
    try:
        return bytes.fromhex(cleaned)
    except ValueError as e:
        print(f"十六进制解析错误: {e}")
        return None

def build_flight_control_frame(**kw) -> bytes:
    sync_bytes = bytes([0xA5, 0x5A])
    source, cmd_type, data_len, end_byte = 0xAA, 0x01, 0x1D, 0xFF

    data  = struct.pack('<B', kw.get('control_mode', 4))
    data += struct.pack('<h', kw.get('x_offset', 0))
    data += struct.pack('<h', kw.get('y_offset', 0))
    data += struct.pack('<h', kw.get('z_offset', 0))
    data += struct.pack('<H', kw.get('max_speed', 1000))
    data += struct.pack('<H', kw.get('heading_angle', 0))
    data += struct.pack('<i', kw.get('target_lon', 1201561707))
    data += struct.pack('<i', kw.get('target_lat', 303048718))
    data += struct.pack('<i', kw.get('target_alt', 2500))
    data += struct.pack('<H', kw.get('target_heading', 18000))
    data += struct.pack('<H', kw.get('target_speed', 1500))
    data += struct.pack('<H', kw.get('max_yaw_rate', 1000))

    frame_body = bytes([source, cmd_type, data_len]) + data
    checksum = sum(frame_body) & 0xFF
    return sync_bytes + frame_body + bytes([checksum, end_byte])

# -------------------- 主程序 --------------------
class InteractiveTCPBridge:
    def __init__(self, ip, port, log_file=None):
        self.tcp_ip, self.tcp_port, self.log_file = ip, port, log_file
        self.sock = None
        self.running = False
        self.connection_active = False

        self.param_lock = threading.Lock()
        self.connection_lock = threading.Lock()

        self.params = dict(
            control_mode=4, x_offset=0, y_offset=0, z_offset=0,
            max_speed=1000, heading_angle=0,
            target_lon=1201561707, target_lat=303048718, target_alt=2500,
            target_heading=18000, target_speed=1500, max_yaw_rate=1000
        )

        self.mouse_config = dict(scale=2.0, center_x=480, center_y=270, canvas_width=960, canvas_height=540)

        self.setup_ui()

    # ---------- UI ----------
    def setup_ui(self):
        self.root = tk.Tk()
        self.root.title(f"ArduPilot TCP控制器 - {self.tcp_ip}:{self.tcp_port}")
        self.root.geometry("650x750")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.mouse_window = tk.Toplevel(self.root)
        self.mouse_window.title("鼠标控制 (Y/Z轴)")
        self.mouse_window.geometry("980x640")
        self.mouse_window.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.connect_status = tk.StringVar(value="未连接")
        self.send_count = tk.IntVar(value=0)
        self.y_err_display = tk.IntVar(value=0)
        self.z_err_display = tk.IntVar(value=0)

        self.create_main_ui()
        self.create_mouse_ui()

    def create_main_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # 状态栏
        status_frame = ttk.Frame(main_frame)
        status_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 15))

        ttk.Label(status_frame, text="连接状态:", font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
        # ✅ 保存 Label 引用
        self.status_label = ttk.Label(status_frame, textvariable=self.connect_status,
                                      foreground="red", font=('Arial', 10, 'bold'))
        self.status_label.pack(side=tk.LEFT, padx=15)

        ttk.Button(status_frame, text="连接", command=self.connect, width=8).pack(side=tk.RIGHT, padx=5)
        ttk.Button(status_frame, text="断开", command=self.disconnect, width=8).pack(side=tk.RIGHT)

        # 发送统计
        stat_frame = ttk.Frame(status_frame)
        stat_frame.pack(side=tk.LEFT, padx=30)
        ttk.Label(stat_frame, text="发送帧数:").pack(side=tk.LEFT)
        ttk.Label(stat_frame, textvariable=self.send_count, font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5)
        ttk.Label(stat_frame, text=" | Y误差:", foreground="blue").pack(side=tk.LEFT, padx=(20, 5))
        ttk.Label(stat_frame, textvariable=self.y_err_display, font=('Arial', 10, 'bold'), foreground="blue").pack(side=tk.LEFT)
        ttk.Label(stat_frame, text="Z误差:", foreground="blue").pack(side=tk.LEFT, padx=(15, 5))
        ttk.Label(stat_frame, textvariable=self.z_err_display, font=('Arial', 10, 'bold'), foreground="blue").pack(side=tk.LEFT)

        # 参数控制
        param_frame = ttk.LabelFrame(main_frame, text="飞行参数控制 (文本框输入)", padding="10")
        param_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S))

        param_configs = [
            ("控制模式", "control_mode", "int", 4, 0, 0),
            ("X偏移", "x_offset", "int", 0, 0, 1),
            ("最大速度", "max_speed", "int", 1000, 0, 2),
            ("航向角", "heading_angle", "int", 0, 0, 3),
            ("目标经度", "target_lon", "int", 1201561707, 0, 4),
            ("目标纬度", "target_lat", "int", 303048718, 0, 5),
            ("目标高度", "target_alt", "int", 2500, 2, 0),
            ("目标航向", "target_heading", "int", 18000, 2, 1),
            ("目标速度", "target_speed", "int", 1500, 2, 2),
            ("最大偏航率", "max_yaw_rate", "int", 1000, 2, 3),
        ]

        self.param_entries = {}
        for label, key, ptype, default, col, row in param_configs:
            ttk.Label(param_frame, text=label, font=('Arial', 9)).grid(row=row, column=col, sticky=tk.W, padx=(0 if col == 0 else 20, 5), pady=8)
            var = tk.StringVar(value=str(default))
            entry = ttk.Entry(param_frame, textvariable=var, width=12, font=('Courier', 9))
            entry.grid(row=row, column=col + 1, sticky=(tk.W, tk.E), pady=8)
            btn = ttk.Button(param_frame, text="更新", width=6, command=lambda k=key, v=var: self.update_param(k, v))
            btn.grid(row=row, column=col + 1, sticky=tk.E, padx=(0, 10))
            entry.bind('<Return>', lambda e, k=key, v=var: self.update_param(k, v))
            self.param_entries[key] = var

        # 缩放系数
        ttk.Label(param_frame, text="鼠标缩放系数", font=('Arial', 9, 'bold')).grid(row=6, column=0, sticky=tk.W, pady=(15, 5))
        scale_frame = ttk.Frame(param_frame)
        scale_frame.grid(row=6, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=(15, 5))
        self.scale_var = tk.DoubleVar(value=self.mouse_config['scale'])
        ttk.Scale(scale_frame, from_=0.5, to=10.0, variable=self.scale_var, orient=tk.HORIZONTAL, length=200,
                  command=self.update_mouse_scale).pack(side=tk.LEFT)
        ttk.Label(scale_frame, text="每像素单位值").pack(side=tk.LEFT, padx=10)
        self.scale_display = ttk.Label(scale_frame, text=f"= {self.mouse_config['scale']:.1f}", font=('Courier', 9, 'bold'))
        self.scale_display.pack(side=tk.LEFT)

        # 日志
        log_frame = ttk.LabelFrame(main_frame, text="通信日志", padding="5")
        log_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=15)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, width=75, state=tk.DISABLED, font=('Courier', 8))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        ttk.Label(main_frame, text="操作说明: 1) 点击鼠标控制窗口设置Y/Z误差 | 2) 在文本框输入参数后按回车或点击更新按钮 | 3) 所有修改实时生效",
                  foreground="gray", font=('Arial', 8)).grid(row=3, column=0, columnspan=3, pady=(5, 0), sticky=tk.W)

    def create_mouse_ui(self):
        canvas_width, canvas_height = self.mouse_config['canvas_width'], self.mouse_config['canvas_height']
        main_container = ttk.Frame(self.mouse_window, padding="10")
        main_container.pack(fill=tk.BOTH, expand=True)

        header_frame = ttk.Frame(main_container)
        header_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(header_frame, text="鼠标控制区域 (960×540像素)", font=('Arial', 12, 'bold')).pack(side=tk.LEFT)
        ttk.Button(header_frame, text="重置误差为零", command=self.reset_errors, width=15).pack(side=tk.RIGHT)
        self.mouse_status = ttk.Label(header_frame, text="当前误差: Y=0, Z=0", font=('Arial', 10, 'bold'), foreground="blue")
        self.mouse_status.pack(side=tk.RIGHT, padx=20)

        canvas_frame = ttk.Frame(main_container, relief=tk.SUNKEN, borderwidth=2)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(canvas_frame, width=canvas_width, height=canvas_height, bg='white', highlightthickness=0)
        self.canvas.pack()

        grid_interval = 60
        for i in range(0, canvas_width + 1, grid_interval):
            self.canvas.create_line(i, 0, i, canvas_height, fill='#f0f0f0')
        for i in range(0, canvas_height + 1, grid_interval):
            self.canvas.create_line(0, i, canvas_width, i, fill='#f0f0f0')

        center_x, center_y = canvas_width // 2, canvas_height // 2
        self.canvas.create_line(center_x, 0, center_x, canvas_height, fill='red', width=2)
        self.canvas.create_line(0, center_y, canvas_width, center_y, fill='red', width=2)
        label_font = ('Arial', 9, 'bold')
        self.canvas.create_text(center_x + 10, 15, text="Z+", fill="blue", anchor=tk.W, font=label_font)
        self.canvas.create_text(10, center_y - 10, text="Y-", fill="blue", anchor=tk.W, font=label_font)
        self.canvas.create_text(center_x + 10, canvas_height - 15, text="Z-", fill="blue", anchor=tk.W, font=label_font)
        self.canvas.create_text(canvas_width - 30, center_y - 10, text="Y+", fill="blue", anchor=tk.W, font=label_font)

        self.click_indicator = self.canvas.create_oval(center_x - 8, center_y - 8, center_x + 8, center_y + 8,
                                                       fill='green', outline='darkgreen', width=2)
        self.canvas.bind("<Button-1>", self.on_mouse_click)
        self.canvas.bind("<Motion>", self.on_mouse_move)

    # ---------- 业务逻辑 ----------
    def on_mouse_click(self, event):
        self.update_mouse_errors(event.x, event.y)

    def on_mouse_move(self, event):
        center_x = self.mouse_config['canvas_width'] // 2
        center_y = self.mouse_config['canvas_height'] // 2
        y_err = (event.x - center_x) * self.mouse_config['scale']
        z_err = -(event.y - center_y) * self.mouse_config['scale']
        self.mouse_status.config(text=f"当前误差: Y={int(y_err)}, Z={int(z_err)} (点击应用)")

    def update_mouse_errors(self, x, y):
        center_x = self.mouse_config['canvas_width'] // 2
        center_y = self.mouse_config['canvas_height'] // 2
        y_offset = (x - center_x) * self.mouse_config['scale']
        z_offset = -(y - center_y) * self.mouse_config['scale']
        with self.param_lock:
            self.params['y_offset'] = int(y_offset)
            self.params['z_offset'] = int(z_offset)
        self.y_err_display.set(int(y_offset))
        self.z_err_display.set(int(z_offset))
        self.mouse_status.config(text=f"当前误差: Y={int(y_offset)}, Z={int(z_offset)}")
        self.canvas.coords(self.click_indicator, x - 8, y - 8, x + 8, y + 8)
        self.log_message(f"鼠标设置: y_offset={int(y_offset)}, z_offset={int(z_offset)}")

    def update_mouse_scale(self, value):
        self.mouse_config['scale'] = float(value)
        self.scale_display.config(text=f"= {float(value):.1f}")

    def reset_errors(self):
        with self.param_lock:
            self.params['y_offset'] = 0
            self.params['z_offset'] = 0
        center_x = self.mouse_config['canvas_width'] // 2
        center_y = self.mouse_config['canvas_height'] // 2
        self.y_err_display.set(0)
        self.z_err_display.set(0)
        self.mouse_status.config(text="当前误差: Y=0, Z=0")
        self.canvas.coords(self.click_indicator, center_x - 8, center_y - 8, center_x + 8, center_y + 8)
        self.log_message("误差已重置为零")

    def update_param(self, key, var):
        try:
            value = var.get().strip()
            if not value:
                return
            if key in ['target_lon', 'target_lat']:
                val = int(float(value))
            elif 'speed' in key or 'rate' in key or key in ['max_speed', 'target_speed', 'max_yaw_rate']:
                val = int(float(value))
            else:
                val = int(value)
            with self.param_lock:
                self.params[key] = val
            self.log_message(f"参数更新: {key} = {val}")
        except ValueError as e:
            messagebox.showerror("参数错误", f"无效的数值: {e}")

    # ---------- TCP ----------
    def connect(self):
        if self.connection_active:
            return
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.sock.settimeout(0.5)
            self.connection_active = True
            self.connect_status.set("已连接")
            # ✅ 修改 Label 颜色
            self.status_label.configure(foreground="green")
            self.log_message(f"成功连接到 {self.tcp_ip}:{self.tcp_port}")
            self.running = True
            self.sender_thread = threading.Thread(target=self.sender_loop, daemon=True)
            self.sender_thread.start()
        except Exception as e:
            self.log_message(f"连接失败: {e}")
            messagebox.showerror("连接错误", f"无法连接到 {self.tcp_ip}:{self.tcp_port}\n{e}")

    def disconnect(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.connection_active = False
        self.connect_status.set("未连接")
        # ✅ 修改 Label 颜色
        self.status_label.configure(foreground="red")
        self.log_message("连接已断开")

    def sender_loop(self):
        interval = 0.5
        while self.running:
            if not self.connection_active:
                time.sleep(0.1)
                continue
            start = time.time()
            try:
                with self.param_lock:
                    params_copy = self.params.copy()
                frame = build_flight_control_frame(**params_copy)
                if self.send_frame(frame):
                    self.send_count.set(self.send_count.get() + 1)
                elapsed = time.time() - start
                time.sleep(max(0, interval - elapsed))
            except Exception as e:
                self.log_message(f"发送错误: {e}")
                self.connection_active = False
                break

    def send_frame(self, frame):
        if not self.connection_active or not self.sock:
            return False
        try:
            with self.connection_lock:
                self.sock.sendall(frame)
            hex_data = binascii.hexlify(frame).decode('utf-8').upper()
            self.log_message(f"[发送] {len(frame)} 字节 | {hex_data}")
            if self.log_file:
                with open(self.log_file, "a") as f:
                    f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}] SEND: {hex_data}\n")
            return True
        except Exception as e:
            self.log_message(f"发送失败: {e}")
            self.connection_active = False
            self.connect_status.set("连接断开")
            self.status_label.configure(foreground="red")
            return False

    # ---------- 日志 ----------
    def log_message(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] {message}\n"
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, log_line)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)
        if int(self.log_text.index('end-1c').split('.')[0]) > 1000:
            self.log_text.delete(1.0, 100.0)

    def on_closing(self):
        if messagebox.askokcancel("退出", "确定要退出程序吗?"):
            self.running = False
            self.disconnect()
            self.root.destroy()

    def run(self):
        self.connect()
        self.root.mainloop()


# -------------------- 入口 --------------------
def main():
    parser = argparse.ArgumentParser(description='交互式TCP模拟视觉飞行控制软件')
    parser.add_argument('--ip', default='127.0.0.1', help='TCP服务器IP (默认: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9998, help='TCP端口 (默认: 9998)')
    parser.add_argument('--log', metavar='FILE', help='记录所有收发数据到文件')
    args = parser.parse_args()

    app = InteractiveTCPBridge(ip=args.ip, port=args.port, log_file=args.log)
    app.run()


if __name__ == "__main__":
    main()