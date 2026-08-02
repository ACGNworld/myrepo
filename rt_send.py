#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ArduPilot FOLLOW_EXT TCP 控制台。

TCP 连接的另一端应提供协议中定义的串口字节流。控制帧按 20 Hz 发送，
图表点击/拖动或手动应用的 Y/Z 偏移会写入后续控制帧。
"""

import argparse
import binascii
import math
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, scrolledtext, ttk


SYNC = b"\xA5\x5A"
SOURCE_NCU = 0xAA
SOURCE_FCU = 0xBB
CONTROL_COMMAND = 0x01
PARAMETER_COMMAND = 0x02
SYSTEM_COMMAND = 0x03
PROTOCOL_VERSION = 0x01

MODE_NAMES = {
    0x00: "待命",
    0x01: "视觉模式",
    0x02: "悬停模式",
    0x03: "导航模式",
    0x04: "起飞模式",
    0x05: "急停模式",
}

PARAMETERS = {
    0x01: ("FOLE_AUTO_ENABLE", 1.0, 0.0, 1.0),
    0x02: ("FOLE_KP_YAW", 1.5, -1000.0, 1000.0),
    0x03: ("FOLE_KP_THR", 0.3, -1000.0, 1000.0),
    0x04: ("FOLE_KD_YAW", 0.0, -1000.0, 1000.0),
    0x05: ("FOLE_KD_THR", 0.0, None, None),
    0x06: ("FOLE_SPEED", 1000.0, 0.0, 10000.0),
    0x07: ("FOLE_ALPHA", 1.0, 0.0, 1.0),
    0x08: ("FOLE_ERR_SLOW_EN", 1.0, 0.0, 1.0),
    0x09: ("FOLE_TURN_LIM_EN", 1.0, 0.0, 1.0),
    0x0A: ("FOLE_CLB_SPD_EN", 1.0, 0.0, 1.0),
    0x0B: ("FOLE_TURN_FF_EN", 1.0, 0.0, 1.0),
    0x0C: ("FOLE_YAW_D_EN", 1.0, 0.0, 1.0),
    0x0D: ("FOLE_ERR_SLOW_SC", 350.0, 0.001, 100000.0),
    0x0E: ("FOLE_VERT_ERR_WT", 0.8, 0.0, 100.0),
    0x0F: ("FOLE_MIN_SPD_MUL", 0.2, 0.0, 1.0),
    0x10: ("FOLE_TURN_ACC_RT", 0.6, 0.0, 1.0),
    0x11: ("FOLE_MIN_YAW_RT", math.radians(2.0), 0.0, 10.0),
}

BOOLEAN_PARAMETER_IDS = frozenset({0x01, 0x08, 0x09, 0x0A, 0x0B, 0x0C})


def parse_hex_data(hex_str):
    """解析调试用十六进制字符串。"""
    cleaned = hex_str.replace(" ", "").replace("0x", "").replace("0X", "")
    cleaned = cleaned.replace(",", "").replace(":", "")
    if not cleaned:
        return None
    if len(cleaned) % 2 != 0:
        cleaned = "0" + cleaned
    try:
        return bytes.fromhex(cleaned)
    except ValueError:
        return None


def build_frame(command_type, payload, source=SOURCE_NCU):
    """按 protocol.md 的通用帧格式构建一帧。"""
    if not 0 <= command_type <= 0xFF:
        raise ValueError("command_type 超出 uint8 范围")
    if len(payload) > 0xFF:
        raise ValueError("payload 超过协议允许的 255 字节")
    frame_body = bytes((source, command_type, len(payload))) + payload
    checksum = sum(frame_body) & 0xFF
    return SYNC + frame_body + bytes((checksum, 0xFF))


def build_flight_control_frame(**kw) -> bytes:
    """构建 36 字节飞行控制帧。所有字段使用 protocol.md 定义的小端类型。"""
    data = struct.pack(
        "<BhhhHhiiihHH",
        kw.get("control_mode", 0x00),
        kw.get("x_offset", 0),
        kw.get("y_offset", 0),
        kw.get("z_offset", 0),
        kw.get("max_speed", 1000),
        kw.get("heading_angle", 0),
        kw.get("target_lon", 1201561707),
        kw.get("target_lat", 303048718),
        kw.get("target_alt", 2500),
        kw.get("target_heading", 18000),
        kw.get("target_speed", 1500),
        kw.get("max_yaw_rate", 1000),
    )
    if len(data) != 0x1D:
        raise ValueError(f"飞行控制数据长度错误: {len(data)}")
    return build_frame(CONTROL_COMMAND, data)


def build_parameter_frame(operation, request_id, parameter_id, value=0.0) -> bytes:
    """构建 FOLLOW_EXT 参数请求帧。"""
    if operation not in (0x01, 0x02, 0x03):
        raise ValueError("不支持的参数操作码")
    if not 0 <= request_id <= 0xFF or parameter_id not in PARAMETERS:
        raise ValueError("请求序号或参数 ID 无效")
    if operation != 0x03 and PARAMETERS[parameter_id][2] is None:
        raise ValueError(f"{PARAMETERS[parameter_id][0]} 只支持读取")
    if not math.isfinite(value):
        raise ValueError("参数值必须是有限浮点数")
    payload = struct.pack(
        "<BBBBf", PROTOCOL_VERSION, operation, request_id, parameter_id, float(value)
    )
    return build_frame(PARAMETER_COMMAND, payload)


def build_system_control_frame(command) -> bytes:
    """构建协议定义的系统控制帧（0x02 重启、0x03 关机）。"""
    if command not in (0x02, 0x03):
        raise ValueError("系统命令仅支持 0x02 重启和 0x03 关机")
    return build_frame(SYSTEM_COMMAND, bytes((command,)))


def extract_frames(buffer):
    """从接收缓存中提取完整帧，返回 (frames, 剩余缓存)。"""
    frames = []
    buffer = bytearray(buffer)
    while True:
        sync_index = buffer.find(SYNC)
        if sync_index < 0:
            # 保留最后一个可能是起始码首字节的 A5。
            buffer = bytearray(buffer[-1:]) if buffer[-1:] == SYNC[:1] else bytearray()
            break
        if sync_index:
            del buffer[:sync_index]
        if len(buffer) < 5:
            break
        data_length = buffer[4]
        total_length = data_length + 7
        if total_length > 64:
            del buffer[0]
            continue
        if len(buffer) < total_length:
            break
        frame = bytes(buffer[:total_length])
        if frame[-1] != 0xFF:
            del buffer[0]
            continue
        if (sum(frame[2:-2]) & 0xFF) != frame[-2]:
            del buffer[0]
            continue
        frames.append(frame)
        del buffer[:total_length]
    return frames, bytes(buffer)


def decode_feedback(frame):
    """解析协议中 FCU 返回的状态、参数和系统反馈。"""
    if len(frame) < 7 or frame[2] != SOURCE_FCU:
        return None
    command_type = frame[3]
    data_length = frame[4]
    payload = frame[5:-2]

    if command_type == CONTROL_COMMAND and data_length == 0x16 and len(payload) == 0x16:
        values = struct.unpack("<BBiiihHhh", payload)
        return {
            "kind": "flight_status",
            "control_mode": values[0],
            "battery": values[1],
            "lon": values[2],
            "lat": values[3],
            "alt": values[4],
            "yaw": values[5],
            "speed": values[6],
            "roll": values[7],
            "pitch": values[8],
        }

    if command_type == PARAMETER_COMMAND and data_length == 0x09 and len(payload) == 0x09:
        version, operation, request_id, parameter_id, status, value = struct.unpack(
            "<BBBBBf", payload
        )
        return {
            "kind": "parameter_feedback",
            "version": version,
            "operation": operation,
            "request_id": request_id,
            "parameter_id": parameter_id,
            "status": status,
            "value": value,
        }

    if command_type == PARAMETER_COMMAND and data_length == 0x02 and len(payload) == 0x02:
        return {"kind": "system_feedback", "command": payload[0], "status": payload[1]}
    return {"kind": "unknown_feedback", "command_type": command_type, "data_length": data_length}


class InteractiveTCPBridge:
    """Tkinter UI 与 TCP 字节流之间的双向桥接器。"""

    def __init__(self, ip, port, log_file=None, rate=20.0):
        self.tcp_ip = ip
        self.tcp_port = port
        self.log_file = log_file
        self.rate = rate if rate > 0 else 20.0
        self.interval = 1.0 / self.rate

        self.sock = None
        self.connection_active = False
        self.connecting = False
        self.connect_generation = 0
        self.stop_event = None
        self.sender_thread = None
        self.receiver_thread = None
        self.state_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.param_lock = threading.Lock()
        self.file_lock = threading.Lock()
        self.request_lock = threading.Lock()
        self.request_id = 0
        self.pending_requests = {}
        self.ui_queue = queue.Queue()
        self.tx_count = 0
        self.rx_count = 0

        self.params = {
            "control_mode": 0x00,
            "x_offset": 0,
            "y_offset": 0,
            "z_offset": 0,
            "max_speed": 1000,
            "heading_angle": 0,
            "target_lon": 1201561707,
            "target_lat": 303048718,
            "target_alt": 2500,
            "target_heading": 18000,
            "target_speed": 1500,
            "max_yaw_rate": 1000,
        }
        self.mouse_config = {"scale": 2.0}
        self.offset_source = "visual"
        self.visual_offsets = (0, 0)
        self.manual_offsets = (0, 0)
        self.setup_ui()

    # ---------- UI ----------
    def setup_ui(self):
        self.root = tk.Tk()
        self.root.title(f"FOLLOW_EXT 控制台 - {self.tcp_ip}:{self.tcp_port}")
        self.root.geometry("1280x950")
        self.root.minsize(1080, 720)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Title.TLabel", font=("Arial", 16, "bold"))
        style.configure("Section.TLabelframe.Label", font=("Arial", 10, "bold"))
        style.configure("Action.TButton", padding=(10, 7))
        style.configure("Danger.TButton", padding=(10, 7), foreground="#a61b1b")
        style.configure("Status.TLabel", font=("Arial", 10, "bold"))

        self.connect_status = tk.StringVar(value="未连接")
        self.status_detail = tk.StringVar(value="等待 TCP 服务器")
        self.send_count_var = tk.StringVar(value="0")
        self.recv_count_var = tk.StringVar(value="0")
        self.current_mode_var = tk.StringVar(value=MODE_NAMES[self.params["control_mode"]])
        self.preview_var = tk.StringVar(value="预览: Y=0, Z=0")
        self.applied_error_var = tk.StringVar(value="已应用(视觉): Y=0, Z=0")
        self.scale_var = tk.DoubleVar(value=self.mouse_config["scale"])
        self.endpoint_ip_var = tk.StringVar(value=self.tcp_ip)
        self.endpoint_port_var = tk.StringVar(value=str(self.tcp_port))
        self.mode_combo_var = tk.StringVar(value=self.current_mode_var.get())
        self.follow_selected_var = tk.IntVar(value=0x01)
        self.follow_values = {param_id: data[1] for param_id, data in PARAMETERS.items()}
        self.follow_value_vars = {
            param_id: tk.StringVar(value=f"{value:g}")
            for param_id, value in self.follow_values.items()
        }

        self.feedback_vars = {
            "mode": tk.StringVar(value="--"),
            "battery": tk.StringVar(value="--"),
            "alt": tk.StringVar(value="--"),
            "speed": tk.StringVar(value="--"),
            "position": tk.StringVar(value="--"),
            "attitude": tk.StringVar(value="--"),
            "last": tk.StringVar(value="尚未收到 FCU 状态反馈"),
        }
        self.param_vars = {}
        self.visual_point = None
        self.visual_marker = None
        self.visual_preview_marker = None
        self.visual_dragging = False
        self.canvas_grid_items = []
        self.status_label = None
        self.log_text = None

        self.create_header()
        self.create_body()
        self.create_log_panel()
        self.root.after(50, self.process_ui_events)

    def create_header(self):
        header = ttk.Frame(self.root, padding=(16, 12, 16, 8))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(1, weight=1)

        ttk.Label(header, text="FOLLOW_EXT 控制台", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(header, text="模拟视觉引导与飞控状态监视", foreground="#5f6b76").grid(
            row=1, column=0, sticky="w", pady=(2, 0)
        )

        status_box = ttk.Frame(header)
        status_box.grid(row=0, column=1, rowspan=2, sticky="e", padx=(20, 0))
        ttk.Label(status_box, text="连接", foreground="#5f6b76").pack(side=tk.LEFT)
        self.status_label = ttk.Label(status_box, textvariable=self.connect_status, style="Status.TLabel")
        self.status_label.pack(side=tk.LEFT, padx=(8, 16))
        ttk.Label(status_box, text="TX", foreground="#5f6b76").pack(side=tk.LEFT)
        ttk.Label(status_box, textvariable=self.send_count_var, style="Status.TLabel").pack(
            side=tk.LEFT, padx=(4, 14)
        )
        ttk.Label(status_box, text="RX", foreground="#5f6b76").pack(side=tk.LEFT)
        ttk.Label(status_box, textvariable=self.recv_count_var, style="Status.TLabel").pack(
            side=tk.LEFT, padx=(4, 0)
        )

    def create_body(self):
        body = ttk.Frame(self.root, padding=(16, 0, 16, 8))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body, width=420)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        left.grid_propagate(False)
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)

        self.create_connection_panel(left)

        tabs = ttk.Notebook(left)
        tabs.grid(row=1, column=0, sticky="nsew")
        self.control_tabs = tabs
        task_tab = ttk.Frame(tabs, padding=(0, 8, 0, 0))
        parameter_tab = ttk.Frame(tabs, padding=(0, 8, 0, 0))
        task_tab.columnconfigure(0, weight=1)
        parameter_tab.columnconfigure(0, weight=1)
        parameter_tab.rowconfigure(0, weight=1)
        tabs.add(task_tab, text="任务与控制帧")
        tabs.add(parameter_tab, text="参数设置")

        self.create_mission_panel(task_tab)
        self.create_flight_params_panel(task_tab)
        self.create_follow_params_panel(parameter_tab)

        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)
        self.create_feedback_panel(right)
        self.create_visual_panel(right)

    def create_connection_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="TCP 连接", padding=10, style="Section.TLabelframe")
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)

        ttk.Label(frame, text="地址").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Entry(frame, textvariable=self.endpoint_ip_var, width=16).grid(
            row=0, column=1, sticky="ew", padx=(0, 8)
        )
        ttk.Label(frame, text="端口").grid(row=0, column=2, sticky="w", padx=(0, 6))
        ttk.Entry(frame, textvariable=self.endpoint_port_var, width=8).grid(row=0, column=3, sticky="ew")
        ttk.Button(frame, text="连接", style="Action.TButton", command=lambda: self.connect(True)).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(9, 0), padx=(0, 5)
        )
        ttk.Button(frame, text="断开", command=self.disconnect).grid(
            row=1, column=2, columnspan=2, sticky="ew", pady=(9, 0)
        )
        ttk.Label(frame, textvariable=self.status_detail, foreground="#5f6b76").grid(
            row=2, column=0, columnspan=4, sticky="w", pady=(7, 0)
        )

    def create_mission_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="任务控制", padding=10, style="Section.TLabelframe")
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)

        ttk.Label(frame, text="当前模式").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.current_mode_var, style="Status.TLabel").grid(
            row=0, column=1, columnspan=2, sticky="e"
        )
        buttons = [
            ("起飞", lambda: self.set_control_mode(0x04), "Action.TButton"),
            ("导航", lambda: self.set_control_mode(0x03), "Action.TButton"),
            ("视觉", lambda: self.set_control_mode(0x01), "Action.TButton"),
            ("悬停", lambda: self.set_control_mode(0x02), "Action.TButton"),
            ("降落", self.request_landing, "Action.TButton"),
            ("急停", lambda: self.set_control_mode(0x05), "Danger.TButton"),
        ]
        for index, (label, command, style_name) in enumerate(buttons):
            ttk.Button(frame, text=label, command=command, style=style_name).grid(
                row=1 + index // 3,
                column=index % 3,
                sticky="ew",
                padx=(0 if index % 3 == 0 else 4, 0 if index % 3 == 2 else 4),
                pady=(8 if index < 3 else 5, 0),
            )
        ttk.Label(
            frame,
            text="降落映射: GPS 位置模式，目标高度设为 0 cm",
            foreground="#7a5b18",
        ).grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))

        ttk.Label(frame, text="模式选择").grid(row=4, column=0, sticky="w", pady=(8, 0))
        self.mode_combo = ttk.Combobox(
            frame,
            textvariable=self.mode_combo_var,
            values=list(MODE_NAMES.values()),
            state="readonly",
            width=14,
        )
        self.mode_combo.grid(row=4, column=1, columnspan=2, sticky="ew", pady=(8, 0))
        self.mode_combo.bind("<<ComboboxSelected>>", self.on_mode_selected)

    def create_flight_params_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="控制帧字段", padding=10, style="Section.TLabelframe")
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)

        fields = [
            ("X 偏移", "x_offset", "int16"),
            ("Y 偏移", "y_offset", "int16"),
            ("Z 偏移", "z_offset", "int16"),
            ("最大速度 cm/s", "max_speed", "uint16"),
            ("航向角 0.01°", "heading_angle", "int16"),
            ("目标高度 cm", "target_alt", "int32"),
            ("目标经度 ×1e7", "target_lon", "int32"),
            ("目标纬度 ×1e7", "target_lat", "int32"),
            ("目标航向 0.01°", "target_heading", "int16"),
            ("目标速度 0.01m/s", "target_speed", "uint16"),
            ("最大偏航率", "max_yaw_rate", "uint16"),
        ]
        for index, (label, key, _type_name) in enumerate(fields):
            row = index // 2
            col = (index % 2) * 2
            ttk.Label(frame, text=label).grid(row=row, column=col, sticky="w", padx=(0, 6), pady=4)
            var = tk.StringVar(value=str(self.params[key]))
            self.param_vars[key] = var
            entry = ttk.Entry(frame, textvariable=var, width=12)
            entry.grid(row=row, column=col + 1, sticky="ew", pady=4)
            entry.bind("<Return>", lambda _event: self.apply_flight_params())

        button_row = (len(fields) + 1) // 2
        ttk.Button(frame, text="应用控制帧字段", command=self.apply_flight_params).grid(
            row=button_row, column=0, columnspan=4, sticky="ew", pady=(7, 0)
        )

    def create_follow_params_panel(self, parent):
        container = ttk.Frame(parent)
        container.grid(row=0, column=0, sticky="nsew")
        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)

        canvas = tk.Canvas(container, highlightthickness=0, borderwidth=0)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        frame = ttk.LabelFrame(
            canvas,
            text="FOLLOW_EXT 参数",
            padding=10,
            style="Section.TLabelframe",
        )
        frame_window = canvas.create_window((0, 0), window=frame, anchor="nw")

        def update_scroll_region(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def resize_frame(event):
            canvas.itemconfigure(frame_window, width=event.width)

        frame.bind("<Configure>", update_scroll_region)
        canvas.bind("<Configure>", resize_frame)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)

        ttk.Label(frame, text="选择 / 参数").grid(row=0, column=0, sticky="w", pady=(0, 5))
        ttk.Label(frame, text="当前值").grid(row=0, column=1, columnspan=2, sticky="w", pady=(0, 5))
        for row, (parameter_id, (name, _default, lower, _upper)) in enumerate(PARAMETERS.items(), start=1):
            ttk.Radiobutton(
                frame,
                text=f"{name} (0x{parameter_id:02X})",
                variable=self.follow_selected_var,
                value=parameter_id,
            ).grid(row=row, column=0, sticky="w", pady=3)
            entry_state = "normal" if lower is not None else "readonly"
            ttk.Entry(
                frame,
                textvariable=self.follow_value_vars[parameter_id],
                state=entry_state,
                width=14,
            ).grid(row=row, column=1, columnspan=2, sticky="ew", pady=3)

        button_row = len(PARAMETERS) + 1
        ttk.Button(
            frame,
            text="读取选中",
            command=lambda: self.send_parameter_request(0x03),
        ).grid(row=button_row, column=0, sticky="ew", pady=(9, 0), padx=(0, 4))
        ttk.Button(
            frame,
            text="临时写入选中",
            command=lambda: self.send_parameter_request(0x01),
        ).grid(row=button_row, column=1, sticky="ew", pady=(9, 0), padx=2)
        ttk.Button(
            frame,
            text="持久化写入选中",
            command=lambda: self.send_parameter_request(0x02),
        ).grid(row=button_row, column=2, sticky="ew", pady=(9, 0), padx=(4, 0))
        ttk.Label(
            frame,
            text="连接后自动读取全部参数；写入操作只作用于当前选中项",
            foreground="#5f6b76",
        ).grid(row=button_row + 1, column=0, columnspan=3, sticky="w", pady=(7, 0))

    def create_feedback_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="FCU 状态反馈", padding=10, style="Section.TLabelframe")
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        for col in range(6):
            frame.columnconfigure(col, weight=1)
        cards = [
            ("模式", "mode"),
            ("电量", "battery"),
            ("高度", "alt"),
            ("速度", "speed"),
            ("位置", "position"),
            ("姿态", "attitude"),
        ]
        for col, (label, key) in enumerate(cards):
            cell = ttk.Frame(frame)
            cell.grid(row=0, column=col, sticky="ew", padx=(0 if col == 0 else 8, 0))
            ttk.Label(cell, text=label, foreground="#5f6b76").pack(anchor="w")
            ttk.Label(cell, textvariable=self.feedback_vars[key], style="Status.TLabel").pack(anchor="w", pady=(3, 0))
        ttk.Label(frame, textvariable=self.feedback_vars["last"], foreground="#5f6b76").grid(
            row=1, column=0, columnspan=6, sticky="w", pady=(8, 0)
        )

    def create_visual_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="模拟视觉识别区域", padding=10, style="Section.TLabelframe")
        frame.grid(row=1, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(frame)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        toolbar.columnconfigure(3, weight=1)
        ttk.Label(toolbar, textvariable=self.preview_var, style="Status.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(toolbar, textvariable=self.applied_error_var, foreground="#22733b").grid(
            row=0, column=1, sticky="w", padx=(16, 0)
        )
        ttk.Button(toolbar, text="误差清零", command=self.reset_errors).grid(row=0, column=2, sticky="e", padx=(16, 8))
        ttk.Label(toolbar, text="缩放").grid(row=0, column=3, sticky="e")
        ttk.Scale(
            toolbar,
            from_=0.5,
            to=10.0,
            variable=self.scale_var,
            orient=tk.HORIZONTAL,
            length=150,
            command=self.update_mouse_scale,
        ).grid(row=0, column=4, sticky="e", padx=(7, 0))
        self.scale_display = ttk.Label(toolbar, text="2.0 / px", width=8)
        self.scale_display.grid(row=0, column=5, sticky="e", padx=(5, 0))

        canvas_frame = ttk.Frame(frame, relief=tk.SUNKEN, borderwidth=1)
        canvas_frame.grid(row=1, column=0, sticky="nsew")
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(
            canvas_frame,
            bg="#f7f9fb",
            highlightthickness=0,
            cursor="crosshair",
            width=760,
            height=540,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda _event: self.redraw_visual_canvas())
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_press)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_release)
        self.root.after_idle(self.redraw_visual_canvas)

    def create_log_panel(self):
        frame = ttk.LabelFrame(self.root, text="通信日志", padding=(10, 7), style="Section.TLabelframe")
        frame.grid(row=2, column=0, sticky="ew", padx=16, pady=(0, 12))
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        debug_bar = ttk.Frame(frame)
        debug_bar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        debug_bar.columnconfigure(1, weight=1)
        ttk.Label(debug_bar, text="原始 HEX").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.raw_hex_var = tk.StringVar()
        raw_entry = ttk.Entry(debug_bar, textvariable=self.raw_hex_var)
        raw_entry.grid(row=0, column=1, sticky="ew")
        raw_entry.bind("<Return>", lambda _event: self.send_raw_hex())
        ttk.Button(debug_bar, text="发送", command=self.send_raw_hex).grid(row=0, column=2, padx=(8, 0))
        ttk.Button(debug_bar, text="清空", command=self.clear_log).grid(row=0, column=3, padx=(5, 0))

        self.log_text = scrolledtext.ScrolledText(
            frame,
            height=7,
            state=tk.DISABLED,
            font=("Courier New", 9),
            background="#111820",
            foreground="#d9e2ec",
            insertbackground="#ffffff",
        )
        self.log_text.grid(row=1, column=0, sticky="ew")

    # ---------- 视觉目标 ----------
    def redraw_visual_canvas(self):
        if not hasattr(self, "canvas"):
            return
        width = max(self.canvas.winfo_width(), 240)
        height = max(self.canvas.winfo_height(), 180)
        center_x, center_y = width / 2, height / 2
        self.canvas.delete("all")

        grid_step = max(30, min(width, height) / 8)
        x = center_x % grid_step
        while x <= width:
            self.canvas.create_line(x, 0, x, height, fill="#e4e9ee", tags="grid")
            x += grid_step
        y = center_y % grid_step
        while y <= height:
            self.canvas.create_line(0, y, width, y, fill="#e4e9ee", tags="grid")
            y += grid_step
        self.canvas.create_line(center_x, 0, center_x, height, fill="#d15b5b", width=2)
        self.canvas.create_line(0, center_y, width, center_y, fill="#d15b5b", width=2)
        self.canvas.create_text(center_x + 8, 15, text="Z+", fill="#285b86", anchor="w")
        self.canvas.create_text(width - 8, center_y - 8, text="Y+", fill="#285b86", anchor="e")
        self.canvas.create_text(center_x + 8, height - 15, text="Z-", fill="#285b86", anchor="w")
        self.canvas.create_text(8, center_y - 8, text="Y-", fill="#285b86", anchor="w")

        if self.visual_point is None:
            self.visual_point = (center_x, center_y)
        else:
            old_width = getattr(self, "visual_canvas_size", (width, height))[0]
            old_height = getattr(self, "visual_canvas_size", (width, height))[1]
            px, py = self.visual_point
            if old_width:
                px = center_x + (px - old_width / 2) * width / old_width
            if old_height:
                py = center_y + (py - old_height / 2) * height / old_height
            self.visual_point = (max(0, min(width, px)), max(0, min(height, py)))
        self.visual_canvas_size = (width, height)
        self.visual_marker = self.canvas.create_oval(0, 0, 0, 0, fill="#2b8a57", outline="#155c37", width=2)
        self.canvas.coords(self.visual_marker, self.visual_point[0] - 8, self.visual_point[1] - 8,
                           self.visual_point[0] + 8, self.visual_point[1] + 8)

    def canvas_error(self, x, y):
        width = max(self.canvas.winfo_width(), 240)
        height = max(self.canvas.winfo_height(), 180)
        x = max(0, min(width, x))
        y = max(0, min(height, y))
        y_error = round((x - width / 2) * self.mouse_config["scale"])
        z_error = round((height / 2 - y) * self.mouse_config["scale"])
        return x, y, y_error, z_error

    def on_mouse_move(self, event):
        if self.visual_dragging:
            self.update_mouse_errors(event.x, event.y, log=False)
            return
        _x, _y, y_error, z_error = self.canvas_error(event.x, event.y)
        self.preview_var.set(f"预览: Y={y_error}, Z={z_error}")

    def on_mouse_press(self, event):
        self.visual_dragging = True
        self.canvas.focus_set()
        try:
            self.canvas.grab_set()
        except tk.TclError:
            pass
        self.update_mouse_errors(event.x, event.y, log=True)

    def on_mouse_release(self, _event):
        self.visual_dragging = False
        try:
            self.canvas.grab_release()
        except tk.TclError:
            pass

    def update_mouse_errors(self, x, y, log=True):
        x, y, y_error, z_error = self.canvas_error(x, y)
        with self.param_lock:
            self.params["y_offset"] = y_error
            self.params["z_offset"] = z_error
            self.visual_offsets = (y_error, z_error)
            self.offset_source = "visual"
        self.param_vars["y_offset"].set(str(y_error))
        self.param_vars["z_offset"].set(str(z_error))
        self.visual_point = (x, y)
        if self.visual_marker:
            self.canvas.coords(self.visual_marker, x - 8, y - 8, x + 8, y + 8)
        self.applied_error_var.set(f"已应用(视觉): Y={y_error}, Z={z_error}")
        if log:
            self.log_message(f"视觉目标更新: Y={y_error}, Z={z_error}")

    def update_visual_point_from_offsets(self, y_error, z_error):
        """将手动输入的 Y/Z 偏移同步到图表标记，不修改已应用的参数值。"""
        width = max(self.canvas.winfo_width(), 240)
        height = max(self.canvas.winfo_height(), 180)
        scale = self.mouse_config["scale"]
        x = width / 2 + y_error / scale
        y = height / 2 - z_error / scale
        x = max(0, min(width, x))
        y = max(0, min(height, y))
        self.visual_point = (x, y)
        if self.visual_marker:
            self.canvas.coords(self.visual_marker, x - 8, y - 8, x + 8, y + 8)

    def update_mouse_scale(self, value):
        scale = float(value)
        self.mouse_config["scale"] = scale
        self.scale_display.config(text=f"{scale:.1f} / px")
        if self.visual_point:
            _x, _y, y_error, z_error = self.canvas_error(*self.visual_point)
            self.preview_var.set(f"预览: Y={y_error}, Z={z_error}")

    def reset_errors(self):
        width = max(self.canvas.winfo_width(), 240)
        height = max(self.canvas.winfo_height(), 180)
        self.update_mouse_errors(width / 2, height / 2, log=False)
        self.log_message("视觉误差已清零")

    # ---------- 控制动作 ----------
    def on_mode_selected(self, _event=None):
        selected = self.mode_combo_var.get()
        for mode, name in MODE_NAMES.items():
            if name == selected:
                self.set_control_mode(mode)
                break

    def set_control_mode(self, mode):
        if mode not in MODE_NAMES:
            return
        with self.param_lock:
            self.params["control_mode"] = mode
        self.current_mode_var.set(MODE_NAMES[mode])
        self.mode_combo_var.set(MODE_NAMES[mode])
        self.log_message(f"控制模式切换: 0x{mode:02X} {MODE_NAMES[mode]}，将随 20 Hz 控制帧发送")

    def request_landing(self):
        if not messagebox.askyesno(
            "确认降落",
            "协议没有独立降落指令。\n将发送 GPS 位置模式，并把当前目标高度设为 0 cm。继续吗？",
        ):
            return
        if not self.apply_flight_params(show_error=True):
            return
        self.param_vars["target_alt"].set("0")
        with self.param_lock:
            self.params["target_alt"] = 0
            self.params["control_mode"] = 0x03
        self.current_mode_var.set(MODE_NAMES[0x03])
        self.mode_combo_var.set(MODE_NAMES[0x03])
        self.log_message("降落请求: GPS 位置模式 + 当前目标经纬度 + target_alt=0 cm")

    def apply_flight_params(self, show_error=True):
        specs = {
            "x_offset": (-32768, 32767),
            "y_offset": (-32768, 32767),
            "z_offset": (-32768, 32767),
            "max_speed": (0, 65535),
            "heading_angle": (-32768, 32767),
            "target_lon": (-2147483648, 2147483647),
            "target_lat": (-2147483648, 2147483647),
            "target_alt": (-2147483648, 2147483647),
            "target_heading": (-32768, 32767),
            "target_speed": (0, 65535),
            "max_yaw_rate": (0, 65535),
        }
        parsed = {}
        try:
            for key, (lower, upper) in specs.items():
                value = int(self.param_vars[key].get().strip())
                if not lower <= value <= upper:
                    raise ValueError(f"{key} 必须在 {lower} 到 {upper} 之间")
                parsed[key] = value
        except (TypeError, ValueError) as error:
            if show_error:
                messagebox.showerror("参数错误", str(error))
            return False
        with self.param_lock:
            self.params.update(parsed)
            self.manual_offsets = (parsed["y_offset"], parsed["z_offset"])
            self.offset_source = "manual"
        self.update_visual_point_from_offsets(parsed["y_offset"], parsed["z_offset"])
        self.applied_error_var.set(
            f"已应用(手动): Y={parsed['y_offset']}, Z={parsed['z_offset']}"
        )
        self.log_message("控制帧字段已应用，将写入后续 0x01 飞行控制帧")
        return True

    # ---------- FOLLOW_EXT 参数 ----------
    def selected_parameter_id(self):
        return self.follow_selected_var.get()

    def request_all_parameters(self):
        """连接成功后读取协议 5.3 定义的全部参数当前值。"""
        self.log_message("连接成功，开始 GET 读取全部 FOLLOW_EXT 参数")
        for index, parameter_id in enumerate(PARAMETERS):
            self.root.after(
                index * 120,
                lambda p=parameter_id: self.send_get_if_connected(p),
            )

    def send_get_if_connected(self, parameter_id):
        with self.state_lock:
            connected = self.connection_active
        if connected:
            self.send_parameter_request(0x03, parameter_id, 0.0)

    def send_parameter_request(self, operation, parameter_id=None, value=None, retry_count=0):
        parameter_id = parameter_id if parameter_id is not None else self.selected_parameter_id()
        if operation == 0x02:
            with self.request_lock:
                persistent_pending = any(item[1] == 0x02 for item in self.pending_requests.values())
            if persistent_pending and retry_count == 0:
                messagebox.showinfo("等待参数应答", "上一条持久化写入尚未收到 FCU 应答")
                return

        if operation == 0x03:
            value = 0.0
        elif value is None:
            try:
                value = float(self.follow_value_vars[parameter_id].get().strip())
            except ValueError:
                value = None

        if operation != 0x03:
            try:
                if value is None or not math.isfinite(value):
                    raise ValueError("参数值必须是有限浮点数")
                _name, _default, lower, upper = PARAMETERS[parameter_id]
                if lower is None or upper is None:
                    raise ValueError("该参数只支持读取")
                if not lower <= value <= upper:
                    raise ValueError(f"参数范围为 {lower} 到 {upper}")
                if parameter_id in BOOLEAN_PARAMETER_IDS and value not in (0.0, 1.0):
                    raise ValueError(f"{PARAMETERS[parameter_id][0]} 只能是 0 或 1")
            except ValueError as error:
                if retry_count == 0:
                    messagebox.showerror("参数错误", str(error))
                else:
                    self.log_message(f"参数重试取消: {error}")
                return

        with self.request_lock:
            request_id = self.request_id
            self.request_id = (self.request_id + 1) & 0xFF
            self.pending_requests[request_id] = (parameter_id, operation, value, retry_count)
        frame = build_parameter_frame(operation, request_id, parameter_id, value)
        operation_name = {0x01: "临时写入", 0x02: "持久化写入", 0x03: "读取"}[operation]
        description = f"参数{operation_name}: ID=0x{parameter_id:02X}={value:g}"
        if self.send_frame(frame, description):
            suffix = f"，第 {retry_count} 次重试" if retry_count else ""
            self.log_message(f"已发送参数{operation_name}: request_id={request_id}{suffix}")
        else:
            with self.request_lock:
                self.pending_requests.pop(request_id, None)
            if retry_count == 0:
                self.log_message("参数请求未发送: 当前未连接 TCP 服务器")

    # ---------- TCP ----------
    def endpoint(self):
        ip = self.endpoint_ip_var.get().strip()
        try:
            port = int(self.endpoint_port_var.get().strip())
        except ValueError:
            raise ValueError("端口必须是整数")
        if not ip or not 1 <= port <= 65535:
            raise ValueError("IP 地址或端口无效")
        return ip, port

    def connect(self, show_error=False):
        with self.state_lock:
            if self.connection_active or self.connecting:
                return
            self.connecting = True
            self.connect_generation += 1
            generation = self.connect_generation
        try:
            ip, port = self.endpoint()
        except ValueError as error:
            with self.state_lock:
                self.connecting = False
            messagebox.showerror("连接参数错误", str(error))
            return
        self.connect_status.set("连接中")
        self.status_detail.set(f"正在连接 {ip}:{port}")
        threading.Thread(
            target=self.connect_worker,
            args=(ip, port, show_error, generation),
            daemon=True,
        ).start()

    def connect_worker(self, ip, port, show_error, generation):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(5.0)
            sock.connect((ip, port))
            sock.settimeout(0.2)
        except Exception as error:
            try:
                sock.close()
            except OSError:
                pass
            with self.state_lock:
                is_current = generation == self.connect_generation
                if is_current:
                    self.connecting = False
            if is_current:
                self.ui_queue.put(("connect_failed", str(error), show_error))
            return

        stop_event = threading.Event()
        with self.state_lock:
            if generation != self.connect_generation or not self.connecting:
                try:
                    sock.close()
                except OSError:
                    pass
                return
            self.sock = sock
            self.tcp_ip = ip
            self.tcp_port = port
            self.connection_active = True
            self.connecting = False
            self.stop_event = stop_event
        self.sender_thread = threading.Thread(target=self.sender_loop, args=(stop_event,), daemon=True)
        self.receiver_thread = threading.Thread(target=self.receiver_loop, args=(sock, stop_event), daemon=True)
        self.sender_thread.start()
        self.receiver_thread.start()
        self.ui_queue.put(("connected", ip, port))

    def disconnect(self):
        with self.state_lock:
            self.connecting = False
            self.connect_generation += 1
            self.connection_active = False
            stop_event = self.stop_event
            sock = self.sock
            self.sock = None
            self.stop_event = None
        with self.request_lock:
            self.pending_requests.clear()
        if stop_event:
            stop_event.set()
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self.connect_status.set("未连接")
        self.status_detail.set("等待 TCP 服务器")
        self.log_message("TCP 连接已断开")

    def mark_connection_lost(self, reason):
        with self.state_lock:
            was_active = self.connection_active
            self.connection_active = False
            stop_event = self.stop_event
            sock = self.sock
            self.sock = None
            self.stop_event = None
        if stop_event:
            stop_event.set()
        if sock:
            try:
                sock.close()
            except OSError:
                pass
        if was_active:
            self.ui_queue.put(("connection_lost", reason))

    def sender_loop(self, stop_event):
        next_tick = time.monotonic()
        local_count = 0
        while not stop_event.is_set():
            with self.param_lock:
                params_copy = self.params.copy()
                if self.offset_source == "manual":
                    params_copy["y_offset"], params_copy["z_offset"] = self.manual_offsets
                else:
                    params_copy["y_offset"], params_copy["z_offset"] = self.visual_offsets
            try:
                frame = build_flight_control_frame(**params_copy)
            except (struct.error, ValueError) as error:
                self.ui_queue.put(("worker_error", f"控制帧构建失败: {error}"))
                stop_event.wait(0.2)
                continue
            local_count += 1
            mode_name = MODE_NAMES.get(params_copy["control_mode"], "未知")
            self.send_frame(
                frame,
                None if local_count % max(1, round(self.rate)) else f"控制帧 {self.rate:g} Hz | {mode_name}",
            )
            next_tick += self.interval
            wait_time = next_tick - time.monotonic()
            if wait_time > 0:
                stop_event.wait(wait_time)
            else:
                next_tick = time.monotonic()

    def receiver_loop(self, sock, stop_event):
        buffer = b""
        while not stop_event.is_set():
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            except (OSError, ConnectionResetError) as error:
                if not stop_event.is_set():
                    self.mark_connection_lost(f"接收异常: {error}")
                return
            if not data:
                if not stop_event.is_set():
                    self.mark_connection_lost("服务器关闭连接")
                return
            buffer += data
            frames, buffer = extract_frames(buffer)
            for frame in frames:
                self.ui_queue.put(("received", frame, decode_feedback(frame)))

    def send_frame(self, frame, ui_description=None):
        with self.state_lock:
            sock = self.sock if self.connection_active else None
        if sock is None:
            return False
        try:
            with self.send_lock:
                sock.sendall(frame)
            self.tx_count += 1
            hex_data = binascii.hexlify(frame).decode("ascii").upper()
            self.write_file_log(f"SEND: {hex_data}")
            if ui_description:
                self.ui_queue.put(("sent", len(frame), ui_description, hex_data))
            return True
        except (OSError, ConnectionResetError) as error:
            self.mark_connection_lost(f"发送异常: {error}")
            return False

    def send_raw_hex(self):
        data = parse_hex_data(self.raw_hex_var.get())
        if data is None:
            messagebox.showerror("HEX 错误", "请输入有效的十六进制字节串")
            return
        if self.send_frame(data, f"原始 HEX {len(data)} 字节"):
            self.raw_hex_var.set("")

    # ---------- 日志与 UI 事件 ----------
    def write_file_log(self, message):
        if not self.log_file:
            return
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        try:
            with self.file_lock:
                with open(self.log_file, "a", encoding="utf-8") as log_file:
                    log_file.write(f"[{timestamp}] {message}\n")
        except OSError as error:
            self.ui_queue.put(("worker_error", f"日志文件写入失败: {error}"))

    def process_ui_events(self):
        try:
            while True:
                event = self.ui_queue.get_nowait()
                kind = event[0]
                if kind == "connected":
                    _kind, ip, port = event
                    self.connect_status.set("已连接")
                    self.status_detail.set(f"TCP 已连接 {ip}:{port}，控制帧 {self.rate:g} Hz")
                    self.status_label.configure(foreground="#22733b")
                    self.log_message(f"成功连接到 {ip}:{port}")
                    self.request_all_parameters()
                elif kind == "connect_failed":
                    _kind, reason, show_error = event
                    self.connect_status.set("未连接")
                    self.status_detail.set("连接失败，检查 SITL/TCP 端口")
                    self.status_label.configure(foreground="#a61b1b")
                    self.log_message(f"连接失败: {reason}")
                    if show_error:
                        messagebox.showerror("连接错误", f"无法连接到 TCP 服务器\n{reason}")
                elif kind == "connection_lost":
                    self.connect_status.set("已断开")
                    self.status_detail.set("连接已丢失，控制帧发送已停止")
                    self.status_label.configure(foreground="#a61b1b")
                    self.log_message(event[1])
                elif kind == "sent":
                    _kind, length, description, hex_data = event
                    self.log_message(f"[发送] {length} 字节 | {description} | {hex_data}")
                elif kind == "received":
                    self.rx_count += 1
                    self.recv_count_var.set(str(self.rx_count))
                    _kind, frame, feedback = event
                    hex_data = binascii.hexlify(frame).decode("ascii").upper()
                    description = self.apply_feedback(feedback)
                    self.write_file_log(f"RECV: {hex_data}")
                    self.log_message(f"[接收] {len(frame)} 字节 | {description} | {hex_data}")
                elif kind == "worker_error":
                    self.log_message(event[1])
        except queue.Empty:
            pass
        self.send_count_var.set(str(self.tx_count))
        self.root.after(50, self.process_ui_events)

    def apply_feedback(self, feedback):
        if not feedback:
            return "未知反馈"
        kind = feedback["kind"]
        if kind == "flight_status":
            mode = MODE_NAMES.get(feedback["control_mode"], f"未知(0x{feedback['control_mode']:02X})")
            self.feedback_vars["mode"].set(mode)
            self.feedback_vars["battery"].set(f"{feedback['battery']} %")
            self.feedback_vars["alt"].set(f"{feedback['alt'] / 100:.2f} m")
            self.feedback_vars["speed"].set(f"{feedback['speed'] / 100:.2f} m/s")
            self.feedback_vars["position"].set(
                f"{feedback['lat'] / 1e7:.6f}, {feedback['lon'] / 1e7:.6f}"
            )
            self.feedback_vars["attitude"].set(
                f"Y {feedback['yaw'] / 100:.1f}°  R {feedback['roll'] / 100:.1f}°  P {feedback['pitch'] / 100:.1f}°"
            )
            self.feedback_vars["last"].set(
                f"最近反馈 {datetime.now().strftime('%H:%M:%S')} | 控制模式 0x{feedback['control_mode']:02X}"
            )
            return f"FCU 状态: {mode}, 电量 {feedback['battery']}%"
        if kind == "parameter_feedback":
            status_names = {
                0x00: "OK",
                0x01: "BAD_VERSION",
                0x02: "BAD_OPERATION",
                0x03: "UNKNOWN_PARAM",
                0x04: "INVALID_VALUE",
                0x05: "NOT_SUPPORTED",
                0x06: "SAVE_FAILED",
                0x07: "BUSY",
                0x08: "MALFORMED",
            }
            request_id = feedback["request_id"]
            parameter_id = feedback["parameter_id"]
            if feedback["status"] == 0x00 and math.isfinite(feedback["value"]):
                if parameter_id in self.follow_values:
                    self.follow_values[parameter_id] = feedback["value"]
                    self.follow_value_vars[parameter_id].set(f"{feedback['value']:g}")
            with self.request_lock:
                pending = self.pending_requests.pop(request_id, None)
            if feedback["status"] == 0x07 and pending:
                pending_parameter_id, pending_operation, pending_value, retry_count = pending
                if retry_count < 3:
                    delay_ms = 500 * (retry_count + 1)
                    self.root.after(
                        delay_ms,
                        lambda p=pending_parameter_id, o=pending_operation, v=pending_value, r=retry_count + 1:
                        self.send_parameter_request(o, p, v, r),
                    )
                    self.log_message(f"参数队列 BUSY，将在 {delay_ms / 1000:g} 秒后重试")
                else:
                    self.log_message("参数队列 BUSY，已达到最大重试次数")
            status_code = feedback["status"]
            status_text = status_names.get(status_code, f"0x{status_code:02X}")
            return (
                f"参数应答: {PARAMETERS.get(parameter_id, ('未知参数',))[0]} "
                f"{status_text}="
                f"{feedback['value']:g}"
            )
        if kind == "system_feedback":
            return f"系统反馈: command=0x{feedback['command']:02X}, status=0x{feedback['status']:02X}"
        return "未识别的 FCU 反馈"

    def log_message(self, message):
        if self.log_text is None:
            return
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > 1200:
            self.log_text.delete("1.0", "200.0")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def clear_log(self):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def on_closing(self):
        if messagebox.askokcancel("退出", "确定要退出控制台吗？"):
            self.disconnect()
            self.root.destroy()

    def run(self):
        # 保留原工具启动即连接的习惯，但失败时不弹窗，避免 SITL 尚未启动时阻塞界面。
        self.root.after(100, lambda: self.connect(False))
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="交互式 TCP 模拟视觉飞行控制软件")
    parser.add_argument("--ip", default="127.0.0.1", help="TCP 服务器 IP (默认: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5762, help="TCP 端口 (默认: 5762)")
    parser.add_argument("--rate", type=float, default=20.0, help="控制帧发送速率 Hz (默认: 20)")
    parser.add_argument("--log", metavar="FILE", help="记录所有收发数据到指定文件")
    args = parser.parse_args()
    app = InteractiveTCPBridge(args.ip, args.port, args.log, args.rate)
    app.run()


if __name__ == "__main__":
    main()
