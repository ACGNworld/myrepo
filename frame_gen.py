import struct

def build_flight_control_frame(
    control_mode: int = 0x03,
    x_offset: int = 0,
    y_offset: int = 0,
    z_offset: int = 0,
    max_speed: int = 1000,
    heading_angle: int = 0,
    target_lon: int = 0,
    target_lat: int = 0,
    target_alt: int = 0,
    target_heading: int = 0,
    target_speed: int = 0,
    max_yaw_rate: int = 0,
) -> bytes:
    """
    构建飞行控制指令帧（小端序，自动校验）
    """
    # 固定字段
    sync_bytes = bytes([0xA5, 0x5A])
    source = 0xAA
    cmd_type = 0x01
    data_len = 0x1D
    end_byte = 0xFF

    # 打包数据段（小端序）
    data = struct.pack('<B', control_mode)
    data += struct.pack('<h', x_offset)
    data += struct.pack('<h', y_offset)
    data += struct.pack('<h', z_offset)
    data += struct.pack('<H', max_speed)
    data += struct.pack('<H', heading_angle)
    data += struct.pack('<i', target_lon)
    data += struct.pack('<i', target_lat)
    data += struct.pack('<i', target_alt)
    data += struct.pack('<H', target_heading)
    data += struct.pack('<H', target_speed)
    data += struct.pack('<H', max_yaw_rate)

    # 计算校验和（从指令来源到数据段末尾）
    frame_body = bytes([source, cmd_type, data_len]) + data
    checksum = sum(frame_body) & 0xFF

    # 构建完整帧
    frame = sync_bytes + frame_body + bytes([checksum, end_byte])
    return frame

# 示例用法
if __name__ == "__main__":
    frame = build_flight_control_frame(
        control_mode=0x04,
        x_offset=0,
        y_offset=0,
        z_offset=1000,
        max_speed=1000,
        heading_angle=0,
        target_lon=1201561707,
        target_lat=303048718,
        target_alt=2500,
        target_heading=18000,
        target_speed=1500,
        max_yaw_rate=1000,
    )
    print("生成的帧（十六进制）:", ' '.join(f'{b:02X}' for b in frame))

#十米起飞
#A5 5A AA 01 1D 04 00 00 00 00 E8 03 E8 03 00 00 6B 60 9E 47 0E 28 10 12 C4 09 00 00 50 46 DC 05 E8 03 D9 FF