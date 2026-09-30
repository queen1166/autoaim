#!/usr/bin/env python3
"""
协议自检 —— 独立于 C++ 的第二份实现，用来交叉验证字节序。

两份实现对同一组输入产生同样的字节，才能说明字节序没写错。
协议没有魔数和 CRC，字节序错了不会报错，只会静默地发疯。

用法：
    python3 tools/protocol_check.py                  # 只做自检
    python3 tools/protocol_check.py /dev/ttyACM0     # 自检 + 真机收几帧
"""

import struct
import sys

SEND_BODY_LEN  = 16
RECV_BODY_LEN  = 28
SEND_FRAME_LEN = 18
RECV_FRAME_LEN = 30
ID_GIMBAL      = 1
ID_SHOOT       = 2


# ── 打包：对应 C++ 的 srm::pack_target ──────────────────────
def pack_target(yaw_deg: float, pitch_deg: float, fire_flag: int) -> bytes:
    """'<' = 小端 + 紧凑无填充，所以这 6 个字段正好 18 字节。"""
    frame = struct.pack("<h h f f h i",
                        SEND_BODY_LEN,
                        ID_GIMBAL, yaw_deg, pitch_deg,
                        ID_SHOOT, fire_flag)
    assert len(frame) == SEND_FRAME_LEN, len(frame)
    return frame


# ── 解包：对应 C++ 的 srm::parse_feedback ───────────────────
def parse_feedback(b: bytes):
    assert len(b) == RECV_FRAME_LEN, len(b)
    if struct.unpack_from("<h", b, 0)[0] != RECV_BODY_LEN:
        return None
    id1, yaw, pitch, roll, mode, color = struct.unpack_from("<h f f f i i", b, 2)
    id2, speed = struct.unpack_from("<h f", b, 24)
    if id1 != ID_GIMBAL or id2 != ID_SHOOT:
        return None
    return dict(id1=id1, yaw=yaw, pitch=pitch, roll=roll,
                mode=mode, color=color,
                id2=id2, bullet_speed=speed)


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# ── 自检 ────────────────────────────────────────────────────
def self_test() -> bool:
    ok = True

    # 1. 协议文档给出的自检向量：yaw=1.0, pitch=0.0, fire_flag=0
    expected = "10 00 01 00 00 00 80 3F 00 00 00 00 02 00 00 00 00 00"
    got = hexs(pack_target(1.0, 0.0, 0))
    print(f"自检向量  期望: {expected}")
    print(f"          实际: {got}")
    if got != expected:
        print("  ✗ 不匹配")
        ok = False
    else:
        print("  ✓ 匹配")

    # 2. 圆整：打包 → 解析，数值应原样回来
    print()
    fb = parse_feedback(
        struct.pack("<h", RECV_BODY_LEN)
        + struct.pack("<h f f f i i", ID_GIMBAL, -45.5, 1.25, 0.0, 0, 3)
        + struct.pack("<h f", ID_SHOOT, 22.0))
    print(f"反馈解析  {fb}")
    if not (fb and abs(fb["yaw"] + 45.5) < 1e-5
            and abs(fb["pitch"] - 1.25) < 1e-5
            and abs(fb["bullet_speed"] - 22.0) < 1e-5):
        print("  ✗ 数值不对")
        ok = False
    else:
        print("  ✓ 数值正确")

    # 3. 负数和角度制的边界
    print()
    for yaw in (-180.0, -0.001, 0.0, 0.001, 179.999):
        f = pack_target(yaw, -3.5, -1)
        print(f"  yaw={yaw:>9.3f} → {hexs(f)}")

    return ok


# ── 可选：真机收几帧 ────────────────────────────────────────
def sniff(dev: str, seconds: float = 3.0) -> None:
    try:
        import serial
    except ImportError:
        print("\n没装 pyserial，跳过真机收帧：pip install pyserial")
        return

    import time
    print(f"\n打开 {dev}，收 {seconds:.0f} 秒…")
    ser = serial.Serial(dev, 115200, timeout=0.05)
    buf = bytearray()
    t0 = time.time()
    n_frame = 0

    while time.time() - t0 < seconds:
        buf.extend(ser.read(64))
        while len(buf) >= RECV_FRAME_LEN:
            body_len = struct.unpack_from("<h", buf, 0)[0]
            if body_len != RECV_BODY_LEN:
                print(f"  [丢帧] body_len={body_len}，清空缓存")
                buf.clear()
                break
            fb = parse_feedback(bytes(buf[:RECV_FRAME_LEN]))
            del buf[:RECV_FRAME_LEN]
            n_frame += 1
            if fb:
                print(f"  yaw={fb['yaw']:8.2f} pitch={fb['pitch']:7.2f} "
                      f"roll={fb['roll']:7.2f} 弹速={fb['bullet_speed']:5.1f} "
                      f"mode={fb['mode']} color={fb['color']}")

    ser.close()
    print(f"\n共收到 {n_frame} 帧")


if __name__ == "__main__":
    if not self_test():
        sys.exit(1)
    if len(sys.argv) > 1:
        sniff(sys.argv[1])
