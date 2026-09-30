#!/usr/bin/env python3
"""
端到端离线测试：合成图像 + PTY 假串口 + 真跑 autoaim 二进制。

回答的问题是"能编译"之外的："整条流水线真的能工作吗"。

它做的事：
  1. 用 make_test_bag.py 生成一批合成图像（装甲板绕中心公转，真值已知）
  2. 开一对 PTY，把从设备路径当作 /dev/ttyACM* 传给 autoaim
  3. 自己扮演下位机：读 18 字节指令帧，回 30 字节反馈帧
  4. 检查 autoaim 发出来的 yaw 是否跟得上真实的装甲板运动

用法:
    python3 tools/e2e_test.py <autoaim可执行文件> [--bag <目录>] [--secs 8]
"""

import argparse
import math
import os
import pty
import struct
import subprocess
import sys
import time

SEND_BODY = 16
RECV_BODY = 28
SEND_LEN = 18
RECV_LEN = 30


def pack_feedback(yaw_deg, pitch_deg, roll_deg, mode, color, bullet_speed):
    return (struct.pack("<h", RECV_BODY)
            + struct.pack("<h f f f i i", 1, yaw_deg, pitch_deg, roll_deg, mode, color)
            + struct.pack("<h f", 2, bullet_speed))


def parse_command(buf):
    """返回 (yaw_deg, pitch_deg, fire_flag) 或 None"""
    if len(buf) < SEND_LEN:
        return None
    if struct.unpack_from("<h", buf, 0)[0] != SEND_BODY:
        return None
    id1 = struct.unpack_from("<h", buf, 2)[0]
    if id1 != 1:
        return None
    yaw, pitch = struct.unpack_from("<f f", buf, 4)
    id2 = struct.unpack_from("<h", buf, 12)[0]
    if id2 != 2:
        return None
    (fire,) = struct.unpack_from("<i", buf, 14)
    return yaw, pitch, fire


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("binary", help="autoaim 可执行文件路径")
    ap.add_argument("--bag", default=None, help="图像目录（不给就现生成）")
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--no-tracker", action="store_true")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    # ── 1. 准备图像 ────────────────────────────────────────
    bag = args.bag or os.path.join("/tmp", "autoaim_e2e_bag")
    if not os.path.isdir(bag) or not os.listdir(bag):
        print(f"[e2e] 生成合成包 → {bag}")
        subprocess.run([sys.executable, os.path.join(here, "make_test_bag.py"), bag,
                        "--frames", "240"], check=True)

    # ── 2. 开 PTY ─────────────────────────────────────────
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print(f"[e2e] 假串口从设备: {slave_name}")

    # 必须和 make_test_bag.py 里的相机/几何参数严格一致，
    # 否则 PnP 解出来的距离就是错的，测试会误报。
    cmd = [args.binary, "--replay", bag, "--device", slave_name, "--tx-hz", "100",
           "--fx", "3478", "--fy", "3478", "--cx", "720", "--cy", "540",
           "--armor-w", "132", "--armor-h", "57", "--cam-pitch", "-3", "--cam-roll", "2.5"]
    if not args.no_tracker:
        cmd += ["--enable-tracker", "--rotation", "carousel"]

    print(f"[e2e] 启动: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)

    # ── 3. 扮演下位机 ──────────────────────────────────────
    buf = bytearray()
    n_cmd = 0
    n_bad = 0
    yaws = []
    fires = 0
    t0 = time.time()
    last_fb = 0.0

    try:
        while time.time() - t0 < args.secs:
            # 读指令
            try:
                chunk = os.read(master, 512)
            except OSError:
                break
            if chunk:
                buf.extend(chunk)
                while len(buf) >= SEND_LEN:
                    parsed = parse_command(bytes(buf[:SEND_LEN]))
                    if parsed is None:
                        n_bad += 1
                        buf.clear()
                        break
                    yaw, pitch, fire = parsed
                    n_cmd += 1
                    if fire:
                        fires += 1
                    # 只统计前 1 秒之后的数据（跳过启动瞬态）
                    if time.time() - t0 > 1.0:
                        yaws.append(yaw)
                    del buf[:SEND_LEN]

            # 50Hz 回反馈
            now = time.time()
            if now - last_fb > 0.02:
                last_fb = now
                os.write(master, pack_feedback(0.0, 0.0, 0.0, 0, 3, 22.0))

            if proc.poll() is not None:
                print("[e2e] autoaim 提前退出")
                break
    finally:
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
        os.close(master)
        os.close(slave)

    # ── 4. 判定 ───────────────────────────────────────────
    print("\n[e2e] autoaim 输出:")
    for line in (out or "").splitlines()[-14:]:
        print("   ", line)

    print(f"\n[e2e] 收到 {n_cmd} 条指令帧，畸形 {n_bad} 条，开火请求 {fires} 次")

    ok = True
    if n_cmd < 100:
        print(f"  ✗ 指令帧太少（{n_cmd}）—— 发送线程没跑起来？")
        ok = False
    if n_bad > 0:
        print(f"  ✗ 有 {n_bad} 条畸形帧")
        ok = False

    if yaws:
        span = max(yaws) - min(yaws)
        print(f"  yaw 指令范围 {min(yaws):+.2f}° ~ {max(yaws):+.2f}°，跨度 {span:.2f}°")
        # 转盘模式下装甲板在 5m 处绕半径 0.3m 公转 → yaw 摆幅约 ±3.4°
        if span < 1.0:
            print("  ✗ yaw 几乎不动 —— 检测/解算/瞄准链路可能断了")
            ok = False
        else:
            print("  ✓ yaw 指令随目标运动")
    else:
        print("  ✗ 没有收集到 yaw 数据")
        ok = False

    print("\n" + ("端到端测试通过" if ok else "端到端测试失败"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
