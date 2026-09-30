#!/usr/bin/env python3
"""
生成合成测试包：一块绕中心公转的装甲板，按已知位姿投影成图像序列。

用途：在没有车、没有相机的情况下端到端验证整条流水线
（检测 → PnP → 坐标变换 → 跟踪 → 瞄准）。

生成的图像是"真值已知"的：脚本知道装甲板在云台系下的确切位置，
所以可以用它检查 C++ 那边解算出来的距离/角度对不对。

用法:
    python3 tools/make_test_bag.py <输出目录> [--frames 200] [--mode carousel|self_spin]

配套: tools/fake_lower.py（假下位机）、autoaim --replay <输出目录>
"""

import argparse
import math
import os

import cv2
import numpy as np

# ── 相机（12mm 镜头 + IMX273，见 README 关于焦距的说明）──────────
FX = FY = 3478.0
CX, CY = 720.0, 540.0
W, H = 1440, 1080

# ── 装甲板（灯条几何，不是板子外形）────────────────────────────
ARMOR_W_MM = 132.0   # 两灯条中心距
ARMOR_H_MM = 57.0    # 灯条长度
BAR_THICK_MM = 16.0  # 灯条本身的宽度

# ── 场景 ────────────────────────────────────────────────────
CENTER = np.array([5.0, 0.0, 0.0])   # 旋转中心（云台系，前/左/上）
RADIUS = 0.30                        # 公转半径
OMEGA = 2.0                          # rad/s（约 3 秒一圈）
ZA = 0.10                            # 装甲板高度

# ── 外参：相机相对云台俯 15° ─────────────────────────────────
CAM_PITCH_DEG = -3.0   # 只能取小值：12mm 镜头垂直半FOV仅 8.9°，俯太多目标就出画面了
# 相机安装滚转。不是可选项 —— 完美轴对齐的灯条投影后是精确矩形，
# findContours 只给 4 个点，会被 findLights 的 contour.size()<5 过滤掉。
# 现实中相机不可能装得完全水平，这里给一点模拟这个必然存在的偏差。
CAM_ROLL_DEG = 2.5
CAM_ORIGIN = np.zeros(3)

# 灯条颜色。两个约束：灰度要 > binary_thres(160)，且 R 与 B 要有差。
BAR_RGB = (255, 180, 180)   # 灰度 ≈ 202


def r_cam2gimbal(pitch_deg=CAM_PITCH_DEG, yaw_deg=0.0, roll_deg=0.0):
    """必须与 modules/solver/coord_transform.cpp 的实现一致。"""
    R0 = np.array([[0.0, 0.0, 1.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0]])
    y, p, r = map(math.radians, (yaw_deg, pitch_deg, roll_deg))

    def Rz(a):
        return np.array([[math.cos(a), -math.sin(a), 0],
                         [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])

    def Ry(a):
        return np.array([[math.cos(a), 0, math.sin(a)], [0, 1.0, 0],
                         [-math.sin(a), 0, math.cos(a)]])

    def Rx(a):
        return np.array([[1.0, 0, 0], [0, math.cos(a), -math.sin(a)],
                         [0, math.sin(a), math.cos(a)]])

    # 注意 -pitch：配置约定负值 = 向下俯，而绕 +y 转 +φ 才是向下
    return Rz(y) @ Ry(-p) @ Rx(r) @ R0


def armor_pose(state_yaw, radius):
    """给定状态 yaw，返回装甲板中心与姿态矩阵（都在云台系）。

    几何关系来自 target_state.h：xa = xc - r*cos(yaw)。
    装甲板法线朝外（背离旋转中心）。
    """
    c = np.array([math.cos(state_yaw), math.sin(state_yaw), 0.0])
    center = CENTER - radius * c
    normal = -c                                # 朝外
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, normal)
    R = np.column_stack([normal, left, up])    # 列 = 模型的 x/y/z 在云台系下的方向
    return center, R


def project(pts_gimbal, R_cg):
    """云台系点 → 像素。返回 (N,2) 和一维有效掩码。"""
    R_cam = R_cg.T                      # 云台系 → 相机系
    p_cam = (R_cam @ (np.asarray(pts_gimbal) - CAM_ORIGIN).T).T
    z = p_cam[:, 2]
    ok = z > 0.05
    uv = np.zeros((len(p_cam), 2))
    safe = np.where(ok, z, 1.0)
    uv[:, 0] = FX * p_cam[:, 0] / safe + CX
    uv[:, 1] = FY * p_cam[:, 1] / safe + CY
    return uv, ok


def render(state_yaw, R_cg, radius):
    img = np.zeros((H, W, 3), np.uint8)   # RGB
    center, R = armor_pose(state_yaw, radius)

    hy = ARMOR_W_MM / 2 / 1000.0
    hz = ARMOR_H_MM / 2 / 1000.0
    ht = BAR_THICK_MM / 2 / 1000.0

    # 两根灯条。模型里 y 是"左"，z 是"上"，x 是法线。
    #
    # 灯条的粗细必须画在【装甲板平面内】（沿模型 y），不能沿法线 x ——
    # 沿法线的话它正对相机、投影后被压成一条线，什么都检测不到。
    # 真实的灯管就是贴在板面上的。
    for side in (+1, -1):
        y_lo = side * hy - ht
        y_hi = side * hy + ht
        corners_model = [
            np.array([0.0, y_lo, -hz]),
            np.array([0.0, y_hi, -hz]),
            np.array([0.0, y_hi, +hz]),
            np.array([0.0, y_lo, +hz]),
        ]
        pts_g = [(R @ cm + center) for cm in corners_model]
        uv, ok = project(pts_g, R_cg)
        if not ok.all():
            continue
        poly = np.round(uv).astype(np.int32)
        cv2.fillConvexPoly(img, poly, BAR_RGB)

    # 轻微模糊，模拟镜头 —— 也让轮廓点数足够（轴对齐矩形只有 4 个点，
    # 会被 findLights 的 contour.size()<5 过滤掉）
    img = cv2.GaussianBlur(img, (3, 3), 0.8)
    return img, center


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--frames", type=int, default=200)
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--mode", choices=["carousel", "self_spin"], default="carousel")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    R_cg = r_cam2gimbal(roll_deg=CAM_ROLL_DEG)

    r_eff = RADIUS if args.mode == "carousel" else 0.0
    dt = 1.0 / args.fps
    truth = []
    for i in range(args.frames):
        t = i * dt
        yaw = OMEGA * t
        img, center = render(yaw, R_cg, r_eff)
        cv2.imwrite(os.path.join(args.outdir, f"{i:06d}.png"),
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        truth.append((t, yaw, *center))

    # 真值写成 csv，方便和 C++ 解算结果对照
    with open(os.path.join(args.outdir, "truth.csv"), "w") as f:
        f.write("t,yaw,xa,ya,za\n")
        for t, yaw, x, y, z in truth:
            f.write(f"{t:.4f},{yaw:.6f},{x:.6f},{y:.6f},{z:.6f}\n")

    print(f"生成 {args.frames} 帧 → {args.outdir}")
    print(f"  模式={args.mode}  半径={r_eff}m  角速度={OMEGA}rad/s")
    print(f"  装甲板中心真值（首帧）: {truth[0][2]:.4f} {truth[0][3]:.4f} {truth[0][4]:.4f}")
    print(f"  相机 fx={FX:.0f}  装甲板在 5m 处约占 "
          f"{ARMOR_W_MM/1000/5.0*FX:.0f}px 宽")


if __name__ == "__main__":
    main()
