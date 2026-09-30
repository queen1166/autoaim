#!/usr/bin/env python3
"""
相机内参标定 —— 棋盘格照片 → --fx --fy --cx --cy

── 为什么必须做 ──────────────────────────────────────────────
PnP 解出来的【绝对距离】直接由焦距决定。默认值 1000/720/540
（app/config.h:75）是纯占位符。

README 里那张蒙特卡洛表讲的是"6mm 镜头下靶板只占 49px，距离估计是
病态的"——那是光学问题，换镜头能解决。但【焦距填错】不一样：它是
一个系统性比例误差，5m 处可能读出 4m 或 6m，而且 aim_solver 会拿这个
距离去算弹道和提前量。这类误差调任何参数都补不回来。

── 用法 ─────────────────────────────────────────────────────
    # 1. 打印一张棋盘格，务必贴在硬板/玻璃上
    #    （不平整的纸标出来的内参是错的，而且错得很隐蔽）
    # 2. 举着它对着相机，前后左右上下各拍一些，15~30 张
    # 3. 全部丢进一个目录
    python3 tools/calib_intrinsic.py --dir calib_imgs --cols 9 --rows 6 --square-mm 25

    # 输出的 --fx/--fy/--cx/--cy 可以直接复制粘贴到 autoaim 命令行

⚠️ --cols / --rows 是【内角点】数，不是方格数。
   一张 10×7 格的棋盘，内角点是 9×6。数错了会一张都找不到。
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np


def find_images(directory: str):
    """列出目录下的图像。扩充名都要覆盖 —— 相机导出的可能是 .JPG。"""
    out = []
    for ext in ("png", "jpg", "jpeg", "bmp", "tif", "tiff"):
        for pattern in (f"*.{ext}", f"*.{ext.upper()}"):
            out += glob.glob(os.path.join(directory, pattern))
    return sorted(set(out))


def build_object_points(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """棋盘格角点在"标定板坐标系"下的理论坐标。

    约定：板子摊平在 z=0 平面上，角点从 (0,0) 开始按方格边长铺开。

    单位用【米】而不是毫米 —— 这样 calibrateCamera 输出的 tvec 单位
    也是米，和项目内部（modules/solver）的单位一致，不会换算错。
    注意 fx/fy/cx/cy 是像素单位，和这里选什么长度单位无关。
    """
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm / 1000.0
    return objp


def detect_corners(gray, cols, rows):
    """在一张图上找内角点，成功则做亚像素精化。

    为什么必须做 cornerSubPix：findChessboardCorners 给的是整数像素，
    精度约 ±0.5px。标定要解的是焦距，fx 的精度直接受角点定位精度影响，
    整数像素差一个点，fx 就差百分之几。亚像素能压到 0.1px 量级。
    """
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found:
        return None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return corners


def per_view_error(objp, corners, K, dist, rvec, tvec) -> float:
    """这一张图的 RMS 重投影误差（像素）。

    含义：把标定出来的内外参重新投影回图像，看和实际检测到的角点
    差多少像素。这是判断"这一张是不是拍糊了/板子弯了"的直接指标。
    """
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    return float(cv2.norm(corners, proj, cv2.NORM_L2) / len(proj))


def calibrate(obj_points, img_points, size):
    """跑一次 calibrateCamera，返回 (rms, K, dist, rvecs, tvecs)。"""
    return cv2.calibrateCamera(obj_points, img_points, size, None, None)


def main():
    ap = argparse.ArgumentParser(
        description="棋盘格内参标定（输出可直接用的 --fx --fy --cx --cy）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True, help="棋盘格图像目录")
    ap.add_argument("--cols", type=int, required=True,
                    help="【内角点】列数 = 方格列数 - 1")
    ap.add_argument("--rows", type=int, required=True,
                    help="【内角点】行数 = 方格行数 - 1")
    ap.add_argument("--square-mm", type=float, required=True,
                    help="单个方格的实测边长（毫米）。拿尺子量，别信打印标称值")
    ap.add_argument("--max-error", type=float, default=1.0,
                    help="单张图重投影误差超过它就从标定中剔除（像素），默认 1.0；"
                         "设 0 关闭剔除")
    ap.add_argument("--save-undistort", metavar="输出图",
                    help="把第一张图去畸变后存到这里，肉眼确认标定结果")
    args = ap.parse_args()

    paths = find_images(args.dir)
    if not paths:
        print(f"✗ {args.dir} 里没有图像", file=sys.stderr)
        return 1
    print(f"找到 {len(paths)} 张图，开始找 {args.cols}×{args.rows} 内角点…\n")

    objp = build_object_points(args.cols, args.rows, args.square_mm)
    obj_points, img_points, used_paths = [], [], []
    size = None
    skipped_size = 0

    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"  [读不到] {os.path.basename(p)}")
            continue

        # 所有图的分辨率必须一致，否则标定出来的 cx/cy 没有意义
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif (img.shape[1], img.shape[0]) != size:
            skipped_size += 1
            print(f"  [分辨率不同，跳过] {os.path.basename(p)} "
                  f"{img.shape[1]}x{img.shape[0]} ≠ {size[0]}x{size[1]}")
            continue

        corners = detect_corners(img, args.cols, args.rows)
        if corners is None:
            print(f"  [没找到棋盘] {os.path.basename(p)}")
            continue

        obj_points.append(objp)
        img_points.append(corners)
        used_paths.append(p)
        print(f"  ✓ {os.path.basename(p)}")

    print()
    if len(obj_points) < 4:
        print(f"✗ 只找到 {len(obj_points)} 张有效图，至少需要 4 张（建议 15 张以上）",
              file=sys.stderr)
        print("  常见原因：--cols/--rows 数的是方格数而不是内角点数", file=sys.stderr)
        return 1

    if skipped_size:
        print(f"  注意：{skipped_size} 张图因分辨率不一致被跳过\n")

    # ── 第一次标定 ───────────────────────────────────────────
    rms, K, dist, rvecs, tvecs = calibrate(obj_points, img_points, size)

    # ── 剔除离群图后重标 ─────────────────────────────────────
    # 为什么要这一步：一张拍糊了、或者板子被手持弯了的图，会把 fx 拉偏，
    # 而它自己在总 RMS 里只占几十分之一，看不出来。单独看每张的误差就能
    # 揪出来。标准做法是剔掉超过阈值的再标一次。
    if args.max_error > 0 and len(obj_points) > 4:
        errs = [per_view_error(obj_points[i], img_points[i], K, dist,
                               rvecs[i], tvecs[i])
                for i in range(len(obj_points))]
        keep = [i for i, e in enumerate(errs) if e <= args.max_error]

        if len(keep) < len(obj_points) and len(keep) >= 4:
            dropped = [used_paths[i] for i in range(len(errs)) if i not in keep]
            print(f"剔除 {len(dropped)} 张离群图（重投影误差 > {args.max_error}px）：")
            for p in dropped:
                print(f"  - {os.path.basename(p)}")
            print()

            obj_points = [obj_points[i] for i in keep]
            img_points = [img_points[i] for i in keep]
            used_paths = [used_paths[i] for i in keep]
            rms, K, dist, rvecs, tvecs = calibrate(obj_points, img_points, size)

    # ── 逐张误差表（剩下的这些） ─────────────────────────────
    errs = [per_view_error(obj_points[i], img_points[i], K, dist,
                           rvecs[i], tvecs[i])
            for i in range(len(obj_points))]
    print("逐张 RMS 重投影误差（像素）：")
    for p, e in sorted(zip(used_paths, errs), key=lambda t: -t[1]):
        flag = "  ← 偏大" if e > args.max_error else ""
        print(f"  {os.path.basename(p):<28} {e:6.3f}{flag}")
    print()

    # ── 结果 ────────────────────────────────────────────────
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    print("═" * 62)
    print(f"  分辨率        {size[0]} × {size[1]}")
    print(f"  fx / fy       {fx:.3f} / {fy:.3f}   像素")
    print(f"  cx / cy       {cx:.3f} / {cy:.3f}   像素")
    print(f"  RMS 重投影    {rms:.4f} px    （用 {len(obj_points)} 张图）")
    print("═" * 62)
    print()

    # RMS 的判断标准：< 0.3px 很好，0.3~0.8 可用，> 1.0 说明有问题
    if rms > 1.0:
        print("⚠️  RMS > 1.0px。常见原因：板子不平（用纸打印的？）、")
        print("    照片拍糊了、棋盘格尺寸量错了、或者图太少/角度太单一。")
        print("    这个结果直接用的话，PnP 距离会有可观的误差。\n")
    elif rms > 0.5:
        print("⚠️  RMS 偏高但可能可用。有条件的话重拍一组，让板子覆盖")
        print("    画面各个区域（尤其是四个角），而不只是中间。\n")

    # 长宽比异常往往是"非正方形像素"或者数据有问题
    if abs(fx - fy) / max(fx, fy) > 0.02:
        print(f"⚠️  fx 和 fy 差了 {abs(fx-fy)/max(fx,fy)*100:.1f}%。"
              f"正常相机应该很接近。\n")

    print("── 复制这一行到 autoaim 命令行 ──────────────────────────")
    print(f"  --fx {fx:.2f} --fy {fy:.2f} --cx {cx:.2f} --cy {cy:.2f}")
    print()

    # ── 畸变系数 ────────────────────────────────────────────
    # ⚠️ 这里有个项目现状要说清楚：dist_coeffs 在 app/config.h:78 里
    #    是个 vector，但 config.cpp 的 parseArgs【没有】对应的命令行开关。
    #    也就是说畸变系数目前只能改代码，不能从命令行传。
    d = dist.ravel()
    print("── 畸变系数 ────────────────────────────────────────────")
    print(f"  k1={d[0]:+.6f}  k2={d[1]:+.6f}  p1={d[2]:+.6f}  "
          f"p2={d[3]:+.6f}  k3={d[4]:+.6f}")

    if max(abs(d[0]), abs(d[1])) < 0.01:
        print("  畸变很小，保持 config.h 里的全 0 即可。")
    else:
        print("  ⚠️ 畸变不可忽略，但【命令行没有传它的开关】。")
        print("     要启用的话，改 app/config.h:78 这一行：")
        print(f"       std::vector<double> dist_coeffs = "
              f"{{{d[0]:.6f}, {d[1]:.6f}, {d[2]:.6f}, {d[3]:.6f}, {d[4]:.6f}}};")
        print("     （做完记得重新编译）")
    print()

    # ── 分辨率提醒 ──────────────────────────────────────────
    print("── 注意 ────────────────────────────────────────────────")
    print("  这组内参只对【标定时的分辨率】成立。如果运行 autoaim 时")
    print("  相机输出的分辨率不是 "
          f"{size[0]}×{size[1]}，要么改回去，要么重新标一次。")
    print("  （分辨率翻倍的话 fx/fy/cx/cy 也大致翻倍，但那是近似，不是正解）")

    # ── 可选：去畸变存图，肉眼验证 ──────────────────────────
    if args.save_undistort:
        img = cv2.imread(used_paths[0])
        if img is not None:
            # getOptimalNewCameraMatrix 顺便裁掉去畸变产生的黑边
            newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, size, 1, size)
            fixed = cv2.undistort(img, K, dist, None, newK)
            x, y, w, h = roi
            if w > 0 and h > 0:
                fixed = fixed[y:y + h, x:x + w]
            cv2.imwrite(args.save_undistort, fixed)
            print(f"\n去畸变后的样例已存到 {args.save_undistort}"
                  f"（看直线还是不是直的）")

    return 0


if __name__ == "__main__":
    sys.exit(main())
