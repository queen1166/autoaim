#!/usr/bin/env python3
"""
相机外参反解 —— "云台角度 + 靶心像素" → --cam-yaw --cam-pitch --cam-roll

── 为什么必须做 ──────────────────────────────────────────────
app/config.h:91 的 cam_pitch_deg = -15 是占位符。而 README 特意点了名：

    "pitch 的符号必须实测：上游仓库里从没算过 pitch，没有可继承的约定。"

符号搞反的现象是云台朝反方向抬 —— 而你在现场只有一次机会。

README 指定的方法就是本工具做的事：
    云台扫几个角度，记录靶心像素，做最小二乘反解。
（不要上 cv::calibrateHandEye，5m 靶用不上那个精度。）

── 模型 ──────────────────────────────────────────────────────
R_cam2gimbal 的定义【逐字节抄自】modules/solver/coord_transform.cpp。
本文件里的 r_cam2gimbal() 必须和它完全一致，否则标出来的外参是错的。
--selftest 会拿 make_test_bag.py 里的同一份实现做交叉验证。

对第 i 个样本，设靶板在世界（= 云台零位）下的方向角为 (az, el)，
云台当前指向 (Y_i, P_i)（来自反馈帧），靶心像素是 (u_i, v_i)：

    像素 → 相机系射线   ray_cam  = normalize(K⁻¹ [u, v, 1])
    先转到【云台零位系】 ray_0    = R_cam2gimbal · ray_cam
    再套上云台当前的朝向 ray_world = R_gimbal(Y_i, P_i) · ray_0

    该方向就是靶板在世界系下的方向：

        angles(ray_world) = (az, el)

其中 R_gimbal(Y, P) = Rz(Y)·Ry(-P)。它不是随便写的 —— 验证：
    Rz(Y)·Ry(-P)·[1,0,0] = [cosP·cosY, cosP·sinY, sinP]
    正好等于 coord_transform.cpp 的 gimbalDirFromAngles(Y, P)。
拿 Ry(-P) 而不是 Ry(+P) 也是同一套约定（配置里 pitch 负值 = 向下）。

5 个未知量：挂载角 (yaw, pitch, roll) + 靶板方向 (az, el) —— 前 3 个就是
我们要的答案，后 2 个是"顺手一起解出来的"冗余参数。
每个样本给 2 个方程，所以【至少 3 个样本】就能解。

关键好处：**完全不需要知道靶板离多远**。
    angles() 只依赖向量的方向、跟长度无关（coord_transform.cpp 里
    用的就是 atan2(y,x) 和 atan2(z,hypot(x,y))），所以 PnP 的尺度问题
    在这里不存在。靶板放哪、多远，都不用量。

── 为什么不能用"angles(R·ray) = (az-Y, el-P)"这种简化写法 ──────
乍看可以省掉 R_gimbal，直接说"靶板在云台当前朝向系下的角度是
(az-Y, el-P)"。但那个分解【只是近似】：绕 y 轴转 -P 并不能简单地从
仰角里减掉 P，除非目标正好在 y-z 平面内（方位角为 0）。

实测（本文件 --selftest 用例 2）：靶板方位角 1.5°，云台在 ±9° 范围扫，
这个近似会给 cam_pitch 引入 【-0.23°】 的系统偏差 —— 5m 处约 2cm 的
指向误差，而且残差看起来还很漂亮（0.007°），完全不会报警。
所以这里必须用上面那套精确模型。

── 假设与限制 ────────────────────────────────────────────────
1. 云台扫动期间【靶板不动】。车要在发射位停好。
2. 靶板要【偏离画面中心】才标得准。靶心一直在主点附近的话，roll 会
   变得很弱（绕光轴转几乎不影响光轴本身的方向）。本工具会检查并警告。
3. ★ 这个反问题【条件数很差】—— 残差漂亮绝不代表参数准。
   实测（20 组 0.5px 检测噪声，±9° yaw × ±5° pitch 扫动，28 个样本）：
       残差 RMS     0.008°          ← 看起来完美
       cam_roll     RMS 0.013°
       cam_yaw      RMS 0.150°
       cam_pitch    RMS 0.182°   最差 0.61°
   参数误差是残差的 80 倍。也就是说：**光看残差完全看不出标歪了。**
   → 本工具因此会打印每个参数的【估计标准差】。标准差大的参数就是
     没标住，别硬用。
   → cam_yaw 最勉强：在那套精确模型下它只通过 pitch 扫动的二阶耦合
     才可观测（pitch 不扫的话它精确不可观测，见 --selftest 用例 3）。
     pitch 扫得越少越不可信。想更稳就用 --fix-yaw 0 ——
     "相机在水平方向上顺着云台炮口装"是最常见的做法，把自由度让给
     其它参数反而更稳。
   → 扫动范围越大、样本越多，条件数越好。yaw 尽量多扫几个角度。

── 用法 ──────────────────────────────────────────────────────
    # 0. 先确认实现在这台机器上是对的（不需要任何数据）
    python3 tools/calib_extrinsic.py --selftest

    # 1. 录一段：云台扫几个角度，每个角度停一下
    ./build/tools/tool_hik_record --out bags/calib --secs 60 \
        --device /dev/serial/by-id/usb-xxx --every 5

    # 2. 反解（内参用 calib_intrinsic.py 的结果）
    python3 tools/calib_extrinsic.py --images bags/calib \
        --angles bags/calib/angles.csv \
        --fx 3478 --fy 3478 --cx 720 --cy 540

    # 输出的 --cam-yaw/--cam-pitch/--cam-roll 直接贴到 autoaim 命令行
"""

import argparse
import csv
import glob
import math
import os
import sys

import cv2
import numpy as np

# ─────────────────────────────────────────────────────────────
# 坐标约定 —— 必须与 modules/solver/coord_transform.cpp 逐字对应
# ─────────────────────────────────────────────────────────────

# 名义映射 R0：相机正对前方、无安装角时，光学系 → 云台系。
#   相机 z(前) → 云台 +x(前)
#   相机 x(右) → 云台 -y（云台 +y 是"左"）
#   相机 y(下) → 云台 -z（云台 +z 是"上"）
R0 = np.array([[0.0, 0.0, 1.0],
               [-1.0, 0.0, 0.0],
               [0.0, -1.0, 0.0]])


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def r_cam2gimbal_rad(yaw, pitch, roll):
    """相机系 → 云台系，角度单位【弧度】。

    ⚠️ 注意 -pitch：配置约定 cam_pitch_deg 负值 = 向下俯，
       而绕云台 +y（左）轴转 +φ 才是向下俯。见 coord_transform.cpp:32-38。
    """
    return _Rz(yaw) @ _Ry(-pitch) @ _Rx(roll) @ R0


def r_cam2gimbal(yaw_deg, pitch_deg, roll_deg):
    """角度制版本。就是上面那个包一层。"""
    return r_cam2gimbal_rad(math.radians(yaw_deg),
                            math.radians(pitch_deg),
                            math.radians(roll_deg))


def r_gimbal_rad(yaw, pitch):
    """云台零位系 → 云台当前朝向系。角度单位【弧度】。

    必须和 coord_transform.cpp 的 gimbalDirFromAngles 一致：
        Rz(Y)·Ry(-P)·[1,0,0] = [cosP·cosY, cosP·sinY, sinP]
                             = gimbalDirFromAngles(Y, P)   ✓

    同样用 -P：配置约定 pitch 负值 = 向下俯，而绕云台 +y 转 +φ 才是向下，
    见 coord_transform.cpp:32-38。
    """
    return _Rz(yaw) @ _Ry(-pitch)


def dir_angles(p):
    """云台系方向 → (yaw, pitch)，弧度。

    对应 coord_transform.cpp 的 yawPitchFromGimbalPoint：
        yaw   = atan2(左, 前)
        pitch = atan2(上, 水平距离)

    注意它【只看方向，不看长度】—— 这正是本工具不需要知道距离的原因。
    """
    horizon = math.hypot(p[0], p[1])
    return math.atan2(p[1], p[0]), math.atan2(p[2], horizon)


def dir_from_angles(yaw, pitch):
    """上面的逆：角度 → 单位方向向量。"""
    cp = math.cos(pitch)
    return np.array([cp * math.cos(yaw), cp * math.sin(yaw), math.sin(pitch)])


def wrap_pi(a):
    """把角度折回 ±π。协议没有序列号，yaw 跨 ±π 时残差会突然跳 2π。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ray_from_pixel(u, v, K):
    """像素 → 相机系单位射线。标准针孔模型。"""
    p = np.linalg.solve(K, np.array([u, v, 1.0]))
    return p / np.linalg.norm(p)


# ─────────────────────────────────────────────────────────────
# 求解器：自带 Levenberg-Marquardt，不依赖 scipy
# ─────────────────────────────────────────────────────────────


def _jacobian(fun, x, f0, h=1e-6):
    """数值雅可比（前向差分）。参数是弧度，h=1e-6 约 5.7e-5 度，够小。"""
    J = np.zeros((len(f0), len(x)))
    for i in range(len(x)):
        xp = x.copy()
        xp[i] += h
        J[:, i] = (fun(xp) - f0) / h
    return J


def levenberg_marquardt(fun, x0, max_iter=300):
    """阻尼最小二乘。返回 (x, cost, residual)。

    为什么不用纯高斯-牛顿：初始猜测和真值差得远时，GN 会一步跳到
    发散区。LM 的阻尼项 lam 保证每步都真的让代价下降，代价降不下去
    就加大阻尼、退回接近梯度下降的行为。

    为什么不用 scipy.optimize.least_squares：少一个依赖，现场少一类
    环境问题（本项目的 config.h 开篇就是这个理由）。
    """
    x = np.asarray(x0, float)
    r = fun(x)
    cost = float(r @ r)
    lam = 1e-3

    for _ in range(max_iter):
        J = _jacobian(fun, x, r)
        A = J.T @ J
        g = J.T @ r
        if not np.all(np.isfinite(A)) or not np.all(np.isfinite(g)):
            break

        # 内层：加大 lam 直到这一步确实让代价下降
        improved = False
        dx = np.zeros_like(x)
        for _ in range(20):
            try:
                # 用 diag(A) 做缩放，比直接加 lam*I 更适应各参数量级差异
                dx = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-15), -g)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue

            x_new = x + dx
            r_new = fun(x_new)
            cost_new = float(r_new @ r_new)

            if np.isfinite(cost_new) and cost_new < cost:
                x, r, cost = x_new, r_new, cost_new
                lam = max(lam * 0.3, 1e-12)
                improved = True
                break
            lam *= 10.0

        if not improved:
            break  # 怎么调 lam 都下不去了，收敛（或卡住）
        if np.linalg.norm(dx) < 1e-14 * (1.0 + np.linalg.norm(x)):
            break

    return x, cost, r


# ─────────────────────────────────────────────────────────────
# 靶心检测
# ─────────────────────────────────────────────────────────────


def detect_target_center(gray, thres, n_bars, roi=None, min_area=20):
    """在一张灰度图里找靶板中心。

    做法：二值化 → 连通域 → 按面积取最大的 n_bars 个 → 它们中心的平均。

    为什么是"取最大的 N 个再平均"而不是"所有亮像素求重心"：
    实验室里有顶灯、有反光。全图求重心会被这些亮点拽偏。取最大的
    几个连通域能把它们排掉。

    为什么是"平均各连通域的中心"而不是"按面积加权"：
    靶板是两根灯条，装甲板中心 = 两根灯条中心的【中点】。
    它俩面积一般接近，等权平均正好就是中点。

    返回 (u, v, 找到的连通域数)；一个都没有时返回 (None, None, 0)。
    """
    if roi is not None:
        x, y, w, h = roi
        gray = gray[y:y + h, x:x + w]
        off = np.array([x, y], float)
    else:
        off = np.zeros(2)

    _, binary = cv2.threshold(gray, thres, 255, cv2.THRESH_BINARY)
    n, _, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)

    # 第 0 个是背景，跳过
    blobs = [(stats[i, cv2.CC_STAT_AREA], centroids[i])
             for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= min_area]
    if not blobs:
        return None, None, 0

    blobs.sort(key=lambda t: -t[0])
    picked = blobs[:n_bars]

    # 面积加权：一根灯条被截断（侧对相机）时，让完整的那根主导一点。
    # 用纯等权平均的话，半根灯条会把中心拽偏。
    total = sum(a for a, _ in picked)
    center = sum((a * c for a, c in picked), start=np.zeros(2)) / total

    return center[0] + off[0], center[1] + off[1], len(blobs)


def load_image_gray(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    return img


# ─────────────────────────────────────────────────────────────
# 样本读取
# ─────────────────────────────────────────────────────────────


def read_angles_csv(path):
    """读 angles.csv（tool_hik_record --device 生成）。

    格式: file,yaw_deg,pitch_deg,u,v
    u/v 留空表示"自己从图像里找"。
    """
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path} 是空文件")
        need = {"yaw_deg", "pitch_deg"}
        missing = need - set(reader.fieldnames)
        if missing:
            raise ValueError(
                f"{path} 缺少列 {sorted(missing)}；"
                f"实际列 = {reader.fieldnames}")

        for i, row in enumerate(reader):
            try:
                yaw = float(row["yaw_deg"])
                pitch = float(row["pitch_deg"])
            except (TypeError, ValueError):
                print(f"  跳过第 {i+2} 行（角度解析失败）")
                continue

            u = row.get("u") or ""
            v = row.get("v") or ""
            u = float(u) if u.strip() else None
            v = float(v) if v.strip() else None

            rows.append({
                "file": (row.get("file") or "").strip(),
                "yaw": yaw,
                "pitch": pitch,
                "u": u,
                "v": v,
                "row": i + 2,
            })
    return rows


# ─────────────────────────────────────────────────────────────
# 核心解算
# ─────────────────────────────────────────────────────────────

PARAM_NAMES = ["cam_yaw", "cam_pitch", "cam_roll", "az_world", "el_world"]


def make_residual(rays, gimbal_yaw, gimbal_pitch, fixed):
    """构造残差函数。参数在【自由参数子空间】里，返回 (residual, free_idx, expand)。

    残差是 2N 维向量，每个样本两个分量：
        预测的靶板世界方向角  -  当前待估的靶板世界方向角 (az, el)

    参数含义（弧度）：0=cam_yaw 1=cam_pitch 2=cam_roll 3=az_world 4=el_world
    """
    free_idx = [i for i in range(5) if i not in fixed]
    n = len(rays)
    R_gimbal = [r_gimbal_rad(gimbal_yaw[i], gimbal_pitch[i]) for i in range(n)]

    def expand(free):
        full = np.zeros(5)
        for i, v in fixed.items():
            full[i] = v
        for k, i in enumerate(free_idx):
            full[i] = free[k]
        return full

    def residual(free):
        p = expand(free)
        R_mount = r_cam2gimbal_rad(p[0], p[1], p[2])
        out = np.empty(2 * n)
        for i in range(n):
            # 相机系 → 云台零位系 → 世界系，两步都是精确旋转，没有近似
            y, pt = dir_angles(R_gimbal[i] @ R_mount @ rays[i])
            out[2 * i] = wrap_pi(y - p[3])
            out[2 * i + 1] = pt - p[4]
        return out

    return residual, free_idx, expand


def solve_extrinsic(rays, gimbal_yaw, gimbal_pitch, fixed):
    """最小二乘反解。

    rays          (N,3) 相机系单位射线
    gimbal_yaw    (N,)  云台当前 yaw（弧度）
    gimbal_pitch  (N,)  云台当前 pitch（弧度）
    fixed         dict 参数下标 → 固定值（弧度）。--fix-* 用

    返回 (params_rad(5,), cost, residual)
    """
    residual, free_idx, expand = make_residual(rays, gimbal_yaw, gimbal_pitch,
                                               fixed)
    n = len(rays)
    R_gimbal = [r_gimbal_rad(gimbal_yaw[i], gimbal_pitch[i]) for i in range(n)]

    # 初始猜测：假设挂载角全是 0（此时 R_mount = R0），把每个样本算出的
    # 世界方向平均一下，作为靶板方位的初值。
    a0 = [dir_angles(R_gimbal[i] @ R0 @ rays[i]) for i in range(n)]
    # yaw 用圆均值：跨 ±π 时直接取算术平均会得到完全错误的结果
    az0 = math.atan2(float(np.mean([math.sin(a[0]) for a in a0])),
                     float(np.mean([math.cos(a[0]) for a in a0])))
    el0 = float(np.mean([a[1] for a in a0]))

    # 多起点：挂载角最可能在小角度范围，但符号方向完全未知，多试几个
    pitch_guesses = [0.0, -15.0, -5.0, -25.0, -10.0]
    yaw_guesses = [0.0, 5.0, -5.0, 10.0, -10.0]
    roll_guesses = [0.0, 3.0, -3.0, 6.0, -6.0]

    best = None
    for k in range(5):
        guess = np.zeros(5)
        guess[0] = math.radians(yaw_guesses[k]) if 0 not in fixed else fixed.get(0, 0)
        guess[1] = math.radians(pitch_guesses[k]) if 1 not in fixed else fixed.get(1, 0)
        guess[2] = math.radians(roll_guesses[k]) if 2 not in fixed else fixed.get(2, 0)
        guess[3] = az0 if 3 not in fixed else fixed.get(3, 0)
        guess[4] = el0 if 4 not in fixed else fixed.get(4, 0)

        x, cost, res = levenberg_marquardt(residual, guess[free_idx])
        if best is None or cost < best[1]:
            best = (expand(x), cost, res)

    return best


def parameter_stderr_deg(rays, gimbal_yaw, gimbal_pitch, fixed, params,
                         cost, res):
    """每个参数的估计标准差（度），以及 JᵀJ 的条件数。

    为什么必须要这个东西：这个反问题条件数很差，残差和参数精度差两个
    数量级（见文件头"假设与限制"第 3 条）。不给标准差的话，用户会看到
    一个漂亮的残差就以为标定很准。

    做法是标准的最小二乘协方差估计：
        cov ≈ σ² (JᵀJ)⁻¹,   σ² = cost / 自由度
        σ_i = sqrt(cov_ii)
    自由度 = 残差维数 - 自由参数个数。

    返回 (stderr_deg(5,), 条件数)。固定住的参数标准差记为 0。
    矩阵奇异时返回 (None, None) —— 调用方要处理这种情况。
    """
    residual, free_idx, _ = make_residual(rays, gimbal_yaw, gimbal_pitch, fixed)
    free = np.array([params[i] for i in free_idx])

    J = _jacobian(residual, free, res)
    JtJ = J.T @ J
    dof = max(len(res) - len(free_idx), 1)

    try:
        cov = (cost / dof) * np.linalg.inv(JtJ)
    except np.linalg.LinAlgError:
        return None, None

    # 条件数：衡量这个反问题有多病态。>1e6 基本就是没标住。
    cond = float(np.linalg.cond(JtJ))

    stderr = np.sqrt(np.abs(np.diag(cov)))
    out = np.zeros(5)
    for k, i in enumerate(free_idx):
        out[i] = math.degrees(stderr[k])
    return out, cond


# ─────────────────────────────────────────────────────────────
# 自检
# ─────────────────────────────────────────────────────────────


def selftest():
    """不需要相机、不需要数据，验证两件事。

    1. 【约定一致性】本文件的 r_cam2gimbal 和仓库里
       tools/make_test_bag.py 的实现必须逐元素相等。
       make_test_bag.py 已经在端到端测试里验证过了，所以它是基准。
       这两份实现分处 Python 和 C++，一旦漂移，标出来的外参就是错的，
       而且错得很隐蔽 —— 必须锁住。

    2. 【求解器正确性】用已知挂载角造一批带噪声的样本，看能不能解回来。
       注意这验证的是 LM 实现和初值策略，不是物理约定（那个由 1 覆盖）。
    """
    ok = True

    # ── 1. 约定一致性 ────────────────────────────────────────
    print("[1] 与 tools/make_test_bag.py 的 R_cam2gimbal 交叉验证")
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import make_test_bag as mtb
    except ImportError as e:
        print(f"  ✗ 导入 make_test_bag 失败: {e}")
        return False

    worst = 0.0
    for yaw in (-180.0, -90.0, -30.0, 0.0, 30.0, 90.0, 179.0):
        for pitch in (-30.0, -15.0, 0.0, 15.0, 30.0):
            for roll in (-10.0, 0.0, 10.0):
                mine = r_cam2gimbal(yaw, pitch, roll)
                theirs = mtb.r_cam2gimbal(pitch_deg=pitch, yaw_deg=yaw,
                                          roll_deg=roll)
                worst = max(worst, float(np.max(np.abs(mine - theirs))))

    if worst > 1e-12:
        print(f"  ✗ 最大元素差 {worst:.3e} —— 两份实现不一致！")
        print("     本文件必须和 coord_transform.cpp / make_test_bag.py 一致。")
        ok = False
    else:
        print(f"  ✓ 105 组角度全部一致（最大元素差 {worst:.1e}）")

    # ── 2. 求解器：多组噪声下的统计 ───────────────────────────
    # 注意：这里用【和求解器同一套方程】造数据，所以它验证的是 LM 实现、
    # 初值策略、自由参数掩码这些代码层面的东西，不是物理约定 ——
    # 物理约定由用例 1（和 make_test_bag.py 交叉验证）覆盖。
    print("\n[2] 求解器：20 组噪声下三参数全解（±9° yaw × ±5° pitch）")
    true_yaw, true_pitch, true_roll = 2.0, -12.0, 3.5
    true_az, true_el = 1.5, -1.0

    K = np.array([[3478.0, 0.0, 720.0],
                  [0.0, 3478.0, 540.0],
                  [0.0, 0.0, 1.0]])
    R_true = r_cam2gimbal(true_yaw, true_pitch, true_roll)
    d_world = dir_from_angles(math.radians(true_az), math.radians(true_el))

    def synth(seed, pitch_list):
        """按正向模型造一批带 0.5px 检测噪声的样本。"""
        r = np.random.RandomState(seed)
        rays, gy, gp = [], [], []
        for yaw_cmd in (-9.0, -6.0, -3.0, 0.0, 3.0, 6.0, 9.0):
            for pitch_cmd in pitch_list:
                Y = math.radians(yaw_cmd)
                P = math.radians(pitch_cmd)
                # 正向：世界方向 → 云台零位系 → 相机系 → 像素
                ray = (r_gimbal_rad(Y, P) @ R_true).T @ d_world
                if ray[2] <= 1e-6:
                    continue
                u = K[0, 0] * ray[0] / ray[2] + K[0, 2]
                v = K[1, 1] * ray[1] / ray[2] + K[1, 2]
                rays.append(ray_from_pixel(u + r.normal(0.0, 0.5),
                                           v + r.normal(0.0, 0.5), K))
                gy.append(Y)
                gp.append(P)
        return np.array(rays), np.array(gy), np.array(gp)

    want = np.array([true_yaw, true_pitch, true_roll])
    errs, rms_list = [], []
    for seed in range(20):
        rays, gy, gp = synth(seed, (-5.0, -2.0, 2.0, 5.0))
        p, c, r = solve_extrinsic(rays, gy, gp, {})
        errs.append([math.degrees(p[0]) - want[0],
                     math.degrees(p[1]) - want[1],
                     math.degrees(p[2]) - want[2]])
        rms_list.append(math.degrees(math.sqrt(c / len(r))))

    errs = np.abs(np.array(errs))
    rms = float(np.mean(rms_list))
    for i, name in enumerate(PARAM_NAMES[:3]):
        print(f"  {name:<10} 真值 {want[i]:+7.2f}   RMS 误差 {errs[:, i].mean():.4f}°"
              f"   最大 {errs[:, i].max():.4f}°")
    print(f"  残差 RMS   {rms:.4f}°")

    # 判据用 RMS 而不是单次最大值：20 个种子里挑最大的，那个数本身也是
    # 个随机量，偏悲观。RMS 才是"这套标定大概准到什么程度"的答案。
    worst_rms = float(errs.mean(axis=0).max())
    worst_max = float(errs.max())
    if worst_rms > 0.3:
        print(f"  ✗ 参数误差 RMS {worst_rms:.4f}° 超过 0.3°")
        ok = False
    else:
        print(f"  ✓ 三参数 RMS 误差都 ≤ 0.3°（最差 {worst_rms:.4f}°）")

    # 这正是文件头"假设与限制"第 3 条警告的事
    print(f"  ⚠ 参数最大误差（{worst_max:.2f}°）是残差（{rms:.4f}°）的 "
          f"{worst_max / max(rms, 1e-12):.0f} 倍")
    print(f"    → 残差漂亮【不代表】参数准。所以本工具会打印每个参数的")
    print(f"      估计标准差，别再只看残差 RMS 了")

    # ── 3. pitch 不扫时 cam_yaw 精确不可观测 ──────────────────
    print("\n[3] 简并：pitch 不扫时 cam_yaw 不可观测（所以扫动必须带上 pitch）")
    r3 = np.random.RandomState(3)
    worst_dy, worst_dp = 0.0, 0.0
    for _ in range(500):
        M = r_cam2gimbal_rad(*r3.uniform(-0.6, 0.6, 3))
        ray = r3.normal(size=3)
        ray /= np.linalg.norm(ray)
        d = r3.uniform(-0.6, 0.6)
        a1, a2 = dir_angles(_Rz(d) @ M @ ray), dir_angles(M @ ray)
        worst_dy = max(worst_dy, abs(wrap_pi(a1[0] - a2[0] - d)))
        worst_dp = max(worst_dp, abs(a1[1] - a2[1]))

    print(f"  Rz(δ) 使 yaw 恰好 +δ（最大偏差 {worst_dy:.1e}），"
          f"pitch 分毫不动（{worst_dp:.1e}）")
    print("  → 「cam_yaw 加 δ」和「靶板方位角 az 加 δ」给出【完全相同】的残差。")
    print("    pitch 不扫时 R_gimbal = Rz(Y)，这个等价对每个样本都成立，")
    print("    所以 cam_yaw 精确不可观测 —— 多起点只会随机落到某一组等价解。")
    print("    扫了 pitch 才能把它打破（用例 2 能解出 cam_yaw 就是靠这个）。")

    if worst_dy > 1e-12 or worst_dp > 1e-12:
        print("  ✗ 简并关系不成立 —— 模型可能写错了")
        ok = False
    else:
        print("  ✓ 简并关系精确成立（误差在浮点精度内）")

    print("\n" + ("自检通过" if ok else "自检失败"))
    return ok


# ─────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser(
        description="外参反解：云台角度 + 靶心像素 → --cam-yaw --cam-pitch --cam-roll",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true",
                    help="验证实现和约定，不需要任何数据")

    ap.add_argument("--angles", help="tool_hik_record 生成的 angles.csv")
    ap.add_argument("--images", help="图像目录（u/v 留空时从这里找靶心）")
    ap.add_argument("--fx", type=float, help="相机内参")
    ap.add_argument("--fy", type=float)
    ap.add_argument("--cx", type=float)
    ap.add_argument("--cy", type=float)

    ap.add_argument("--thres", type=int, default=160,
                    help="找靶心的二值化阈值，默认 160（和 --thres 同义）")
    ap.add_argument("--n-bars", type=int, default=2,
                    help="取几个最大的亮连通域来定中心，默认 2（两根灯条）")
    ap.add_argument("--roi", help="只在图像的这个区域找靶心，格式 x,y,w,h")
    ap.add_argument("--debug-out", help="把找到的靶心画出来存到这个目录，肉眼复核")

    ap.add_argument("--fix-yaw", type=float, help="固定 cam_yaw，不参与优化")
    ap.add_argument("--fix-pitch", type=float, help="固定 cam_pitch")
    ap.add_argument("--fix-roll", type=float, help="固定 cam_roll")

    ap.add_argument("--drop-worst", type=int, default=0,
                    help="先解一次，丢掉残差最大的 n 个样本再解一次")
    args = ap.parse_args()

    if args.selftest:
        return 0 if selftest() else 1

    missing = [n for n in ("angles", "fx", "fy", "cx", "cy")
               if getattr(args, n) is None]
    if missing:
        ap.error("缺参数: " + ", ".join("--" + m for m in missing))

    K = np.array([[args.fx, 0.0, args.cx],
                  [0.0, args.fy, args.cy],
                  [0.0, 0.0, 1.0]])

    roi = None
    if args.roi:
        try:
            roi = tuple(int(t) for t in args.roi.split(","))
            assert len(roi) == 4
        except (ValueError, AssertionError):
            print(f"✗ --roi 格式应是 x,y,w,h，收到 {args.roi!r}", file=sys.stderr)
            return 1

    # ── 读样本 ───────────────────────────────────────────────
    rows = read_angles_csv(args.angles)
    if not rows:
        print("✗ 角度日志里没有有效样本", file=sys.stderr)
        return 1
    print(f"读入 {len(rows)} 个样本")

    rays, gy, gp, labels = [], [], [], []
    n_auto, n_manual, n_missing = 0, 0, 0
    for i, r in enumerate(rows):
        u, v = r["u"], r["v"]

        if u is None or v is None:
            if not args.images:
                print(f"  第 {r['row']} 行 u/v 为空，但没给 --images，跳过")
                continue
            if not r["file"]:
                print(f"  第 {r['row']} 行没有文件名，跳过")
                continue
            path = os.path.join(args.images, r["file"])
            gray = load_image_gray(path)
            if gray is None:
                print(f"  读不到 {path}，跳过")
                continue
            u, v, n_blobs = detect_target_center(gray, args.thres, args.n_bars, roi)
            if u is None:
                print(f"  {r['file']}: 没找到亮斑。两种可能——"
                      f"阈值 {args.thres} 太高，或者靶板这时在画面外")
                n_missing += 1
                continue
            if n_blobs < args.n_bars:
                print(f"  {r['file']}: 只找到 {n_blobs} 个亮斑（期望 "
                      f"{args.n_bars} 个），中心可能偏半块灯条间距")
            n_auto += 1

            if args.debug_out:
                os.makedirs(args.debug_out, exist_ok=True)
                vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                cv2.drawMarker(vis, (int(round(u)), int(round(v))),
                               (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
                if roi:
                    cv2.rectangle(vis, (roi[0], roi[1]),
                                  (roi[0] + roi[2], roi[1] + roi[3]),
                                  (0, 255, 0), 2)
                cv2.imwrite(os.path.join(args.debug_out, r["file"]), vis)
        else:
            n_manual += 1

        rays.append(ray_from_pixel(u, v, K))
        gy.append(math.radians(r["yaw"]))
        gp.append(math.radians(r["pitch"]))
        labels.append(r["file"] or f"row{r['row']}")

    if len(rays) < 3:
        print(f"✗ 只有 {len(rays)} 个可用样本，至少需要 3 个", file=sys.stderr)
        return 1

    rays = np.array(rays)
    gy = np.array(gy)
    gp = np.array(gp)

    print(f"  自动检测靶心 {n_auto} 个，手工给定 {n_manual} 个"
          + (f"，【丢了 {n_missing} 个】" if n_missing else "") + "\n")
    if n_missing:
        # 丢帧不只是少几个样本：它会把云台的扫动范围截短，条件数变差。
        print(f"⚠️ 有 {n_missing} 帧没找到靶板。最可能的原因是靶板出了画面 ——")
        print("   12mm 镜头的垂直半 FOV 只有 8.8°，相机俯角一大，"
              "云台稍低一点靶板就出去了。")
        print("   这会让实际参与解算的扫动范围变小，参数更不稳。"
              "重录时把云台角度范围收窄到靶板始终在画面内。\n")

    # ── 可观测性检查 ─────────────────────────────────────────
    # 靶心一直贴着主点的话，绕光轴的 roll 是解不出来的。先提醒。
    centers = []
    for i in range(len(rays)):
        # 反推像素：用射线比例即可，不需要真的解
        centers.append((rays[i][0] / rays[i][2] * args.fx + args.cx,
                        rays[i][1] / rays[i][2] * args.fy + args.cy))
    centers = np.array(centers)
    radii = np.hypot(centers[:, 0] - args.cx, centers[:, 1] - args.cy)

    print(f"靶心到主点的距离：中位 {np.median(radii):.0f}px  "
          f"最大 {radii.max():.0f}px")
    if np.median(radii) < 40:
        print("⚠️ 靶心基本一直在主点附近 —— cam_roll 接近不可观测，")
        print("   解出来的 roll 会不可信。让靶板在画面里明显偏离中心再标。\n")

    # 云台扫动范围：太小的话 az/el 和挂载角分不开
    span_yaw = math.degrees(gy.max() - gy.min())
    span_pitch = math.degrees(gp.max() - gp.min())
    print(f"云台扫动范围：yaw {span_yaw:.1f}°  pitch {span_pitch:.1f}°")
    if span_yaw < 5.0:
        print("⚠️ yaw 扫动范围太小（< 5°）—— 挂载角和靶板方向会耦合在一起，")
        print("   解不唯一。让云台多扫几个角度，每个角度都要停稳。\n")

    # ── 求解 ────────────────────────────────────────────────
    fixed = {}
    if args.fix_yaw is not None:
        fixed[0] = math.radians(args.fix_yaw)
    if args.fix_pitch is not None:
        fixed[1] = math.radians(args.fix_pitch)
    if args.fix_roll is not None:
        fixed[2] = math.radians(args.fix_roll)

    def report(params, res, tag):
        rms = math.degrees(math.sqrt(float(res @ res) / len(res)))
        worst = math.degrees(np.max(np.abs(res)))
        print(f"  {tag}: 残差 RMS {rms:.4f}°  "
              f"最大 {worst:.4f}°  （{len(res)//2} 个样本）")
        return rms, worst

    params, cost, res = solve_extrinsic(rays, gy, gp, fixed)
    print("解算结果：")
    rms_deg, worst_deg = report(params, res, "全部样本")

    # ── 丢离群样本重解 ──────────────────────────────────────
    if args.drop_worst > 0 and len(rays) > args.drop_worst + 3:
        # 每个样本的残差是 2 个分量，按样本取模长排序
        per_sample = np.linalg.norm(res.reshape(-1, 2), axis=1)
        drop = set(np.argsort(-per_sample)[:args.drop_worst].tolist())
        keep = [i for i in range(len(rays)) if i not in drop]

        print(f"\n  丢掉残差最大的 {len(drop)} 个样本：")
        for i in sorted(drop):
            print(f"    - {labels[i]}  ({math.degrees(per_sample[i]):.3f}°)")

        params2, cost2, res2 = solve_extrinsic(rays[keep], gy[keep], gp[keep], fixed)
        rms2, worst2 = report(params2, res2, "剔除后")
        if rms2 < rms_deg:
            params, cost, res = params2, cost2, res2
            # 数组也要一起换掉，否则下面算标准差用的还是旧数据
            rays, gy, gp = rays[keep], gy[keep], gp[keep]
            labels = [labels[i] for i in keep]
            rms_deg, worst_deg = rms2, worst2
            print("  → 采用剔除后的结果")
        else:
            print("  → 剔除后反而更差，保留原结果")
            print("    （这通常说明那几帧不是坏数据，而是模型在某个角度不成立）")

    # ── 逐样本残差 ──────────────────────────────────────────
    print("\n逐样本残差（度）：")
    per_sample = np.linalg.norm(res.reshape(-1, 2), axis=1)
    for i in np.argsort(-per_sample):
        mark = "  ← 偏大" if math.degrees(per_sample[i]) > 0.5 else ""
        print(f"  {labels[i]:<20} {math.degrees(per_sample[i]):7.3f}{mark}")

    # ── 参数标准差 ──────────────────────────────────────────
    # 这一步不是锦上添花。这个反问题条件数很差（见文件头），残差漂亮
    # 完全不代表参数准 —— 不报标准差的话，用户会拿着一个 0.3° 误差的
    # cam_pitch 以为它准到 0.01°。
    stderr, cond = parameter_stderr_deg(rays, gy, gp, fixed, params, cost, res)
    if stderr is None:
        print("\n⚠️ 估不出参数标准差（JᵀJ 奇异）—— 样本太少或扫动范围太小。")
        print("   这不是小事：说明有参数在互相抵消，解出来的值不可信。")
    else:
        print("\n参数估计标准差（1σ，单位度）：")
        for i, name in enumerate(PARAM_NAMES[:3]):
            if i in fixed:
                print(f"  {name:<10} 固定为 {math.degrees(params[i]):+7.2f}°"
                      f"   （你指定的，不参与解算）")
            else:
                bar = "  ← 偏大，基本没标住" if stderr[i] > 0.5 else ""
                print(f"  {name:<10} ± {stderr[i]:.3f}°{bar}")
        note = "  ← 病态：参数之间在互相抵消" if cond > 1e6 else ""
        print(f"  JᵀJ 条件数 {cond:.2e}{note}")

    # ── 输出 ────────────────────────────────────────────────
    yaw_d, pitch_d, roll_d = (math.degrees(params[i]) for i in range(3))
    az_d, el_d = math.degrees(params[3]), math.degrees(params[4])

    print("\n" + "═" * 62)
    if stderr is not None:
        print(f"  cam_yaw    {yaw_d:+8.3f} °  ± {stderr[0]:.3f}")
        print(f"  cam_pitch  {pitch_d:+8.3f} °  ± {stderr[1]:.3f}")
        print(f"  cam_roll   {roll_d:+8.3f} °  ± {stderr[2]:.3f}")
    else:
        print(f"  cam_yaw    {yaw_d:+8.3f} °")
        print(f"  cam_pitch  {pitch_d:+8.3f} °")
        print(f"  cam_roll   {roll_d:+8.3f} °")
    print("  ── 顺带解出的靶板方位（不是要填的参数）──")
    print(f"  靶板 az    {az_d:+8.3f} °   靶板 el {el_d:+8.3f} °")
    print("═" * 62)

    if abs(pitch_d) > 30.0:
        print("\n⚠️ cam_pitch 超过 30°。可能是真的，但也可能是解到了错误的")
        print("   分支。用卷尺或量角器粗量一下相机和云台的水平夹角对一下。")
    if stderr is not None and stderr[0] > 0.5:
        print(f"\n⚠️ cam_yaw 标准差 ±{stderr[0]:.2f}° 偏大 —— 它只靠 pitch 扫动")
        print("   的二阶耦合才可观测（见文件头说明）。两个选择：")
        print("   · pitch 多扫几个角度、拉开范围，重录一次")
        print("   · 或者直接 --fix-yaw 0（相机水平方向顺着云台炮口装），")
        print("     把这个自由度让给 pitch/roll，反而更稳")
    if rms_deg > 0.5:
        print(f"\n⚠️ 残差 RMS {rms_deg:.2f}° 偏大。按严重程度排查：")
        print("   1. 云台没停稳就录了 → 每个角度停稳 1 秒再录（最常见）")
        print("   2. 靶心检测被顶灯/反光带偏了 → 用 --debug-out 存图肉眼看一下")
        print("   3. 靶板在扫动期间被挪动了 → 重录")

    print("\n── 复制到 autoaim 命令行 ───────────────────────────────")
    print(f"  --cam-yaw {yaw_d:.2f} --cam-pitch {pitch_d:.2f} "
          f"--cam-roll {roll_d:.2f}")
    print()
    print("── 想写进代码当默认值的话 ──────────────────────────────")
    print(f"  改 app/config.h:90-92 的 ExtrinsicCfg：")
    print(f"    double cam_yaw_deg   = {yaw_d:.2f};")
    print(f"    double cam_pitch_deg = {pitch_d:.2f};")
    print(f"    double cam_roll_deg  = {roll_d:.2f};")

    return 0


if __name__ == "__main__":
    sys.exit(main())
