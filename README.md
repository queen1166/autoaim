# autoaim —— 校内赛自瞄赛道上位机

C++17 / CMake，不依赖 ROS。通信协议严格按 `self_aim_protocol.md`。

检测与跟踪算法派生自开源项目 **rm_auto_aim**（MIT），详见 [THIRD_PARTY.md](THIRD_PARTY.md)。
**不含任何 SRM 队内代码。**

---

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cd build && ctest --output-on-failure
```

### 依赖

| 依赖 | 说明 |
| --- | --- |
| CMake ≥ 3.16 | |
| g++ ≥ 9（C++17） | |
| **Eigen** | 优先用系统的；没有则用仓库内 `third_party/eigen`（已 vendor） |
| **OpenCV** | 可选。没有它仍能编译和测试协议 / 跟踪 / 瞄准部分 |
| 海康 MVS SDK | 只在编译 `bsp/camera/hik_camera` 时需要，路径 `/opt/MVS` |

> ⚠️ 规则 §2.3 要求不要修改系统级配置、且小电脑是共用设备。
> **不要 `sudo apt install libopencv-dev`** —— 装到自己的 conda 环境里，
> 或用系统已装的。Eigen 已经 vendor 进仓库，不需要装。

## 当前进度

**全部模块已完成，全新构建 `-Wall -Wextra` 零警告，6 个单元测试 + 1 个端到端测试全过。**

| 模块 | 状态 |
| --- | --- |
| `bsp/serial` | ✅ termios 封装（线程安全） |
| `bsp/camera` | ✅ 抽象接口 + 离线回放；海康 MVS 需 SDK（未装时不影响构建） |
| `modules/protocol` | ✅ 18B 打包 / 30B 流式组帧，与协议文档逐字节相符 |
| `modules/detect` | ✅ 灯条检测、装甲板配对、单灯条降级 |
| `modules/solver` | ✅ PnP 解算 + 相机系→云台系 |
| `modules/tracker` | ✅ EKF + 双旋转模式 |
| `modules/aim` | ✅ 瞄准层（提前量 + 弹道）、开火决策 |
| `app` + `threads` | ✅ 四线程流水线 |

### 怎么跑

```bash
# 1. 离线：生成合成测试包（一块绕中心公转的装甲板），回放看检测
python3 tools/make_test_bag.py /tmp/bag --frames 240
./build/tools/tool_detect_view /tmp/bag          # 滑动条实时调 binary_thres

# 2. 端到端：合成图像 + PTY 假串口 + 真跑二进制
python3 tools/e2e_test.py ./build/autoaim

# 3. 真机：先只瞄准不开火
./build/autoaim --hik --device /dev/serial/by-id/usb-XXX     --fx <标定值> --fy <标定值> --cx <标定值> --cy <标定值>     --cam-pitch <实测> --armor-w <实测> --armor-h <实测> --debug-dump

# 确认瞄得准之后，才加 --enable-fire（且必须已有安全员许可）
```

## 现场必须实测的四件事

代码里的默认值是**占位符**，不实测就是错的。

### 1. 装甲板几何（决定 PnP 尺度）

`modules/solver/pnp_solver.h` 的 `ArmorGeometry`。

**这不是板子外形尺寸，是灯条几何：**
- `width` = 两根灯条**中心**之间的距离
- `height` = 灯条的**长度**

反标方法：靶板放卷尺量准的 5.00 m，打印 `tvec` 模长。
实测标定结果：**填 140 而非 132，距离直接错 +5.15%**（见 `tests/test_pnp.cpp` 用例 3）。

### 2. 相机外参

`modules/solver/coord_transform.h` 的 `ExtrinsicConfig`。这是**全局唯一**允许定义
相机→云台约定的地方。参数化成 yaw/pitch/roll，用"云台扫几个角度、记录靶心像素"
做最小二乘反解。不要上 `cv::calibrateHandEye`，5 m 靶用不上那个精度。

**pitch 的符号必须实测**：上游仓库里从没算过 pitch，没有可继承的约定。

### 3. 靶板怎么转（决定用哪套模型）

`TrackerConfig::mode`，默认 `AUTO`。现场判定：

```
站侧面看：装甲板在圆周上跑 → CAROUSEL
          位置不动、只是面朝方向在转 → SELF_SPIN

或开 debug 打印 r：
  稳定收敛到 0.15~0.5 → CAROUSEL
  持续趋近 0         → SELF_SPIN
```

用错模式的代价实测是 6 cm 位置误差（`tests/test_tracker.cpp` 用例 4）。

### 4. `binary_thres` 与曝光

默认 160 是上游实验室灯下定的。它的实际含义是**"多亮才算灯条"** ——
纯红 `(255,0,0)` 的灰度只有 76，远低于阈值，根本抓不到。
227 实验室/走廊的光照完全不同，**必须重调**。

## 实测发现的、影响成绩的两件事

### 1. 镜头焦距比算法更影响精度

蒙特卡洛实测（靶板 140×125 @ 5 m，像素噪声 1px）：

| 镜头 | 靶板在画面里占多宽 | PnP 距离中位误差 / p95 |
| --- | --- | --- |
| 6 mm | 49 px | 7.5 cm / 22.8 cm |
| 12 mm | 97 px | 3.4 cm / 11.0 cm |
| 16 mm | 130 px | 2.7 cm / 7.7 cm |

6 mm 镜头下靶板只占 49 像素，距离估计是病态的。**若 CS016 的镜头可换，优先 12–16 mm。**

### 2. `binary_thres` 的实际含义是"多亮才算灯条"

纯红 `(255,0,0)` 的灰度只有 76，远低于默认阈值 160 —— 完全抓不到。
真实灯条是过曝的 LED，灰度很高。**这个值直接和曝光、补光、环境光强挂钩，换场地必须重调。**

## 已知的坑（都踩过了，写在代码注释里）

1. **`Eigen::eulerAngles(2,1,0)(0)` 不等于 tf2 的 `getRPY().yaw`。**
   实测在 |yaw| > π 时会跳到另一组合法分支，返回 `yaw - π`。
   必须用 `atan2(R(1,0), R(0,0))`。漏掉的现象是"靶板转到背面就丢目标"。
   → `modules/solver/coord_transform.cpp`，回归测试 `test_coord` 用例 7b

2. **`SOLVEPNP_IPPE` 有重尾。** 中位精度和 ITERATIVE 相当，但 p95 差 10 倍。
   注意要求正方形的是 `IPPE_SQUARE`，不是 `IPPE`。

3. **`USB 包边界 ≠ 协议帧边界`。** 一次 read 可能是半帧 / 整帧 / 多帧，
   必须流式组帧。协议无魔数无 CRC，长度不对只能丢缓存重来。

4. **必须发完整的 ID 1 + ID 2。** 省略记录时下位机不会清零旧字段。

5. **发送线程里不能做重活。** 打印、写文件都会让 100 Hz 定频抖动。

6. **云台不动先按鼠标右键。** 视觉控制由右键进入/退出。

7. **转盘模式下 `v_yaw` 会冲到 55 rad/s**（真值 2.0）。
   装甲板侧对相机时检测中断（实测约 29% 的帧），EKF 在空窗期盲推，
   重捕时速度估计炸掉。`aim_solver` 里已把 `v_yaw` 夹到 10 rad/s，
   `app` 层还有一道单帧角度跳变闸门。**这是现场最需要调的地方。**

## 目录

```
autoaim/
├── bsp/          serial（termios）、camera（抽象 + 回放 + 海康）
├── modules/      protocol / detect / solver / tracker / aim / common
├── app/          autoaim_app（流水线）、config、main；main_stage1（扫动测试）
├── threads/      四线程编排
├── tests/        6 个离线单元测试
├── tools/        make_test_bag.py / e2e_test.py / detect_view / protocol_check.py
├── third_party/  Eigen 3.4.0（已 vendor，不需要装）
├── README.md         本文件
├── ARCHITECTURE.md   各文件作用、命名来源、模块关系（想读懂代码先看这个）
├── COMPLIANCE.md     ★ 校内赛规则符合性检查
└── THIRD_PARTY.md    第三方代码来源与许可
```

## 到现场之前

先读 **[COMPLIANCE.md](COMPLIANCE.md)** —— 里面有逐条规则核查和 **4 个必须现场处理的风险项**，
其中"海康 MVS SDK 是系统级安装，与规则 §2.3 冲突"必须先问赛务。
