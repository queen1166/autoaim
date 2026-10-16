# 测试清单 —— 有哪些、测什么、怎么调用

> 本文覆盖 **9 个单测、共 73 条用例**（`test_mpc` / `test_time` / `test_protocol`
> 是后加的，`test_coord` 用例 8~10、`test_pnp` 用例 6~7 也是后加的）。
>
> 一句话用途：**车不在手边时，质量保证全靠第一章那张表里的前四项。**

---

## 一、总表

| 程序 | 在哪 | 测什么 | 碰硬件 | 耗时 |
| --- | --- | --- | --- | --- |
| `ctest`（**9 个**单测） | `build/tests/` | 算法正确性：坐标 / PnP / 检测 / 跟踪 / 瞄准 / 开火 / MPC / 时间对齐 / 协议组帧 | ❌ 纯合成数据 | 秒级 |
| `e2e_test.py` | `tools/` | **整条流水线**：合成图 + 假串口 + 真二进制 | ❌ | ~10 s |
| `protocol_check.py` | `tools/` | 协议字节序（独立的第二份 Python 实现） | 可选 | 秒级 |
| `calib_extrinsic.py --selftest` | `tools/` | 外参反解的实现自校验 | ❌ | 秒级 |
| `tool_detect_view` | `build/tools/` | 检测调参，**有滑块** | ❌ | 手动 |
| `autoaim --replay` | `build/` | 离线整跑，看真实输出和画面 | ❌ | 手动 |
| `main_stage1` | `build/` | **串口 + 云台链路** | ✅ 云台会动 | 手动 |

> 📌 **前四项不碰硬件**，笔记本上就能跑。

---

## 二、怎么构建、怎么跑

```bash
# 构建（在仓库根目录）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# 跑全部 9 个单测，只打结果
cd build && ctest --output-on-failure
```

产物位置：

| 产物 | 路径 |
| --- | --- |
| 9 个单测 | `build/tests/test_<模块名>` |
| C++ 工具 | `build/tools/tool_<名字>` |
| 主程序 | `build/autoaim` |
| 云台扫动程序 | `build/main_stage1` |

**想看每条用例的详细数字，直接跑单个可执行文件**（ctest 只给通过/失败）：

```bash
./build/tests/test_coord      # 坐标变换
./build/tests/test_pnp        # PnP 解算
./build/tests/test_detect     # 传统检测
./build/tests/test_tracker    # EKF 跟踪
./build/tests/test_aim        # 瞄准与弹道
./build/tests/test_fire       # 开火决策
./build/tests/test_mpc        # MPC 与 Planner
./build/tests/test_time       # 姿态队列与延迟链
./build/tests/test_protocol   # 协议组帧与错误恢复
```

### 依赖上与构建有关的两条硬约束

1. **Eigen 不用装**。`third_party/eigen` 已 vendor（规则 §2.3 禁止改系统级配置，
   所以不依赖系统 Eigen）。
2. ⚠️ **`tests/` 和 `tools/` 没有按 OpenCV 做条件编译**（`CMakeLists.txt:123-124`
   无条件 `add_subdirectory`）。没装 OpenCV 时，`autoaim_core` 会**跳过
   `detect/solver/camera`**，于是 `test_coord` / `test_pnp` / `test_detect`
   连编都编不过。CMakeLists 里"OpenCV 可选"只对主程序成立。

> 海康 MVS SDK 只影响 `bsp/camera/hik_camera`，**不影响任何单测**。
> 头文件 vendor 在 `third_party/mvs/include/`，库仍用机器上 `/opt/MVS/lib/64` 的 `.so`。

---

## 三、九个单元测试（逐个）

每个测试都是**自带 `check()` 的小程序**：逐条打印 `PASS` / `FAIL`，
最后一行是 `全部通过` 或 `有失败项`，退出码 0 = 全过、1 = 有失败。
**ctest 靠退出码判断**。

### 3.1 `test_coord` —— 坐标变换链（11 条）

`相机系 → 云台系 → 云台零位系 → 世界系`。**全工程唯一定义坐标约定的地方**，
所以这里错一个符号，下游全部静默出错。

| 用例 | 验什么 |
| --- | --- |
| 1 | `R_cam2gimbal` 是正常旋转（`RᵀR = I`、`det = +1`） |
| 2 | 光轴方向 = `cam_pitch_deg` |
| 3 | 相机 x(右) → 云台 −y；相机 y(下) → 云台 −z |
| 4 | `tvec=(0,0,5)` 且相机下俯 15° → `yaw=0°, pitch=−15°` |
| 5 | 光心偏移 5 cm 带来多大角度误差（结论 < 1°，所以 `cam_x/y/z` 通常不用管） |
| 6 | `gimbalDirFromAngles` ↔ `yawPitchFromGimbalPoint` 往返一致 |
| 7 | `yawFromQuat` 提取的是 yaw 不是 roll |
| **7b** | **yaw 全程扫描 −350°~350°，不许在 ±π 处跳分支** ← 对应"靶板转到背面就丢目标"的坑 |
| **8** | **`R_gimbal2world` 与配置角度的符号约定** |
| **9** | **世界系——姿态决定绝对角（漏掉这步就振荡的那个 bug）** |
| **10** | `attitudeToQuat` ↔ `attitudeFromQuat` 往返 |

> **用例 8/9 是本仓库最贵的两条测试。** 它们钉住的是运行时缺 `R_gimbal`、
> 把"云台零位系相对角"当绝对角下发、导致云台一离开 0° 就来回荡的那个 bug。
> 标定工具 `calib_extrinsic.py` 里本来就有这一步，运行时却没有——两条测试就是防它再犯。

### 3.2 `test_pnp` —— PnP 解算（7 条）

由"图上 4 个角点 + 已知装甲板真实尺寸"反推位置和姿态。

| 用例 | 验什么 |
| --- | --- |
| 1 | 132×57 mm @ 5.00 m，尺寸填对 → 解出距离 = 5.0000 |
| 2 | `SINGLE` 类型应被拒绝 |
| **3** | **几何参数填错（±6%）→ 距离错同样比例** ← 这就是"必须实测反标、不能猜"的证据 |
| 4 | 多个距离下的解算，误差 < 5% |
| 5 | `calculateDistanceToCenter`（选靶用的） |
| **6** | **降自由度成功时解必须几何自洽**（重投影残差 ≈ 0.04 px） |
| **7** | **降自由度永不劣化——失败时必须安全退回**原先的解 |

> 用例 6/7 盯的是 yaw 降自由度（三分法一维搜索）。踩过的坑：把搜索放在相机系、
> 可见角窗口整体差 π；平面靶板接近正对时**两个分支残差都接近 0**（真歧义），
> 只取代价最低那支会在帧间来回翻——所以传参考 yaw 选支。

### 3.3 `test_detect` —— 传统检测（7 条）

二值化 → 找灯条 → 两根灯条配对成装甲板。

| 用例 | 验什么 |
| --- | --- |
| 1 | 两块灯条 → 一块 `SMALL` 装甲板 |
| 2 | 单块灯条 → `SINGLE` 降级 |
| 3 | 关掉降级 → 单灯条不产出 |
| 4 | 中心距 12 倍灯条长 → 配不出对 |
| 5 | 蓝色灯条 + `detect_color=RED` → 不选中 |
| **6** | **纯红 (255,0,0) 灰度仅 76 < 阈值 160 → 抓不到** ← 直接证明"阈值抓的是过曝灯条，不是红板子" |
| 7 | 中心距 4 倍灯条长 → 判为 `LARGE` |

### 3.4 `test_tracker` —— 9 维 EKF（6 条）

状态 `[xc,vx,yc,vy,za,va,yaw,v_yaw,r]`；观测量是**装甲板**，估计的是**旋转中心**。

| 用例 | 验什么 |
| --- | --- |
| 1 | EKF 的 `h(x)` 与 `armorPositionFromState` 自洽（两处实现符号没写反） |
| 2 | `CAROUSEL`：真值 `r=0.30`、`ω=2.0 rad/s`，看能不能估回来 |
| 3 | `SELF_SPIN`：真值 `r=0`（位置不动，只有朝向在转） |
| **4** | **`r=0` 的数据喂给 CAROUSEL 模式会怎样** ← 用错旋转模式的代价 |
| 5 | 跨 ±π 时 `v_yaw` 不跳变（跑 6 秒 ≈ 1.9 圈） |
| 6 | 观测中断后的状态机：丢多久转 `LOST` |

### 3.5 `test_aim` —— 瞄准与弹道（8 条）

| 用例 | 验什么 |
| --- | --- |
| 1 | 弹道补偿（5 m、弹速 22 → 约 2.9°），并代回方程验残差 |
| 2 | 距离越远，补偿越大 |
| 3 | `solveFromMeasurement` 正前方 5 m 静止靶 |
| 4 | 关掉弹道模型 + 经验偏置 3° |
| 5 | 转盘模式 `r=0.5, v_yaw=+1.0` → 有提前量 |
| 6 | 自旋模式 `r=0` → 角度不应有提前量 |
| 7 | `r=0` 时换不同 `v_yaw`，角度不变 |
| **8** | **弹速为 0 或 NaN 时不产生 inf/nan** ← 对应"弹速=0"缺陷：**不崩，只是不准** |

### 3.6 `test_fire` —— 开火四道闸（9 条）

| 用例 | 验什么 |
| --- | --- |
| 1 | 角误差超容差 → 不开火 |
| 2 | 刚对准（< `min_converge_s`）→ 不开火 |
| 3 | 对准并稳定 100 ms → 开火，且按 `fire_hold_s` 持续 |
| 4 | `min_interval_s = 0.9` 的节流 |
| 5 | `aim=179°, gimbal=−179°` → 真实差 2°，应当开火（跨 ±π） |
| 6 | `max_shots` 上限 |
| 7 | `aim.valid = false` → 不开火 |
| 8 | `require_tracking = true` 时跟踪丢失不开火 |
| **9** | **反馈过期 → 不开火（安全）** ← 串口断线时反馈会冻住，这道闸就是防它的 |

### 3.7 `test_mpc` —— MPC 与 Planner（7 条）

盯一个很隐蔽的坑：MPC 的二次型 `H` 元素量级由 `dt²` 决定（`dt=0.01` 时只有 `1e-2`），
**ADMM 的惩罚系数 ρ 不跟着缩放，求解器会"看起来在跑、实际一动不动"**。

| 用例 | 验什么 |
| --- | --- |
| **1** | **常值参考必须跟上去**（这是失控的那个坑；误差 < 0.2°） |
| 2 | 角加速度约束必须被尊重（`\|α\| ≤ max_acc`，阶跃 57.3° 也不许爆表） |
| 3 | 初始已对准时不应乱动（输出贴在参考上，误差 < 1e-3°） |
| 4 | 参考是斜坡时，应有稳定跟随滞后而不是不动（误差 < 0.5°） |
| 5 | **Planner 在转盘目标下 yaw 指令必须随目标摆动**（与 `make_test_bag.py` 同工况） |
| 6 | Planner 的俯仰必须带弹道补偿（静止 5 m 目标 → pitch 为正、量级 < 3.6°） |
| 7 | `fly_time` 迭代收敛（5 m @ 22 m/s → 0.2273 s） |

> 用例 5 的判据是 `4° < yaw 跨度 < 9°`——**但这只在合成图上成立**。
> 真值摆幅约 ±3.4°（跨度 6.9°），所以 4°~9° 是"跟得上、又没夸张"的窗口。

### 3.8 `test_time` —— 姿态队列与延迟链（7 条）

盯两件事：**Slerp 插值出来必须是合法单位四元数**（线性插值会掉出球面）；
**插值结果要真的落在两个样本之间**，而不是简单取最近一包。

| 用例 | 验什么 |
| --- | --- |
| 1 | 中间时刻取到的是插值，不是最近样本（105↔110 ms 取 40% → 4.0°，取最近会是 10°） |
| 2 | 插值结果仍是单位四元数（150° 大角度下 `\|‖q‖−1\| < 1e-12`，证明用的是 Slerp） |
| 3 | 区间外取端点，**不外推** |
| 4 | 空队列返回 `false`（调用方据此退化为单位姿态） |
| 5 | 乱序 / 重复时间戳被丢弃，队列保持单调 |
| 6 | 容量上限会丢最旧的 |
| 7 | 延迟链总量 = `process_latency + comm_latency + fly_time` |

> 用例 1 是这一站存在的唯一理由：IMU 姿态和相机帧物理上异步，
> 用"缓冲区里最新那包"去解释几毫秒前的图像，在云台追旋转靶时会带来
> **与转速成正比的系统性角度偏差**。

### 3.9 `test_protocol` —— 协议组帧与错误恢复（11 条）

盯两件事：**线上字节序**，以及**错误恢复策略贴合协议文档 §1**。
它是全工程唯一不依赖 OpenCV 也能建的"通信层"测试（只 include `srm_protocol.h`）。

| 用例 | 验什么 |
| --- | --- |
| 1 | 发送帧字节序锚点：`yaw=1.0, pitch=0.0, fire=0` → 18 字节逐字节匹配（与 `protocol_check.py` 同一向量） |
| 2 | 半帧分两次喂入 → 第二次才产出 |
| 3 | 只收到 `body_len` 两字节 → 继续等（防 `>=2` / `>2` 差一） |
| 4 | 差最后一字节 → 不产出 |
| 5 | 背靠背两帧一次喂入 → 2 回调 |
| 6 | 整帧 + 尾随半帧 → 半帧留着等下次 |
| **7** | **非法长度 → 丢弃整个缓存**，其后紧跟的合法帧也一并不产出（**有意丢失**） |
| **8** | **未知 ID（ID1 / ID2 各一）→ 丢弃整个缓存** |
| 9 | `resyncCount()` 只在丢弃时 +1，半帧等待 / 成功解析都不算 |
| 10 | 与 `protocol_check.py` 的解析向量对表（负数、角度制边界） |
| 11 | 单次灌入 > `kMaxBufferLen` 字节 → 丢弃，之后能恢复 |

> **用例 7 / 8 是这一站存在的理由。** 协议没有魔数和 CRC，`body_len` 或 ID
> 对不上时**无法在任意字节处可靠重同步**（逐字节滑动可能撞上 payload 里碰巧
> 出现的 `28`），所以文档要求"丢弃当前缓存并等待下一次完整帧"。
> 代价是坏字节之后的合法帧会被一起丢掉 —— 这是**文档要求的行为**，不是 bug。
> 两条用例把它钉死，避免后人改回滑动。

---

## 四、工具类测试

### 4.1 `e2e_test.py` —— 最重要的一条

```bash
python3 tools/e2e_test.py ./build/autoaim
```

**它不是单测的重复**：单测只验算法，这个验"**整条流水线真的能工作**"。

它做四件事：

1. 调 `make_test_bag.py` 生成一批合成图像（装甲板绕中心公转，**真值已知**）
2. 开一对 **PTY**，把从设备路径当作 `/dev/ttyACM*` 传给 `autoaim`
3. **自己扮演下位机**：读 18 字节指令帧，回 30 字节反馈帧
4. 检查 `autoaim` 发出来的 yaw 跟不跟得上真实的装甲板运动

**判据**：指令帧数 ≥ 100、畸形帧 = 0、yaw 跨度 ≥ 1.0°。
最后一行打印 `端到端测试通过` 或 `端到端测试失败`。

```bash
python3 tools/e2e_test.py ./build/autoaim --secs 12       # 跑久一点
python3 tools/e2e_test.py ./build/autoaim --no-tracker    # 关掉跟踪器再跑一遍
python3 tools/e2e_test.py ./build/autoaim --bag /tmp/bag  # 用现成的图像目录
```

> 📌 它自己起假串口，**不需要下位机、不需要相机、不需要 sudo**。

⚠️ **已知未达标项**：这条能跑通、帧格式也对，但**指令 yaw 的摆幅明显大于真值**
（真值约 ±3.4°，实测经常到 ±5°~±15°，且启动阶段有一次较大漂移）。
坐标链、yaw 降自由度、MPC 都已排除（单测全过），嫌疑在 EKF 的 yaw / 半径噪声参数。
**别拿合成数据的摆幅当验收标准**——合成图的灯条边缘是理想矩形，
和真实靶板的模糊/反光差别很大。**以实车打靶的弹着点为准。**

### 4.2 `protocol_check.py` —— 协议字节序

```bash
python3 tools/protocol_check.py                  # 只做自检
python3 tools/protocol_check.py /dev/ttyACM0     # 自检 + 真机收几帧
```

**为什么需要它**：协议**没有魔数、没有 CRC**，`body_len` 或 ID 对不上就只能
**丢弃整个缓存、等下一帧**（协议 §1 明确禁止逐字节滑动重同步）。
**字节序写错了不会报错，只会静默地发疯**——所以要用 Python 独立再实现一份字节序，
和 C++ 对**同一组输入**比字节。两份一致才能说明没写错。

`test_fire` 验的是决策逻辑，`test_protocol` 验的是组帧与错误恢复，这个验的是
**字节**，三码事。

### 4.3 `make_test_bag.py` —— 造合成数据

```bash
python3 tools/make_test_bag.py /tmp/bag --frames 240
python3 tools/make_test_bag.py /tmp/bag --mode self_spin
```

参数：`outdir`（位置参数）、`--frames`（默认 200）、`--fps`（默认 60）、
`--mode {carousel,self_spin}`（默认 `carousel`）。

生成"一块绕中心飞的靶板"，**真值已知**，是下面 4.4 / 4.6 和 e2e 的输入。
（`/tmp` 在 WSL 重启后会清空，随时可重新生成。）

### 4.4 `tool_detect_view` —— 唯一能拖滑块的地方

```bash
python3 tools/make_test_bag.py /tmp/bag --frames 240
./build/tools/tool_detect_view /tmp/bag          # 两个滑块：binary_thres / color(0=R,1=B)
./build/tools/tool_detect_view some.png          # 只喂单张图
./build/tools/tool_detect_view /tmp/bag --no-loop
```

> 📌 `autoaim --debug-view` **没有滑块**，只有 `imshow` + `waitKey`。
> 现场调 `--thres` 用这个工具，比反复改命令行快。

### 4.5 `calib_extrinsic.py --selftest` —— 标外参之前的必跑项

```bash
python3 tools/calib_extrinsic.py --selftest
```

**不需要任何数据**。它拿 `make_test_bag.py` 里的同一份实现做交叉验证——
确认本机上 `R_cam2gimbal` 的实现和 C++ 那边**逐字节一致**。

> 📌 标外参**之前**先跑这个。实现不一致的话，标出来的外参是错的，而且看不出来。

标外参本身的调用（需要先用 `tool_hik_record` 录云台扫角数据）：

```bash
./build/tools/tool_hik_record --out /tmp/sweep     # 录图 + angles.csv
python3 tools/calib_extrinsic.py --angles /tmp/sweep/angles.csv \
        --images /tmp/sweep --fx <内参> --fy <内参> --cx <内参> --cy <内参> \
        --debug-out /tmp/sweep/debug                # 把找到的靶心画出来，肉眼复核
```

### 4.6 `autoaim --replay` —— 离线整跑（不碰硬件）

```bash
python3 tools/make_test_bag.py /tmp/bag --frames 240

# 只打印跟踪状态
./build/autoaim --replay /tmp/bag --enable-tracker --debug-dump

# 开窗口：画面左上有 d=?.??m / yaw / pitch，画面正中有黄十字（代码认为的光心）
./build/autoaim --replay /tmp/bag --enable-tracker --debug-view
```

**这一条是"单元测试全过了，但程序到底能不能跑"的答案**：真线程、真主循环、
真检测、真解算，只是把相机换成图像目录、串口不接。

`--debug-dump` 是本仓库现场排查的主力开关，重点看两个量：`r`（旋转半径，
应稳定在实测半径附近）和 `v_yaw`（应等于靶板真实转速）。
`r` 顶到 `r_max`、`v_yaw` 明显偏大 → 先查靶板朝向的 π 约定（`--armor-yaw-offset`）。

### 4.7 `main_stage1` —— 硬件链路测试（⚠️ 会让云台真的动）

```bash
./build/main_stage1 /dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_<序列号>-if00
```

**测什么**：串口通不通 + 下位机反馈能不能解析 + 云台听不听指令——三件事一次测完。
**它做的事**：100 Hz 发指令，yaw 在 ±15° 之间往复扫（周期 6 s），
同时把收到的反馈打印在同一行。

| 现象 | 结论 |
| --- | --- |
| 数字在刷、云台在扫 | ✅ 全链路通 |
| 数字在刷、云台不动 | 串口没问题 → 是**供电**或**控制权** |
| 打不开串口 | 程序自己会打排查步骤（设备名 / dmesg / dialout 组 / lsof） |
| `弹速=0.0` | 已知缺陷（见本文 §七） |

> ⚠️ **这一步会让云台真的动起来。** 车要架稳、供电正常、安全员在旁边，
> 三条齐了再跑。它**不含视觉**——相机、检测、PnP 都不参与。

---

## 五、改了什么，就至少跑什么

| 你改了什么 | 至少跑 |
| --- | --- |
| 只改注释 / 文档 | 不用跑 |
| 算法（`modules/aim` `tracker` `detect` `solver` `plan` `time`） | `ctest` |
| 协议（`modules/protocol`） | `ctest` + `protocol_check.py` |
| 线程 / 相机 / 主循环（`app/` `threads/` `bsp/camera`） | 上面 + `e2e_test.py` |
| 串口 / 复位逻辑（`bsp/serial`） | 上面 + `main_stage1`（有硬件时） |
| **上场之前** | **全套**：`ctest` → `protocol_check.py` → `e2e_test.py` → `main_stage1` |

---

