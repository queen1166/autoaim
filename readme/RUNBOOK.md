# 从零开始运行 autoaim


## 一、编译跑通


  ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   ```
   如果 OpenCV 装在 conda 环境里没被找到，加 `-DCMAKE_PREFIX_PATH=$CONDA_PREFIX` 重新配置。

  ```bash
   
   cmake --build build -j
   ```

  ```bash
   cd build && ctest --output-on-failure
   ```

跑端到端测试：生成合成图像、开假串口、真跑编译出来的二进制，最后打印"端到端测试通过"。

   ```bash
   cd .. && python3 tools/e2e_test.py ./build/autoaim
   ```

想看检测画面：先生成测试包，再开窗口工具，拖滑块调二值化阈值，按 q 退出。
   注意 `/tmp` 在 WSL 重启后会清空，包随时可重新生成。

   ```bash
   python3 tools/make_test_bag.py /tmp/bag --frames 240
   ./build/tools/tool_detect_view /tmp/bag
   ```

> 📖 **每个测试程序分别测什么、怎么跑、怎么看结果** → 见 [第四部分](#四测试程序一览)。

---

## 二、比赛机器上的一次性准备（硬件相关）

7. **海康 MVS SDK**。这是最高优先级风险项，装之前先问赛务。允许的话从海康官网下载 .deb
   装到 `/opt/MVS`，然后重新配置和编译：

   ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ```

   配置时出现"海康 MVS SDK: 已找到"就成功。没装时 `--hik` 会在启动时被拒绝并提示。

8. **串口权限**。先用 `ls /dev/serial/by-id/` 记下真实设备名；按 [README.md](README.md) 把用户
   加进 dialout 组后重新登录。共用机器上的权限改动先跟赛务确认。

9. **标定四件套**。默认值都是占位符，必须实测：

   ```bash
   # 内参：棋盘格照片解出 --fx --fy --cx --cy，输出可直接粘贴
   python3 tools/calib_intrinsic.py --dir calib_imgs --cols 9 --rows 6 --square-mm 25

   # 外参：边扫云台边录图像和角度，再最小二乘反解出 --cam-yaw --cam-pitch --cam-roll
   ./build/tools/tool_hik_record --out calib_ext --device /dev/serial/by-id/usb-XXX
   python3 tools/calib_extrinsic.py --angles calib_ext/angles.csv --images calib_ext \
     --fx <内参> --fy <内参> --cx <内参> --cy <内参>

   # 灯条几何 --armor-w / --armor-h：卷尺实测反标，填 140 还是 132 能让 5 米距离差 5%
   # 阈值 --thres：现场用 tool_detect_view 拖滑块调（--debug-view 没有滑块）
   ```

---

## 三、现场每次怎么跑
 只瞄准不开火，把标定值填齐。

    ```bash
    ./build/autoaim --hik --device /dev/serial/by-id/usb-XXX \
      --fx <内参> --fy <内参> --cx <内参> --cy <内参> \
      --cam-pitch <标定值> --cam-yaw <标定值> --cam-roll <标定值> \
      --armor-w <实测> --armor-h <实测> --thres <现场调> \
      --enable-tracker --rotation auto --debug-dump
    ```

12. 只在正式比赛、有安全员许可时才加 `--enable-fire`。平时调试和第一次上场都不加。

> 📖 **每个参数的值怎么写、格式规则、三套现成命令** → 见 [第五部分](#五命令行传参速查autoaim-主程序)。

---

## 四、测试程序一览

### 4.0 先看这张总表：哪个程序测什么

| 程序 | 在哪 | 测什么 | 碰硬件吗 | 多久 |
| --- | --- | --- | --- | --- |
| `ctest`（6 个单元测试） | `build/tests/` | 算法正确性（坐标/PnP/检测/跟踪/瞄准/开火） | ❌ 纯合成数据 | 秒级 |
| `tool_detect_view` | `build/tools/` | 检测调参，**有滑块** | ❌ | — |
| `e2e_test.py` | `tools/` | **整条流水线**（合成图 + 假串口 + 真二进制） | ❌ | ~10 s |
| `protocol_check.py` | `tools/` | 协议字节序（独立的第二份实现） | 可选 | 秒级 |
| `calib_extrinsic.py --selftest` | `tools/` | 外参反解的实现自校验 | ❌ | 秒级 |
| `autoaim --replay` | `build/` | 离线整跑，看真实输出和画面 | ❌ | 手动 |
| `main_stage1` | `build/` | **串口 + 云台链路** | ✅ 串口 + 云台会动 | 手动 |

> 📌 **前四个不碰硬件，随时能跑。** 车不在手边的时候，质量保证全靠它们。

---

### 4.1 单元测试（ctest，6 个）

```bash
cd build && ctest --output-on-failure
```

只看结果。**想看每个用例的详细数字，直接跑单个可执行文件**：

```bash
./build/tests/test_coord     # 坐标变换
./build/tests/test_pnp       # PnP 解算
./build/tests/test_detect    # 检测
./build/tests/test_tracker   # EKF 跟踪
./build/tests/test_aim       # 瞄准解算
./build/tests/test_fire      # 开火决策
```

**怎么看结果**：每个用例打印一行行 `PASS` / `FAIL`，最后一行是 `全部通过` 或 `有失败项`。ctest 靠退出码判断（不等于 0 就是失败）。

> ⚠️ **`test_coord` / `test_detect` / `test_pnp` 用了 OpenCV 头文件**，这台机器上没 OpenCV 的话这三个连编都编不过（[CMakeLists.txt](CMakeLists.txt) 里写的"OpenCV 可选"只对主程序成立）。

#### 各测试的用例清单

**`test_coord` —— 坐标变换（相机系 → 云台系）**

| 用例 | 验什么 |
| --- | --- |
| 1 | `R_cam2gimbal` 是正常旋转（`RᵀR = I`、`det = +1`） |
| 2 | 光轴方向 = `cam_pitch_deg` |
| 3 | 相机 x(右) → 云台 -y；相机 y(下) → 云台 -z |
| 4 | `tvec=(0,0,5)` 且相机下俯 15° → `yaw=0°, pitch=-15°` |
| 5 | 光心偏移 5 cm 带来多大角度误差（结论 < 1°，所以 `cam_x/y/z` 通常不用管） |
| 6 | `gimbalDirFromAngles` ↔ `yawPitchFromGimbalPoint` 往返一致 |
| 7 | `yawFromQuat` 提取的是 yaw 不是 roll |
| **7b** | **yaw 全程扫描 -350°~350°，不许在 ±π 处跳分支** ← 这条对应"靶板转到背面就丢目标"的坑 |

**`test_pnp` —— PnP 解算**

| 用例 | 验什么 |
| --- | --- |
| 1 | 132×57 mm @ 5.00 m，尺寸填对 → 解出距离 = 5.0000 |
| 2 | `SINGLE` 类型应被拒绝 |
| **3** | **几何参数填错（±6%）→ 距离错同样比例** ← 这就是"必须实测反标、不能猜"的证据 |
| 4 | 多个距离下的解算，误差 < 5% |
| 5 | `calculateDistanceToCenter`（选靶用的） |

**`test_detect` —— 检测**

| 用例 | 验什么 |
| --- | --- |
| 1 | 两块灯条 → 一块 `SMALL` 装甲板 |
| 2 | 单块灯条 → `SINGLE` 降级 |
| 3 | 关掉降级 → 单灯条不产出 |
| 4 | 中心距 12 倍灯条长 → 配不出对 |
| 5 | 蓝色灯条 + `detect_color=RED` → 不选中 |
| **6** | **纯红 (255,0,0) 灰度仅 76 < 阈值 160 → 抓不到** ← 直接证明"阈值抓的是过曝灯条，不是红板子" |
| 7 | 中心距 4 倍灯条长 → 判为 `LARGE` |

**`test_tracker` —— EKF 跟踪**

| 用例 | 验什么 |
| --- | --- |
| 1 | EKF 的 `h(x)` 与 `armorPositionFromState` 自洽 |
| 2 | `CAROUSEL`：真值 `r=0.30`、`ω=2.0 rad/s`，看能不能估回来 |
| 3 | `SELF_SPIN`：真值 `r=0` |
| **4** | **`r=0` 的数据喂给 CAROUSEL 模式会怎样** ← 用错旋转模式的代价 |
| 5 | 跨 ±π 时 `v_yaw` 不跳变（跑 1.9 圈） |
| 6 | 观测中断后的状态机：丢多久转 `LOST` |

**`test_aim` —— 瞄准解算**

| 用例 | 验什么 |
| --- | --- |
| 1 | 弹道补偿（5 m、弹速 22 → 约 2.9°），并代回方程验残差 |
| 2 | 距离越远，补偿越大 |
| 3 | `solveFromMeasurement` 正前方 5 m 静止靶 |
| 4 | 关掉弹道模型 + 经验偏置 3° |
| 5 | 转盘模式 `r=0.5, v_yaw=+1.0` → 有提前量 |
| 6 | 自旋模式 `r=0` → 角度不应有提前量 |
| 7 | `r=0` 时换不同 `v_yaw`，角度不变 |
| **8** | **弹速为 0 或 NaN 时不产生 inf/nan** ← 对应"弹速=0"缺陷：不崩，只是不准 |

**`test_fire` —— 开火四道闸**

| 用例 | 验什么 |
| --- | --- |
| 1 | 角误差超容差 → 不开火 |
| 2 | 刚对准（< `min_converge_s`）→ 不开火 |
| 3 | 对准并稳定 100 ms → 开火，且按 `fire_hold_s` 持续 |
| 4 | `min_interval_s = 0.9` 的节流 |
| 5 | `aim=179°, gimbal=-179°` → 真实差 2°，应当开火（跨 ±π） |
| 6 | `max_shots` 上限 |
| 7 | `aim.valid = false` → 不开火 |
| 8 | `require_tracking = true` 时跟踪丢失不开火 |
| **9** | **反馈过期 → 不开火（安全）** ← 串口断线时反馈会冻住，这道闸就是防它的 |

---

### 4.2 端到端测试 `e2e_test.py` —— 最重要的一条

```bash
python3 tools/e2e_test.py ./build/autoaim
```

**它不是单测的重复**：单测只验算法，这个验"**整条流水线真的能工作**"。

它做四件事：

1. 调 `make_test_bag.py` 生成一批合成图像（装甲板绕中心公转，**真值已知**）
2. 开一对 **PTY**，把从设备路径当作 `/dev/ttyACM*` 传给 `autoaim`
3. **自己扮演下位机**：读 18 字节指令帧，回 30 字节反馈帧
4. 检查 `autoaim` 发出来的 yaw 跟不跟得上真实的装甲板运动

**判据**（全部满足才算过）：

| 判据 | 阈值 |
| --- | --- |
| 指令帧数量 | ≥ 100（少了说明发送线程没跑起来） |
| 畸形帧 | = 0 |
| yaw 跨度 | ≥ 1.0°（转盘靶在 5 m 处绕 0.3 m 公转 → 理论摆幅约 ±3.4°） |

**输出**：最后一行是 `端到端测试通过` 或 `端到端测试失败`。

**常用参数**：

```bash
python3 tools/e2e_test.py ./build/autoaim --secs 12      # 跑久一点
python3 tools/e2e_test.py ./build/autoaim --no-tracker   # 关掉跟踪器再跑一遍
python3 tools/e2e_test.py ./build/autoaim --bag /tmp/bag # 用现成的图像目录
```

> 📌 它自己起假串口，**不需要下位机、不需要相机、不需要 sudo**。

---

### 4.3 协议自检 `protocol_check.py`

```bash
python3 tools/protocol_check.py                  # 只做自检
python3 tools/protocol_check.py /dev/ttyACM0     # 自检 + 真机收几帧
```

**为什么需要它**：协议**没有魔数、没有 CRC**，长度不对只能靠"删 1 字节再试"重新同步。**字节序写错了不会报错，只会静默地发疯** —— 所以要用 Python 独立再实现一份字节序，和 C++ 对**同一组输入**比字节。两份一致才能说明没写错。

`test_fire` 验的是决策逻辑，这个验的是**字节**，两码事。

---

### 4.4 检测调参 `tool_detect_view`（有滑块）

```bash
# 先生成一份合成图（/tmp 在 WSL 重启后会清空，随时可重新生成）
python3 tools/make_test_bag.py /tmp/bag --frames 240

# 开窗口，两个滑块：binary_thres / color(0=R,1=B)，按 q 退出
./build/tools/tool_detect_view /tmp/bag
```

也可以只喂单张图：

```bash
./build/tools/tool_detect_view some.png
./build/tools/tool_detect_view /tmp/bag --no-loop    # 不循环
```

> 📌 这是**唯一能拖滑块**的地方。`autoaim --debug-view` **没有滑块**，只有 `imshow` + `waitKey`。

---

### 4.5 外参反解的实现自校验

```bash
python3 tools/calib_extrinsic.py --selftest
```

**不需要任何数据**。它拿 `make_test_bag.py` 里的同一份实现做交叉验证 —— 确认本机上 `R_cam2gimbal` 的实现和 C++ 那边**逐字节一致**。

> 📌 标外参**之前**先跑这个。实现不一致的话，标出来的外参是错的，而且看不出来。
> （`calib_intrinsic.py` 没有 `--selftest`，它靠重投影误差自检。）

---

### 4.6 离线整跑 `autoaim --replay`（不碰硬件）

```bash
python3 tools/make_test_bag.py /tmp/bag --frames 240

# 只打印跟踪状态
./build/autoaim --replay /tmp/bag --enable-tracker --debug-dump

# 开窗口：画面左上有 d=?.??m / yaw / pitch，画面正中有黄十字（代码认为的光心）
./build/autoaim --replay /tmp/bag --enable-tracker --debug-view
```

**这一条是"单元测试全过了，但程序到底能不能跑"的答案**：真线程、真主循环、真检测、真解算，只是把相机换成图像目录、串口不接。

---

### 4.7 硬件链路测试 `main_stage1`（⚠️ 会让云台真的动）

```bash
./build/main_stage1 /dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00
```

**测什么**：串口通不通 + 下位机反馈能不能解析 + 云台听不听指令 —— 三件事一次测完。

**它做的事**：100 Hz 发指令，yaw 在 ±15° 之间往复扫（周期 6 s），同时把收到的反馈打印在同一行。

**期望**：屏幕上 `反馈 yaw= ... pitch= ... roll= ... 弹速= ... mode= ... color= ...` 在刷，云台跟着扫。

**怎么排查**：

| 现象 | 结论 |
| --- | --- |
| 数字在刷、云台在扫 | ✅ 全链路通 |
| 数字在刷、云台不动 | 串口没问题 → 是**供电**或**控制权**，见 [FIELD_CHECKLIST.md](readme/FIELD_CHECKLIST.md) 线路 3 |
| 打不开串口 | 程序自己会打排查步骤（设备名 / dmesg / dialout 组 / lsof） |
| `弹速=0.0` | 已知缺陷，见 [FIELD_CHECKLIST.md](readme/FIELD_CHECKLIST.md) 第三部分 |

> ⚠️ **这一步会让云台真的动起来。** 车要架稳、供电正常、安全员在旁边，三条齐了再跑。

---

### 4.8 改了什么，就至少跑什么

| 你改了什么 | 至少跑 |
| --- | --- |
| 只改注释 / 文档 | 不用跑 |
| 算法（`modules/aim` `modules/tracker` `modules/detect` `modules/solver`） | `ctest` |
| 协议（`modules/protocol`） | `ctest` + `protocol_check.py` |
| 线程 / 相机 / 主循环（`app/` `threads/` `bsp/camera`） | 上面 + `e2e_test.py` |
| 串口 / 复位逻辑（`bsp/serial`） | 上面 + `main_stage1`（有硬件时） |
| **上场之前** | **全套**：`ctest` → `protocol_check.py` → `e2e_test.py` → `main_stage1` |

---

## 五、命令行传参速查（`autoaim` 主程序）

### 5.1 格式规则

```bash
./build/autoaim  [选项] [选项] ...
```

| 规则 | 说明 |
| --- | --- |
| **没有位置参数** | 所有东西都得用 `--flag` 传。写一个裸的设备路径会被当成"未知参数"报错 |
| **带值的写成 `--名字 值`** | 中间是**空格**，不是等号。`--fx 3478` ✓，`--fx=3478` ❌ |
| **布尔开关只写名字** | `--enable-fire` ✓，`--enable-fire 1` ❌ |
| **顺序随意** | 解析是顺序扫一遍，前后无所谓 |
| **重复传同一个参数** | **后写的覆盖先写的**（比如 `--thres 160 --thres 200` → 生效 200） |

**三种输入错误，都会打印帮助并退出（退出码 1）**：

| 输入 | 报错 |
| --- | --- |
| 不认识的参数 | `未知参数: xxx` |
| 带值的参数后面没跟东西 | `参数 --xxx 缺少值` |
| 值不合法 | `--tx-hz 要在 1~1000 之间` / `--rotation 只能是 carousel / self_spin / auto` |

> ⚠️ **`-h` / `--help` 的退出码也是 1**，不是 0（[config.cpp:70-72](app/config.cpp#L70-L72) 打印完帮助返回 `false`）。写脚本判断"程序是否正常退出"时别被这个坑到。

---

### 5.2 全部参数一览

**相机**

| 参数 | 值 | 默认 | 说明 |
| --- | --- | --- | --- |
| `--replay <目录>` | 目录路径 | `bags/latest` | 用图像目录做离线回放。**这是默认模式** |
| `--replay-fps <fps>` | 数字 | `60` | 回放帧率 |
| `--no-loop` | 无 | — | 回放不循环（默认循环） |
| `--hik` | 无 | — | 用海康相机。**需要 MVS SDK，没有就直接退出** |
| `--exposure <us>` | 数字（微秒） | `5000` | 曝光时间 |
| `--gain <g>` | 数字 | `10` | 增益 |

**串口**

| 参数 | 值 | 默认 | 说明 |
| --- | --- | --- | --- |
| `--device <路径>` | 设备路径 | `/dev/ttyACM0` | 建议用 `/dev/serial/by-id/...`（名字唯一，不会随插拔变号） |
| `--tx-hz <hz>` | 1~1000 | `100` | 发送频率。**超出范围直接报错退出** |

**检测与解算**

| 参数 | 值 | 默认 | 说明 |
| --- | --- | --- | --- |
| `--thres <n>` | 数字 | `160` | 二值化阈值。★ 现场必调 |
| `--fx` `--fy` `--cx` `--cy` | 数字 | 1000 / 1000 / 720 / 540 | 相机内参。★ **全是占位符，必须标定** |
| `--armor-w <mm>` | 数字（毫米） | `132` | 两灯条**中心距**。★ 必须实测 |
| `--armor-h <mm>` | 数字（毫米） | `57` | 灯条**长度**。★ 必须实测 |
| `--cam-pitch <deg>` | 数字（度） | `-15` | 相机俯仰安装角，负值向下。★ **符号必须实测** |
| `--cam-yaw <deg>` | 数字（度） | `0` | 相机水平安装角 |
| `--cam-roll <deg>` | 数字（度） | `0` | 相机翻滚安装角 |

**跟踪**

| 参数 | 值 | 默认 | 说明 |
| --- | --- | --- | --- |
| `--enable-tracker` | 无 | 关 | 启用 EKF 跟踪器（有了它才有提前量） |
| `--rotation <mode>` | `carousel` \| `self_spin` \| `auto` | `auto` | 靶板怎么动。**建议手工指定，别用 auto** |

**开火**

| 参数 | 值 | 默认 | 说明 |
| --- | --- | --- | --- |
| `--enable-fire` | 无 | **关** | 允许开火。**只在正式比赛、有安全员许可时才加** |

**调试**

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `--debug-view` | 无 | 开图像窗口。**没有滑块**，`q` 退出。左上有 `d=?.??m`，正中有黄十字（代码认为的光心） |
| `--debug-dump` | 无 | 打印跟踪状态（`r` / `v_yaw` / `posdiff` / `yawdiff`） |
| `-h` / `--help` | 无 | 打印帮助。**退出码 1** |

---

### 5.3 三套现成命令，直接抄

**① 离线回放（不碰硬件，调检测参数用）**

```bash
python3 tools/make_test_bag.py /tmp/bag --frames 240

./build/autoaim --replay /tmp/bag --enable-tracker --debug-dump
./build/autoaim --replay /tmp/bag --enable-tracker --debug-view   # 要开窗口
```

**② 真机：只瞄准，不开火（标定和调试阶段一律用这条）**

```bash
./build/autoaim --hik \
  --device /dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00 \
  --fx <内参> --fy <内参> --cx <内参> --cy <内参> \
  --cam-pitch <标定值> --cam-yaw <标定值> --cam-roll <标定值> \
  --armor-w <实测> --armor-h <实测> --thres <现场调> \
  --enable-tracker --rotation carousel --debug-dump
```

**③ 真机：开火（确认瞄得准、且安全员许可之后）**

在 ② 的基础上**只加一个** `--enable-fire`。别的都不要动。

> 📌 判断"参数有没有被吃进去"：启动后第一行会打印
> `[app] 相机=... 串口=... @ 100 Hz 跟踪器=开 开火=关`
> —— 跟踪器和开火的状态在这里能确认。

---

### 5.4 启动时会发生什么（按顺序）

| 步 | 检查 | 失败会怎样 |
| --- | --- | --- |
| 1 | 解析参数 | 报错 + 打印帮助，**退出码 1** |
| 2 | 用了 `--hik` 但这次构建没 SDK | 打印提示，**退出码 1**（[main.cpp:51-57](app/main.cpp#L51-L57)） |
| 3 | 开相机 | `[app] 相机打不开`，**退出码 1** |
| 4 | 开串口 | `[app] 串口打不开: <路径>` + 四行排查提示，**退出码 1** |
| 5 | 起 4 个线程 | 打印 `[app] 相机=... 串口=...@100Hz 跟踪器=... 开火=...` |
| 6 | — | 加了 `--enable-fire` 会多打一行警告 |
| 7 | 主循环 | **每 5 秒**打印一次统计 |

统计那一坨怎么看，见 [PIPELINE.md](readme/PIPELINE.md#零线程与数据槽) 的计数器表。

> ⚠️ **回车之前先确认云台周围是安全的**：`txLoop` 一起来就发 100 Hz，而启动瞬间 `latest_cmd_` 还是空的 → 会先发 `yaw=0, pitch=0` 的默认帧，**可能命令云台回 0 度**。

---

### 5.5 退出码

| 码 | 什么情况 |
| --- | --- |
| `0` | 正常退出（Ctrl-C、`SIGTERM`、回放结束） |
| `1` | 参数错 / 缺值 / `-h` / 没 SDK / 相机打不开 / 串口打不开 |

---

### 5.6 三个容易踩的坑

| 坑 | 说明 |
| --- | --- |
| **`--auto-exposure` 不存在** | 文档老版本写过它，但 [config.cpp](app/config.cpp) 里没有这个分支。传了会被当"未知参数"。自动曝光要改 [config.h](app/config.h#L28) 重编 |
| **`--cam-roll` 能传，但 `--help` 里没列** | 它**能用**，只是帮助文本漏写了 |
| **EKF 噪声、瞄准夹取、开火阈值都没有开关** | 这些全是"编译前"的，改源码重编才生效。见 [PIPELINE.md](readme/PIPELINE.md#参数怎么定--逐个参数的具体取法) |

---

