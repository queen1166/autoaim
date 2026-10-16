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

---

## 二、比赛机器上的一次性准备

按顺序做，四步。

1. **系统依赖**。Ubuntu 22.04 上：

   ```bash
   sudo apt install -y build-essential cmake libopencv-dev python3-opencv
   ```

   `libopencv-dev` 在 22.04 上就是 **4.5.4**，这是本项目**唯一实测过的版本**
   （[CMakeLists.txt:39](CMakeLists.txt#L39) 用 `find_package(OpenCV QUIET ...)`，
   **不校验版本**，所以更新的 OpenCV 也能编过，但没人验过模型能不能跑）。
   Eigen 已 vendor 在 `third_party/eigen`，**不用装**。

2. **模型文件**。**不用做任何事** —— 装甲板模型随仓库分发，`git clone` 之后就在
   `models/Infantry-v8n-opencv.onnx`，程序默认探测路径就是它。

   > 所以一定要在**仓库根目录**跑 `./build/autoaim`：模型路径是相对进程当前目录的。
   > 从别处跑会落空、回退传统视觉（提示里会打出它实际找的绝对路径）。

3. **海康 MVS SDK**。这是最高优先级风险项，装之前先问赛务。允许的话从海康官网下载 .deb
   装到 `/opt/MVS`，然后重新配置和编译：

   ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ```

   配置时出现"海康 MVS SDK: 已找到"就成功。没装时 `--hik` 会在启动时被拒绝并提示。

4. **串口权限**。先用 `ls /dev/serial/by-id/` 记下真实设备名；把用户
   加进 dialout 组后重新登录。共用机器上的权限改动先跟赛务确认。

5. **标定四件套**。默认值都是占位符，必须实测：

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

   > 第 3 步之前做不了 4 和 5（要相机和串口）。第 5 步的阈值只在**传统视觉**后端生效，
   > 默认走神经网络时它不起作用，见 §4.6。

---

## 三、现场每次怎么跑

只瞄准不开火，把标定值填齐。**从仓库根目录运行**（模型路径是相对的）。

```bash
./build/autoaim --hik --device /dev/serial/by-id/usb-XXX \
  --fx <内参> --fy <内参> --cx <内参> --cy <内参> \
  --cam-pitch <标定值> --cam-yaw <标定值> --cam-roll <标定值> \
  --armor-w <实测> --armor-h <实测> \
  --enable-tracker --rotation auto --debug-dump
```

只在正式比赛、有安全员许可时才加 `--enable-fire`。平时调试和第一次上场都不加。

> 📖 **每个参数的值怎么写、格式规则、四套现成命令** → 见 [第四部分](#四命令行传参速查autoaim-主程序)。

---

## 四、命令行传参速查（`autoaim` 主程序）

### 4.1 格式规则

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

### 4.2 全部参数一览

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

**探测后端**

> **默认就是神经网络**，`./build/autoaim` 不带任何参数就走 YOLO-Pose，参数已经按仓库自带的
> 那个模型填好了。下表只有在**换模型**或**显式用传统视觉**时才需要动。

| 参数 | 值 | 默认 | 说明 |
| --- | --- | --- | --- |
| `--no-yolo` | 无 | — | 改用传统视觉（二值化 + 灯条配对）。**调传统参数时必须加**，见 §4.3 ① |
| `--yolo <模型.onnx>` | 路径 | `models/Infantry-v8n-opencv.onnx` | 换成别的 YOLO-Pose 模型 |
| `--yolo-layout <mode>` | `ultralytics` \| `color-class-kpt` | `color-class-kpt` | 输出头布局。**一键切整套惯例**：W×H / 类别数 / kpt_dim / 输入缩放一起变 |
| `--yolo-w` `--yolo-h <px>` | 数字 | 640 / 480 | 输入尺寸。★ **要跟模型一致**，见 §4.6 |
| `--yolo-classes <n>` | 数字 | `9` | 类别数（ultralytics 惯例是 `1`） |
| `--yolo-kpt-dim <n>` | `2` 或 `3` | `2` | 每个关键点几个数（ultralytics 惯例是 `3`） |
| `--yolo-input-scale <f>` | 数字 | `1.0` | 输入缩放（ultralytics 惯例是 `1/255`）。少用 |
| `--yolo-conf <f>` | 0~1 | `0.25` | 检出置信度门限 |
| `--no-corner-refine` | 无 | — | 关掉 PCA 角点精修。两种探测后端都受影响 |

`--yolo-layout` 给的是那一组惯例的默认值；**显式写了的 `--yolo-*` 永远赢**，且与书写顺序无关
（`--yolo-classes 5 --yolo-layout color-class-kpt` 和反着写都是 5 类）。

> 默认模型是深大 RM26 装甲板模型，**随仓库分发**（`models/Infantry-v8n-opencv.onnx`），
> clone 下来就能用，不用另外下载或转换。规格、来源和改写原因见
> [models/pretrained/README.md](../models/pretrained/README.md)。
> 那个文件是**为 OpenCV 4.5.4 改写过的**——原始下载件在 4.5.4 上加载成功但
> `forward()` 会抛异常，别拿别的转换版本替换它。
>
> 路径是相对**进程当前目录**的，所以要在仓库根目录跑；从别处跑会落空并回退传统视觉
> （见 §4.4），提示里会打出它实际找的绝对路径。

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

### 4.3 四套现成命令，直接抄

**① 离线回放（不碰硬件，调传统视觉检测参数用）**

```bash
python3 tools/make_test_bag.py /tmp/bag --frames 240

./build/autoaim --replay /tmp/bag --no-yolo --enable-tracker --debug-dump
./build/autoaim --replay /tmp/bag --no-yolo --enable-tracker --debug-view   # 要开窗口
```

> ★ 这条是调**传统视觉**（`--thres` / `--color` 那一套）用的，**必须加 `--no-yolo`**。
> 不加的话默认先加载 12.7 MB 神经网络模型，`--thres` 全都不生效，回放还慢 50 倍。

**② 真机：只瞄准，不开火（标定和调试阶段一律用这条）**

```bash
./build/autoaim --hik \
  --device /dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00 \
  --fx <内参> --fy <内参> --cx <内参> --cy <内参> \
  --cam-pitch <标定值> --cam-yaw <标定值> --cam-roll <标定值> \
  --armor-w <实测> --armor-h <实测> \
  --enable-tracker --rotation carousel --debug-dump
```

> `--thres` / `--color` 只在探测后端是**传统视觉**时才有效，默认走神经网络所以上面没写它们。
> 想按传统视觉调好再上真机，用 ① 那条命令调，上真机时加 `--no-yolo` 带上调好的值。

**③ 真机：开火（确认瞄得准、且安全员许可之后）**

在 ② 的基础上**只加一个** `--enable-fire`。别的都不要动。

**④ 用神经网络探测（深大 RM26 装甲板模型；这是默认，不用传参数）**

```bash
./build/autoaim --replay /tmp/bag --enable-tracker --debug-view
```

就这么多。① 和 ④ 的区别只是 `--no-yolo` 有没有。

启动后确认这一行，参数对不对全在这里：

```
[yolo] 已加载 models/Infantry-v8n-opencv.onnx（输入 640x480，9 类，关键点 2 维，color-class-kpt 布局，输入缩放 1）
```

**换成别的模型 / 切回 ultralytics 惯例：**

```bash
--yolo <模型.onnx>                  # 只换路径，其余参数照旧
--yolo-layout ultralytics           # 换一整套惯例：640×640 / 1 类 / kpt 3 / 缩放 1÷255
--yolo-layout ultralytics --yolo-classes 5   # 在整套惯例上再覆盖单项，显式值优先
```

> 默认模型**随仓库分发**（`.gitignore` 里 `*.onnx` 的唯一例外），clone 下来就在，
> 换机器不用另外拷。要新增别的模型才需要自己放，放哪都行、用 `--yolo` 指过去。
> 模型路径是相对**进程当前目录**的，所以要按上面那样从仓库根目录跑 `./build/autoaim`；
> 从别处跑会落空、回退传统视觉（不算故障，见 §4.4），提示里会打出它实际找的绝对路径。

> 📌 判断"参数有没有被吃进去"：启动后第一行会打印
> `[app] 相机=... 串口=... @ 100 Hz 跟踪器=开 开火=关`
> —— 跟踪器和开火的状态在这里能确认。

---

### 4.4 启动时会发生什么（按顺序）

| 步 | 检查 | 失败会怎样 |
| --- | --- | --- |
| 1 | 解析参数 | 报错 + 打印帮助，**退出码 1** |
| 2 | 用了 `--hik` 但这次构建没 SDK | 打印提示，**退出码 1**（[main.cpp:51-57](app/main.cpp#L51-L57)） |
| 3 | 开相机 | `[app] 相机打不开`，**退出码 1** |
| 4 | 开串口 | `[app] 串口打不开: <路径>` + 四行排查提示，**退出码 1** |
| 5 | 加载 YOLO 模型（默认） | `[yolo] 找不到模型文件: <路径>` + `[app] YOLO 模型不可用 → 回退到传统视觉探测`。**不算启动失败，退出码仍 0**，后面照常用传统视觉跑。默认模型随仓库分发、正常不会走到这一格；撞上通常是**没在仓库根目录运行**（模型路径是相对的），提示里会打出它实际找的绝对路径 |
| 6 | 起 4 个线程 | 打印 `[app] 相机=... 串口=...@100Hz 探测器=... 跟踪器=... 开火=...` |
| 7 | — | 加了 `--enable-fire` 会多打一行警告 |
| 8 | 主循环 | **每 5 秒**打印一次统计 |

> ⚠️ **回车之前先确认云台周围是安全的**：`txLoop` 一起来就发 100 Hz，而启动瞬间 `latest_cmd_` 还是空的 → 会先发 `yaw=0, pitch=0` 的默认帧，**可能命令云台回 0 度**。

---

### 4.5 退出码

| 码 | 什么情况 |
| --- | --- |
| `0` | 正常退出（Ctrl-C、`SIGTERM`、回放结束） |
| `1` | 参数错 / 缺值 / `-h` / 没 SDK / 相机打不开 / 串口打不开 |

> 模型文件找不到**不在**这个表里——它是回退，不是失败。

---

### 4.6 五个容易踩的坑

| 坑 | 说明 |
| --- | --- |
| **`--auto-exposure` 不存在** | 文档老版本写过它，但 [config.cpp](app/config.cpp) 里没有这个分支。传了会被当"未知参数"。自动曝光要改 [config.h](app/config.h#L28) 重编 |
| **`--cam-roll` 能传，但 `--help` 里没列** | 它**能用**，只是帮助文本漏写了 |
| **EKF 噪声、瞄准夹取、开火阈值都没有开关** | 这些全是"编译前"的，改源码重编才生效 |
| **离线调 `--thres` 忘了加 `--no-yolo`** | 默认后端是神经网络，`--thres` / `--color` 完全不生效。日志里没有报错，只是"调了半天阈值一点变化都没有"，而且每帧慢 50 倍（40 ms vs 0.8 ms）。**调传统参数一律加 `--no-yolo`**（§4.3 ①） |
| **手工覆盖 `--yolo-*` 时破坏了整套惯例的一致性** | `--yolo-layout` 会给 W×H / 类别数 / kpt_dim / 输入缩放**一起**填默认值，所以光用 layout 是安全的。坑在于手工再覆盖单项：只改 `--yolo-w/-h` 不改别的、或把 `--yolo-layout` 和 `--yolo-input-scale` 配错。**配错不会报错，只会一帧都检不出来**——缩放错→输出恒为 0；`ultralytics` 布局喂深大模型→前 4 个通道是颜色分数被当 bbox 读，所有框退化成 1px 小框。两者都照常打印 `[yolo] 已加载 ...`，只能靠"没有检出"发现。**解法是看启动那行的 layout / 尺寸 / 缩放值对不对** |

---

