# 架构说明 —— 每个文件是干什么的、名字从哪来、彼此怎么连

配合 [README.md](README.md) 阅读。README 讲**怎么跑**，本文讲**代码怎么长的**。

- 第零部分：**头文件阅读顺序**（第一次读代码，从这里开始）
- 第一部分：逐个文件说人话
- 第二部分：命名来源（哪些是继承的、哪些是原创的）
- 第三部分：它们之间的关系

---

# 第零部分：头文件阅读顺序

工程自己的头文件共 **23 个、约 2260 行**（`third_party/eigen` 下那 400 多个不用读）。
一次性能读完，但**顺序很重要**。

## 0.1 一条原则：按数据流读，不按目录、不按字母序

数据是单向流动的：**图像 → 装甲板 → 位置 → 状态 → 角度 → 字节**。
头文件的顺序就照这条线走。

**反例**：`app/autoaim_app.h` 字母序排第一，但它 `#include` 了另外 10 个本工程的头文件
—— 先读它等于先看答案，只会一头雾水（"`latest_cmd_` 是什么？`AimSolver` 又是干嘛的？"）。
它排在最后。

## 0.2 第 0 阶段：三个全局契约

先读这三个小文件。后面每一个模块都默认你已经知道它们的约定。

| # | 文件 | 行数 | 读什么 |
| --- | --- | --- | --- |
| 1 | `modules/common/units.h` | 27 | 度↔弧度系数。全局只有这一处定义 |
| 2 | `modules/message_center/latest.h` | 88 | 线程间"小黑板"。**读透注释里"为什么不能用队列"那段**，这是理解四线程架构的钥匙 |
| 3 | `bsp/camera/camera_source.h` | 71 | 两样公共东西：`Frame`（图 + 时间戳）和 `nowNs()`（全工程统一时钟） |

## 0.3 第 1 阶段：主干流水线

跟着一帧图走：检测 → 测距 → 变换 → 跟踪 → 瞄准 → 开火。

| # | 文件 | 行数 | 读什么 |
| --- | --- | --- | --- |
| 4 | `modules/detect/armor.h` | 102 | 装甲板的**数据契约**：两根灯条 + 中心 + 类型 |
| 5 | `modules/detect/number_classifier.h` | 57 | 数字分类器接口。默认不启用，快速扫过 |
| 6 | `modules/detect/armor_detector.h` | 151 | 找靶员：二值化 → 找灯条 → 配对 |
| 7 | `modules/solver/pnp_solver.h` | 102 | 角点 + 已知真实尺寸 → `tvec` / `rvec` |
| 8 | `modules/solver/coord_transform.h` | 106 | **全工程唯一允许定义坐标约定的地方**，重点读 |
| 9 | `modules/tracker/target_state.h` | 56 | 9 维状态向量每一位代表什么 |
| 10 | `modules/tracker/armor_measurement.h` | 41 | 观测数据包，替代 ROS 消息类型，最小 |
| 11 | `modules/tracker/tracker_config.h` | 95 | 公转（CAROUSEL）/ 自转（SELF_SPIN）模式开关 |
| 12 | `modules/tracker/extended_kalman_filter.h` | 116 | 通用 EKF，它自己不懂装甲板 |
| 13 | `modules/tracker/tracker.h` | 186 | 跟踪员，全工程最大的头文件之一 |
| 14 | `modules/aim/aim_solver.h` | 129 | 提前量：补偿约 0.3 秒的延迟 |
| 15 | `modules/aim/fire_decision.h` | 113 | 开火条件组合，数据流的终点 |

## 0.4 第 2 阶段：出口（状态 → 字节）

| # | 文件 | 行数 | 读什么 |
| --- | --- | --- | --- |
| 16 | `bsp/serial/serial_port.h` | 94 | 唯一跟下位机说话的通道 |
| 17 | `modules/protocol/srm_protocol.h` | 119 | 结构体 ↔ 字节，以及 `FrameParser` 流式组帧 |

## 0.5 第 3 阶段：边角与总指挥

| # | 文件 | 行数 | 读什么 |
| --- | --- | --- | --- |
| 18 | `bsp/camera/hik_camera.h` | 73 | 真眼睛（海康 MVS SDK），可略读 |
| 19 | `bsp/camera/replay_source.h` | 79 | 假眼睛，现场调参主力 |
| 20 | `bsp/camera/camera_factory.h` | 48 | 采购员。所有 `#ifdef` 都关在这一个文件里 |
| 21 | `app/config.h` | 139 | 参数总表（它 include 了 5 个模块头，所以排在这儿） |
| 22 | `app/autoaim_app.h` | 203 | **总指挥**，唯一认识所有人的类；`processFrame()` 是九步主流程 |
| 23 | `threads/threads.h` | 66 | 排班表，全工程唯一知道"谁用什么频率跑"的地方 |

## 0.6 读的时候两个提示

- 依赖是**单向**的（`bsp` ← `modules` ← `app` ← `threads`），箭头不会倒着走。
  只有三处例外：`pnp_solver.h` 用了 `detect/armor.h` 的 `Armor`，
  `aim_solver` 和 `tracker.cpp` 用了 `coord_transform.h` 的角度换算。
  都是"用类型 / 用数学"，不是反向调用，见到时不用怀疑自己看错了。
- 主干读完后回头看一眼 **第三部分 3.2「一帧图像的一生」**那张图，
  上面这 23 个文件会一次性各就各位。

---

# 第一部分：每个文件是干什么的

先建立一个大比喻：

> 整个程序 = 一个**打靶机器人**。
> 它每秒看 100 张照片，从照片里找出装甲板，算出云台该转到哪，然后把角度报给下位机。
> 它**不直接开火**，只是"请求开火"，最终扣扳机还要下位机火控 + 鼠标左键 + 安全员同意。

按"离硬件由近到远"排。

## 1. 硬件层 `bsp/`（眼睛和嘴）

| 文件 | 人话版 |
| --- | --- |
| `bsp/serial/serial_port.h/.cpp` | **嘴和耳朵**。打开 `/dev/ttyACM0`，往外写 18 字节，往内读 30 字节。读写各有一把锁，所以多个线程同时用不会串。它是唯一跟下位机说话的通道。 |
| `bsp/camera/camera_source.h` | 一张**"相机应该长什么样"的图纸**（抽象类）。里面还定义了两样公共东西：`Frame`（一张图 + 时间戳）和 `nowNs()`（全工程统一用这个时钟）。它自己没有任何实现。 |
| `bsp/camera/hik_camera.h/.cpp` | **真眼睛**：海康工业相机，走 MVS SDK。注意它不是普通 USB 摄像头，`cv::VideoCapture(0)` 打不开。没装 SDK 就编不了这个文件（CMake 会自动跳过）。 |
| `bsp/camera/replay_source.h/.cpp` | **假眼睛**：把一个文件夹里的图片当成视频播放。这是现场调参的主力——不用排队等车，在自己电脑上就能调。 |
| `bsp/camera/camera_factory.h/.cpp` | **采购员**：看配置决定造"真眼睛"还是"假眼睛"，顺便把所有 `#ifdef` 关在这一个文件里，别的地方保持干净。 |

## 2. 基础零件 `modules/common`、`modules/message_center`

| 文件 | 人话版 |
| --- | --- |
| `modules/common/units.h` | **换算小卡片**：度↔弧度。顺带避开 `M_PI` 在 C++ 里不算标准的问题。 |
| `modules/message_center/latest.h` | **一块小黑板**：线程之间传值用。规则是"**只留最新一份，谁都能反复看**"。为什么不用队列——检测每秒出 60 个角度，发送线程每秒要发 100 次，平均同一个角度要重复发 1.67 次；如果用队列，发送线程会有近一半时间取不到值，云台就一顿一顿的。 |

## 3. 感知链 `modules/detect` + `modules/solver`

| 文件 | 人话版 |
| --- | --- |
| `detect/armor.h` | **数据表格**：一块装甲板长什么样——4 个角点 + 中心 + 颜色类型（RED/BLUE/NONE/SINGLE）。 |
| `detect/armor_detector.h/.cpp` | **找靶员**。输入一张图，输出一列表装甲板。流程：二值化 → 找灯条 → 两根灯条配对成一块装甲板。还带一个"只找到一个灯条"的降级模式。 |
| `detect/number_classifier.h` | **认数字员**（接口）。上游用它区分 1~5 号装甲板；校内赛靶板没数字，**默认不启用**，专门留了接口但没实现。 |
| `solver/pnp_solver.h/.cpp` | **测距员**。由"图上 4 个角点 + 已知装甲板真实尺寸"，反推出"这块板在相机前方多远、姿势朝哪"。输出位置 `tvec` + 姿态 `rvec`。 |
| `solver/coord_transform.h/.cpp` | **翻译官**。把"相机坐标系下的位置"换成"云台坐标系下的位置"——因为相机是斜着装的（默认向下俯 15°）。**这里是全工程唯一允许定义坐标约定的地方**，别处不许自己拼旋转矩阵，否则出错时分不清是检测错、PnP 错还是变换错。 |

## 4. 跟踪与瞄准 `modules/tracker` + `modules/aim`

| 文件 | 人话版 |
| --- | --- |
| `tracker/extended_kalman_filter.h/.cpp` | **数学工具**：一个通用卡尔曼滤波器。它自己不知道靶板是什么，模型由外面传进来。 |
| `tracker/target_state.h` | **状态说明书**：那 9 个数字分别代表什么（旋转中心坐标/速度、高度/速度、朝向/转速、半径）。全工程只有这一处定义。 |
| `tracker/armor_measurement.h` | **观测数据包**：替代上游的 ROS 消息类型，就是一个结构体。 |
| `tracker/tracker_config.h` | **跟踪器参数**：主要是"靶板怎么转"的模式开关。 |
| `tracker/tracker.h/.cpp` | **跟踪员**。输入一连串观测，输出"靶板现在的旋转中心在哪、转多快、半径多大"。核心价值两个：**滤抖动** + **预测未来**（板子侧对相机看不见时，靠预测撑住）。支持两种模式：绕中心公转（CAROUSEL）和原地自转（SELF_SPIN）。 |
| `aim/aim_solver.h/.cpp` | **瞄准手**。输入"靶板在哪、转多快"，输出"云台 yaw/pitch 该指多少"。**关键是提前量**：从曝光到子弹出膛大约 0.3 秒，靶板若 3 秒转一圈，这功夫就转了 34°，不补偿必然脱靶。 |
| `aim/fire_decision.h/.cpp` | **扳机手**。决定什么时候请求开火。条件：对准了 + 稳定持续 0.1 秒 + 距上次够久（0.9 秒）+ 云台反馈没过期。 |

## 5. 通信 `modules/protocol`

| 文件 | 人话版 |
| --- | --- |
| `protocol/srm_protocol.h/.cpp` | **翻译+快递打包**。往外：结构体 → 18 字节；往内：30 字节 → 结构体。里面还有个 `FrameParser` 流式组帧器——因为**USB 每次读到的字节数 ≠ 协议的帧长**，可能读到半帧、一帧半、三帧，必须攒着拼。协议没有魔数也没有 CRC，长度字段是唯一的边界。 |

## 6. 总指挥 `app/`

| 文件 | 人话版 |
| --- | --- |
| `app/config.h/.cpp` | **参数总表 + 命令行解析**。所有能调的数都在这，代码里不许再出现魔数。带 ★ 的是占位符，现场必须实测。 |
| `app/autoaim_app.h/.cpp` | **总指挥（唯一知道全局的人）**。它持有所有模块对象，定义了四个循环，把一帧图的完整处理写成了 `processFrame()`：检测 → 选板 → PnP → 坐标变换 → 跟踪 → 瞄准 → 跳变闸门 → 开火判定。 |
| `app/main.cpp` | **入口**。解析参数、造 App、起线程、等 Ctrl-C。 |
| `app/main_stage1.cpp` | **另一个独立小程序**（另一个可执行文件）：只让云台扫动 + 收反馈打印角度，用来单独验证串口通不通。 |

## 7. 编排与验证

| 文件 | 人话版 |
| --- | --- |
| `threads/threads.h/.cpp` | **排班表**。起 4 个线程、等它们结束。全工程**只有这里**知道"谁用什么频率跑"。 |
| `tests/test_*.cpp` | 6 个离线单元测试，不用车不用相机，在笔记本上就能跑。 |
| `tools/make_test_bag.py` | 造假的图像序列（一块绕圈飞的靶板，真值已知）。 |
| `tools/detect_view.cpp` | 带滑动条的可视化调参工具，现场拖滑动条调二值化阈值。 |
| `tools/e2e_test.py` | 端到端测试：假图像 + 假串口，真的跑一遍编译出来的程序。 |
| `tools/protocol_check.py` | 协议的**第二份独立实现**（Python），用来交叉验证字节序没写反。 |

---

# 第二部分：文件名从哪来

命名来源分成三条线：**上游继承 / 本工程原创 / 第三方 vendor**，加上一层
**"目录结构模仿队内工程、但代码不抄"** 的约定（逐文件来源见 [THIRD_PARTY.md](THIRD_PARTY.md)）。

## 2.1 顶层目录名

| 目录 | 名字来源 |
| --- | --- |
| `autoaim/` | 项目名 = **auto aim**（自瞄），对应赛道名「自瞄赛道」 |
| `bsp/` | **Board Support Package**，嵌入式/机器人圈的标准分层名。这里被弱化成「硬件相关封装」= `serial` + `camera` |
| `modules/` | 功能模块层。名字**直接对齐规则书 §3.3 要求上位机负责的五件事**：检测 / 跟踪 / 瞄准解算 / 指令生成 / 通信 |
| `app/` | 应用层 / 主状态机 |
| `threads/` | 线程编排。注释写明「对应 `control-2026` 的 `os_task.c`」—— **模仿队内工程的职责划分，但代码是新写的** |
| `tests/` | 离线单元测试（6 个） |
| `tools/` | 开发/调试工具 |
| `third_party/` | 约定俗成的 vendor 目录名，只放了 Eigen 3.4.0 |
| `config/` `models/` | 目前是**空占位目录** —— 预留 yaml 配置、ONNX 模型（`models/label.txt` 待补） |

> `bsp/` `threads/` 这两个名字是**工程惯例名**，`control-2026` 里也叫这个。
> 工程刻意在 [COMPLIANCE.md](COMPLIANCE.md) 里说明了「未使用任何 `control-2026` 代码」，避免歧义。

## 2.2 `modules/` 里的模块名来源

| 模块 | 名字来源 |
| --- | --- |
| `protocol/` | 协议编解码。文件名 `srm_protocol.*`，**`SRM` 是队名前缀**（命名空间也是 `srm`，唯一没走 `autoaim::` 的地方） |
| `detect/` | 目标检测 |
| `solver/` | 解算。拆成 `pnp_solver`（位姿）+ `coord_transform`（坐标系变换） |
| `tracker/` | 目标跟踪（EKF） |
| `aim/` | 瞄准层。`aim_solver` = 提前量 + 弹道；`fire_decision` = 开火决策 |
| `common/` | 公共常量，目前只有 `units.h` |
| `message_center/` | 线程间数据交换。文件 `latest.h` 取的是**「Latest 覆盖式共享槽」的语义**——名字直接描述了它的行为（读不是消费） |

## 2.3 逐文件名来源

### A. 派生自开源项目 `rm_auto_aim`（MIT）—— 文件名基本原样保留

| 本工程 | 上游原名 |
| --- | --- |
| `detect/armor.h`、`armor_detector.*` | `armor_detector/.../armor.hpp`、`detector.hpp/.cpp` |
| `detect/number_classifier.h` | `number_classifier.hpp` |
| `solver/pnp_solver.*` | `pnp_solver.hpp/.cpp` |
| `tracker/extended_kalman_filter.*` | `extended_kalman_filter.hpp/.cpp` |
| `tracker/tracker.*` | `armor_tracker/.../tracker.hpp/.cpp` |

配套术语也是 RM 视觉圈通用词：**armor（装甲板）、light bar（灯条）、PnP（Perspective-n-Point）、EKF**。

> 唯一的形式改动：上游是 ROS 风格的 `.hpp/.cpp`，本工程统一成 **`.h/.cpp`**。
> 每个文件头部都保留了 `Copyright (C) 2022 ChenJun / 2024 Zheng Yu` 并写明「派生自 xxx，改了什么」。

### B. 本工程原创 —— 名字描述的是「上游没有、这里补上的那一层」

| 文件 | 取名理由 |
| --- | --- |
| `aim/aim_solver.*` | 上游输出止于 Target 消息，**从没算过瞄准点**，这层是新补的 |
| `aim/fire_decision.*` | 上游没有开火决策 |
| `solver/coord_transform.*` | 上游靠 tf2 + URDF 做变换，非 ROS 环境改为「相机系→云台系」实测反解 |
| `protocol/srm_protocol.*` | 校内赛协议（`self_aim_protocol.md`）的独立实现 |
| `bsp/serial/serial_port.*` | termios 封装 |
| `tracker/target_state.h` | **状态向量契约**（9 维布局），从上游 `tracker_node.cpp` 里拎出来单独定义 |
| `tracker/tracker_config.h` | 上游配置散在 `declare_parameter` 里，这里集中 |
| `tracker/armor_measurement.h` | **替代 ROS 消息类型** `auto_aim_interfaces::msg::Armor` |
| `common/units.h` | 集中定义弧度换算，顺带避开 `M_PI` 非标准的问题 |
| `app/main_stage1.cpp` | **「阶段 1 最小测试程序」**——扫动测串口用，名字来自调试流程的阶段划分 |

### C. 硬件/工具类 —— 名字即来源

| 文件 | 名字来源 |
| --- | --- |
| `bsp/camera/hik_camera.*` | **Hik = 海康**（Hikvision），走 MVS SDK |
| `bsp/camera/replay_source.*` | **replay = 离线回放**，名字强调它是「把录好的图像序列当相机」 |
| `bsp/camera/camera_source.h` | 取帧抽象接口 |
| `bsp/camera/camera_factory.*` | 工厂模式命名 |
| `tools/make_test_bag.py` | **bag** 借的是 ROS bag（录包）的说法，虽然这里只是图像目录 |
| `tools/e2e_test.py` | **e2e = end-to-end** 端到端 |
| `tools/protocol_check.py` | 协议自检 |
| `tools/detect_view.cpp` | 检测可视化调参 |

### D. 根目录文档 + 构建

| 文件 | 来源 |
| --- | --- |
| `README.md` / `CMakeLists.txt` | 通用约定 |
| `THIRD_PARTY.md` | **MIT 许可合规要求**——逐文件声明派生来源，所以必须有 |
| `COMPLIANCE.md` | 自创，**逐条对照 `rules.md` + `self_aim_protocol.md` 做规则符合性检查** |
| `ARCHITECTURE.md` | 本文件 |

## 2.4 代码内部的命名约定

- **文件/变量/函数**：`snake_case`（Google C++ / ROS 2 风格）
- **类型/类**：`CamelCase`（`ArmorDetector`、`AimSolver`、`FireDecision`、`Latest<T>`）
- **命名空间**：`autoaim::<模块名>`，**与目录名一一对应**
  （`detect` / `solver` / `tracker` / `aim` / `bsp` / `app` / `threads` / `config`）；
  例外是 `srm`（协议层，队名前缀）
- **编译宏**：`AUTOAIM_HAVE_MVS` / `AUTOAIM_HAVE_OPENCV` —— `<项目名>_HAVE_<依赖>` 的形式，用来做可选依赖的开关
- **常量**：`k` 前缀（`kDeg2Rad`、`kRad2Deg`）
- **测试文件**：`test_<被测模块>.cpp`，与模块名对齐

---

# 第三部分：它们之间的关系

## 3.1 串起来的是「数据流」，不是「互相调用」

最重要的一点：**模块之间基本互不认识**。只有 `AutoAimApp` 认识所有人。

```
                       threads/     ← 只负责"什么时候调用"
                          │
                          ▼
   ┌─────────────────  app::AutoAimApp  ─────────────────┐
   │         （唯一知道全局的地方，把模块串起来）           │
   └──┬────────┬────────┬────────┬────────┬────────┬─────┘
      │        │        │        │        │        │
   serial   camera  detector   pnp    tracker   aim/fire
      │        │        │        │        │        │
      └────────┴────────┴─── 被调用，但不反过来调用别人 ──┘
```

`bsp` ← `modules` ← `app` ← `threads/main`，**依赖是单向的**，箭头不会倒着走。

只有三个例外，而且都是"用类型/用数学"，不是"调功能"：

- `pnp_solver.h` 用 `detect/armor.h` 里的 `Armor` 结构体（要拿角点）
- `aim_solver.cpp` 用 `solver/coord_transform.h` 的角度换算、用 `tracker/target_state.h` 的状态布局
- `tracker.cpp` 用 `solver/coord_transform.h` 把四元数转成连续 yaw

## 3.2 一帧图像的一生

这是理解整个工程的**最快捷径**。跟着一张图走一遍：

```
【相机线程 cameraLoop】
   取一帧 ──► latest_frame_.set(frame)          ← 放进小黑板
                    │
                    ▼
【处理线程 processLoop】—— 只要黑板上有新帧就干活
   ① detector_.detect()        找出装甲板     → 一列表
   ② 选离画面中心最近的那块
   ③ pnp_->solvePnP()         算出位置/姿态   → tvec, rvec
   ④ 重投影误差闸门            误差太大就丢弃
   ⑤ cameraToGimbal(tvec)     相机系 → 云台系
   ⑥ tracker_->update()       滤波 + 预测     → 9 维状态
   ⑦ aim_.solveFromState()    算提前量        → yaw/pitch
   ⑧ 跳变闸门                  单帧跳太多就保持上次角度
   ⑨ fire_.update()           决定要不要开火  → fire_flag
                    │
                    ▼
   publishCommand() ──► latest_cmd_.set(cmd)    ← 再放一块小黑板
                    │
                    ▼
【发送线程 txLoop】—— 100Hz 定频，雷打不动
   取最新值 → pack_target() 打成 18 字节 → serial_.write_bytes()
                    │
                    ▼
                  下位机（云台）

              ↑ 30 字节反馈原路返回
【接收线程 rxLoop】
   serial_.read_bytes() → FrameParser::feed() 组帧 → parse_feedback()
                    │
                    ▼
   latest_feedback_.set(fb) ──► 处理线程用它算"云台现在转到哪了"
                                （开火判定 + 下一步的收敛判断）
```

**两块小黑板就是整条流水线的关节**：`latest_frame_`（相机→处理）和
`latest_cmd_`（处理→发送）。反馈那块 `latest_feedback_` 是反向的第三条。

## 3.3 为什么是四个线程，各自什么节奏

| 线程 | 频率 | 干什么 | 为什么必须独立 |
| --- | --- | --- | --- |
| `cameraLoop` | 相机帧率 ~60 | 只管取帧，塞黑板 | 取帧会阻塞，不能拖累别人 |
| `processLoop` | 有新帧就跑 | 检测+解算+瞄准（重活） | 最慢的一环，绝不能挡住发送 |
| `txLoop` | **100 Hz 定频** | 取最新值→打包→写串口 | 协议要求持续发；这线程里**只许做这三件事**，打印/存图都会让定频抖动 |
| `rxLoop` | 收到就读 | 读 30 字节→组帧→解析 | 读会阻塞 |

关键设计：**发送是定频的，处理不是**。所以发送线程会把同一个角度重复发好几次——
这正是 `Latest` 必须"读而不是消费"的原因。这是 `latest.h`、`threads.h`、
`autoaim_app.h` 三个文件咬合的地方。

## 3.4 坐标与时间的「单一真相源」

工程里有两条贯穿所有模块的主线，各自只有一个定义处：

```
坐标：OpenCV 相机系 ──coord_transform──► 云台系 ──aim_solver──► yaw/pitch
        (x右 y下 z前)                    (x前 y左 z上)
       ↑ 全工程只允许 coord_transform.h 定义这个约定

时间：所有时间戳统一用 nowNs() 单调时钟（camera_source.h 定义）
      角度：内部一律弧度，只有协议帧用角度（srm_protocol.h 明说）
```

## 3.5 编译上怎么组装

```
third_party/eigen  ─┐
系统 OpenCV        ─┼─►  autoaim_core（静态库）
海康 MVS SDK       ─┘         │
                              ├─► autoaim         (app/main.cpp)
                              └─► main_stage1     (app/main_stage1.cpp)
```

`CMakeLists.txt` 用 `GLOB` 把 `bsp/ modules/ threads/ app/` 下所有 `.cpp` 收进
`autoaim_core`，然后把两个带 `main()` 的文件排除掉，单独编成可执行文件。
OpenCV 和 MVS SDK 都是**可选的**——没有它们，`detect/solver/camera` 会被过滤掉，
但协议、跟踪、瞄准部分依然能编译和测试。

---

## 一句话总结

**`bsp` 管"数据从哪来、往哪去"，`modules` 是六个互相不认识的专业工位，
`app/AutoAimApp` 是唯一的工头，用两块小黑板（`Latest`）把工位串成流水线，
`threads` 排班决定节奏，`protocol` 是跟下位机说话的翻译。**

数据单向流动：**图像 → 装甲板 → 位置 → 状态 → 角度 → 字节。**
