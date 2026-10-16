# autoaim —— RoboMaster 校内赛自瞄上位机

从图像到云台指令的完整链路：**装甲板检测 → PnP 位姿解算 → 坐标变换 → EKF 跟踪 →
弹道规划（MPC） → 串口下发**。跑在 NUC / 工控机上，通过串口和 STM32 下位机通信。

C++17 / CMake / OpenCV。核心算法全部有离线单元测试，不接硬件也能跑通整条流水线。

---

## 快速开始

```bash
git clone git@github.com:queen1166/autoaim.git
cd autoaim

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# 造一段合成回放数据（不需要相机、不需要车）
python3 tools/make_test_bag.py /tmp/bag --frames 240

# 跑起来
./build/autoaim --replay /tmp/bag --device /dev/ttyACM0 --debug-view
```

**神经网络探测是默认路径，模型文件已经随仓库提供**（`models/Infantry-v8n-opencv.onnx`），
上面的命令不带任何 `--yolo*` 参数就会走它，启动时会打印：

```
[yolo] 已加载 models/Infantry-v8n-opencv.onnx（输入 640x480，9 类，关键点 2 维，color-class-kpt 布局，输入缩放 1）
```

> ⚠️ 模型路径是相对**进程当前目录**的，所以要在**仓库根目录**跑 `./build/autoaim`。
> 从 `build/` 里跑会落空（程序会打印它实际找的绝对路径）并自动回退传统视觉。
> 想显式用传统视觉（二值化 + 灯条配对）加 `--no-yolo`。

---

## 环境要求

| 依赖 | 版本 | 说明 |
| --- | --- | --- |
| Ubuntu | 22.04 | 本项目实际使用的环境（WSL 亦可） |
| OpenCV | **4.5.4** | `sudo apt install libopencv-dev`。★ 唯一实测过的版本，见下 |
| CMake | ≥ 3.16 | |
| 编译器 | 支持 C++17 | `build-essential` |
| Eigen | — | **已 vendor**，不用装 |
| 海康 MVS SDK | — | **可选**，只有真机相机需要（`--hik`） |

> ⚠️ **OpenCV 版本**：Ubuntu 22.04 的 `libopencv-dev` 是 4.5.4，这是唯一验证过的版本。
> [CMakeLists.txt](CMakeLists.txt) 用的是 `find_package(OpenCV QUIET ...)`，**不校验版本**，
> 所以更新的 OpenCV（如 24.04 的 4.6+）也能配置、能编过 —— 但**没人验过模型在那上面能不能跑**。
> 出问题的症状是加载成功但 `forward()` 抛异常，不是编译错误。

---

## 真机（比赛机器）

在快速开始的基础上还要做四件事，**按顺序**走
[`readme/RUNBOOK.md`](readme/RUNBOOK.md) 的「二、比赛机器上的一次性准备」：

1. 系统依赖 + 海康 MVS SDK（装之前先问赛务）
2. 串口权限（`dialout` 组）与设备名
3. **标定四件套**：相机内参、外参、灯条几何、二值化阈值 —— 默认值全是占位符，必须实测

标定没做之前，瞄准角是没有意义的。

---

## 文档导航

| 文档 | 讲什么 |
| --- | --- |
| [`readme/RUNBOOK.md`](readme/RUNBOOK.md) | **从零开始运行** —— 编译、比赛机器准备、现场流程、全部命令行参数速查 |
| [`readme/TESTS.md`](readme/TESTS.md) | 测试清单 —— 9 个单测 + 工具类测试，以及「改了什么就至少跑什么」 |
| [`readme/git_workflow.md`](readme/git_workflow.md) | Git/GitHub 两机协作速查 |
| [`models/pretrained/README.md`](models/pretrained/README.md) | 模型文件说明 —— 规格、来源、为什么要改写、怎么验证 ONNX 形状 |
| [`third_party/THIRD_PARTY.md`](third_party/THIRD_PARTY.md) | 第三方代码与模型的来源与许可声明 |

`上海科技大学2026视觉技术方案开源.md` 是一份第三方开源教程（坐标系、卡尔曼、
各赛题场景的实现思路），本工程的代码大量参考了它，**但它不是本工程的文档**。

---

## 安全

**默认不开火。** `--enable-fire` 只在正式比赛、且有安全员许可时才加。
`txLoop` 一起来就按 100 Hz 下发，启动瞬间可能先发一帧 `yaw=0, pitch=0` ——
**回车之前先确认云台周围是安全的**。
