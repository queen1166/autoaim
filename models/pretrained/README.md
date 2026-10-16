# models/pretrained —— 基座权重，不是装甲板模型

> **本目录只有这份 README 进版本控制。** 下面那两个基座权重在 `.gitignore` 里，
> 新 clone 的仓库里不会有；要它们的话按文中链接自己下。
>
> 只想跑通装甲板探测的话，直接看下文「能直接用的装甲板模型：深大 RM26」——
> 那个模型**已经随仓库分发**在上一层的 `models/` 里，本目录这两个文件不用管。

## ⚠️ 本目录这两个文件**不能**直接用来打装甲板

这里放的是 Ultralytics 官方发布的 **COCO 人体姿态** 基座权重。它们是**训练用的起点**，
不是装甲板检测模型。直接喂给 `--yolo` 会失败。

原因（实测）：

| 项 | 基座权重实测值 | 本程序要求 |
| --- | --- | --- |
| 关键点数 | 17（人体骨架） | **4**（装甲板四角） |
| 类别 | `{0: 'person'}` | 装甲板 |
| ONNX 输出形状 | `[1, 56, 8400]` | **`[1, 17, 8400]`** |

`56 = 4(bbox) + 1(类别) + 17×3`，`17 = 4 + 1 + 4×3`。
通道数对不上时 [yolo_pose.cpp](../../modules/detect/yolo_pose.cpp) 的 `decode()` 会打印
`[yolo] 输出通道数 56 与配置不符` 并返回空列表，**不会回退**——表现为"一帧都检不出来"。

## 文件清单

| 文件 | 大小 | 说明 |
| --- | --- | --- |
| `yolov8n-pose.pt` | 6.8 MB | 训练起点（nano，最小最快） |
| `yolov8n-pose.onnx` | 13.5 MB | 同上的 ONNX 导出，输入 `images [1,3,640,640]`，输出 `output0 [1,56,8400]` |

来源：`https://github.com/ultralytics/assets/releases/tag/v8.4.0`

## 怎么验证任何 ONNX 的形状

**最省事**：把 `.onnx` 拖进 [Netron](https://netron.app/)（网页版就行），看输出层的 shape。

**看命令行**：装过 onnxruntime 的话

```bash
python -c "import onnxruntime as ort; s=ort.InferenceSession('模型.onnx'); \
print('in ', [(i.name,i.shape) for i in s.get_inputs()]); \
print('out', [(o.name,o.shape) for o in s.get_outputs()])"
```

判断标准：

| 输出形状 | 结论 |
| --- | --- |
| `[1, 17, N]` | ✅ 能用（ultralytics 布局） |
| `[1, 56, N]` | ❌ COCO 人体 17 关键点（就是本目录这两个） |
| `[1, 5, N]` | ❌ 普通检测模型，只有框没有关键点 |
| `[1, 21, N]` | ✅ 能用（深大布局，但要 `--yolo-layout color-class-kpt`，见下） |
| `[1, N, 17]` | ⚠️ 转置布局，布局对但轴序反了，需要改 `decode()` |

判据是 `通道数 == 4 + 类别数 + 4 × 每关键点数值个数`，两种布局共用这个公式，区别只在前
4 个通道装的是什么（bbox 还是颜色分数）。

## 能直接用的装甲板模型：深大 RM26

**来源**：`https://github.com/SZURPVision/26_NNDeployment_Lib_and_Detection_Models`
（RoboMaster 上海科技大学校内赛用到的开源部署库，490 MB，其中约 450 MB 是测试视频，
**不要整个 clone**，只下 `所有模型/onnx/` 里的那一个文件即可。）

**本仓库里的文件**：`models/Infantry-v8n-opencv.onnx` —— 这是改写后的可用版本，
**随仓库分发**（`.gitignore` 里 `*.onnx` 的唯一例外），clone 下来就能用。

原始下载件是 `models/Infantry-v8n.onnx`，本机留了一份做对照，但**不进版本控制**：
它在 OpenCV 4.5.4 上加载成功、`forward()` 会抛异常，提交它等于给队友埋雷。

### 规格（实测）

| 项 | 值 |
| --- | --- |
| 输入 | `images [1,3,480,640]`，取 **0–255 原始像素值** |
| 输出 | `output0 [1,21,6300]`，6300 = 80×60 + 40×30 + 20×15 |
| 21 = | 4 颜色分 + 9 类别分 + 4 关键点 ×(x, y)，**没有 bbox、没有关键点置信度** |
| 关键点 | letterbox 后的 640×480 像素坐标，顺序 TL,BL,BR,TR（顺序无所谓，见下） |
| 类别 | 0 哨兵 / 1 英雄 / 2 工程 / 3~5 号步兵 / 6 前哨站 / 7 基地底 / 8 基地顶 |
| 颜色 | 0 蓝 / 1 红 / 2 白 / 3 紫 —— **和 [armor.h](../../modules/detect/armor.h) 的 `RED=0`/`BLUE=1` 相反** |

关键点顺序不影响代码：[`buildArmorFromKeypoints()`](../../modules/detect/yolo_pose.cpp)
按 x 分左右灯条、灯条内按 y 分上下端点，与训练时的排列约定无关。

### 必须改的两个地方

这个头有两处和 ultralytics 常规假设冲突，所以加了 `--yolo-layout color-class-kpt` 分支：

1. **通道 0–3 是颜色分数，不是 `cx,cy,w,h`。** 直接当 bbox 读，所有候选框会退化成原点附近
   1px 的小框，被 NMS 互相压制到只剩一个——不是"检不出来"，是更隐蔽的"总是只有一两个框"。
   该分支改成用 4 个关键点的外接矩形当框。
2. **输入要吃 0–255。** 按惯例除以 255 后输出全通道恒为 0，一帧都检不出来。
   `--yolo-layout` 会顺带把 `input_scale` 设成 1.0（`ultralytics` 则设成 1/255）。

这两点代码里都已经按本模型实现，而且是**默认值**——不带任何参数启动就走这条路。

### 原始文件为什么要先转换（两步，都必须做）

`Infantry-v8n.onnx` 是从 OpenVINO 转出来的，有两个坑，`Infantry-v8n-opencv.onnx` 是两步都
做过的结果，直接用即可。

**第一步：去掉 `Cast` 节点（否则加载就失败）**

原始文件首尾各留了一个 `Cast` 节点，`cv::dnn::readNetFromONNX` 会在第一个 Conv 处直接抛
`inputs.size() != 0`——**不是 fp16 的锅，也不是 opset 的锅**（fp32、opset 12 都试过，照样失败）。
旁路掉那两个 `Cast` 并把 147 个 fp16 张量转成 fp32 之后才能加载。

**第二步：改写检测头（否则加载成功但 forward 崩溃）**

这一步是被 Ubuntu 22.04 自带的 **OpenCV 4.5.4** 逼出来的（4.13 上原样就能跑）。实测：
**4.5.4 的 DNN 里，`Add/Mul/Sub/Div` 只要第二个操作数不是标量（1 个元素），一律抛**

```
OpenCV(4.5.4) shape_utils.hpp:170: error: (-215:Assertion failed)
  start <= (int)shape.size() && end <= (int)shape.size() && start <= end in function 'total'
```

我用最小模型把 4 种算子 × 6 种常量形状跑了一遍，结论是只有 `标量 + 4D输入` 这一格能过。
而这个模型头上正好有一句 `Mul(..., 一个 shape (1,6300) 的 stride 常量)`：

```
bbox 三分支 → Reshape[1,4,2,6300] → ×2.0 → +grid(2,6300) → ×stride(1,6300) → Reshape[1,8,6300]
                                     ↑ 标量，能过   ↑ 能过（是 initializer）  ↑ 就是这里崩
```

改写利用 `(2c + g)·S == 2S·c + g·S`，把乘法拆到三个分支各自去做：

- 每个 bbox 分支的卷积输出后面插一个**标量** `Mul`，乘 `2×stride`（8/16/32 → 16/32/64）；
- 全局那句 `×2.0` 和 `×stride` 删掉；
- grid 常量提前乘好 stride，变成 `g·S`。

全程只剩标量乘，两个版本在 OpenCV 4.13 下输出**逐位相同**（max abs diff = 0）；在 4.5.4 下
与 4.13 参考值 max abs diff = 0.012（fp32 累加顺序差异），检出结果一致。

> 换模型的话这步得重做。改写的是 `model.22` 那 8 个节点，位置和形状见上表；三个分支的对应
> 关系靠 anchor 数确认（4800/1200/300 = 80×60/40×30/20×15 = stride 8/16/32）。

### 跑起来的命令

**不用传任何参数**——这个模型的路径和上表那套参数已经是代码里的默认值
（[modules/detect/yolo_pose.h](../../modules/detect/yolo_pose.h) 的 `YoloPoseConfig`），
不带 `--yolo` 启动就走神经网络：

```bash
./build/autoaim --replay bags/latest --debug-view
```

想显式写出来（或者换成别的模型）：

```bash
./build/autoaim --replay bags/latest --yolo models/Infantry-v8n-opencv.onnx \
  --yolo-layout color-class-kpt --debug-view
```

> 模型路径是相对**进程当前目录**的，所以要在仓库根目录跑 `./build/autoaim`。
> 从别的目录跑会落空，提示里会打出它实际找的绝对路径，然后自动回退传统视觉。
> 这个 `.onnx` **随仓库分发**，`git clone` 下来就在，不需要另外下载或转换。

已验证：用一张真实装甲板照片能出 `1.000 前哨站` 的检出，四角几何合理；
在 WSL 的 OpenCV 4.5.4 上跑 `--replay` 240 帧全程无异常，退出码 0。

> ⚠️ 那张验证用的照片**不在仓库里**（当时是临时找的测试图）。程序本身不依赖它，
> 但想复现这次检出验证的话得自己准备一张真实靶板照片。

### 没接进管线的部分

- **4 个颜色通道没有写进 `Armor::color`**，9 类标号也没写进 `tag_id`。本赛题只有一种靶板、
  一个目标，敌我过滤和标号都用不上；而且颜色枚举顺序是反的（见上表），要用得单独做。
- 大/小装甲板判定改为按关键点外接矩形的宽高比（`YoloPoseConfig::large_aspect_ratio`，
  默认 1.4）。**上场前用真实靶板确认一次这个判定**，它会决定 PnP 选哪套 3D 物点。

## 另外两个基座权重留着

`yolov8n-pose.pt` / `.onnx` 是训练起点（人体 17 关键点），要用得自己采集标注装甲板四角
数据做微调。

**这两个文件不进版本控制** —— `.gitignore` 的 `*.onnx` / `*.pt` 管着它们（共 20 MB，
比赛用不上）。所以新 clone 的仓库里**本目录只有这份 README**，那两个基座权重得另外去
上面那个 ultralytics release 链接下载。装甲板模型不受影响，它是 `.gitignore` 里唯一的例外、
随仓库分发。
