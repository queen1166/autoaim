# 数据流水线详解 —— 从一帧图像到下位机的控制命令

> 本文对着**当前代码**逐站拆解：每一站的**输入类型 → 算法 → 输出类型 → 谁在这里调参 → 失败了怎么降级**。
> 想看"现场该测什么"看 [FIELD_CHECKLIST.md](readme/FIELD_CHECKLIST.md)；想看"为什么这么设计"看 [ARCHITECTURE.md](readme/ARCHITECTURE.md)。

---

## 一页速览

```
                    ┌──────────── 相机 ────────────┐
                    │  Hikrobot CS016-10UC (USB3)  │
                    └──────────────┬───────────────┘
                                   │ cv::Mat (RGB 8UC3) + 纳秒时间戳
 ┌─────────────────────────────────▼─────────────────────────────────┐
 │  ② 二值化 + 找灯条         → vector<Light>    (top/bottom 两个端点) │
 │  ③ 配对成装甲板            → Armor            (左右灯条 + 类型)     │
 │  ④ PnP 解算                → rvec / tvec      (旋转 + 平移)         │
 │  ⑤ 相机系 → 云台系          → Vector3d / Quat  (米)                 │
 │  ⑥ EKF 跟踪                → VectorXd(9 维)   (位置 + 速度 + yaw + r)│
 │  ⑦ 瞄准解算                → AimResult       (目标 yaw / pitch)    │
 │  ⑧ 跳变闸门                → AimResult 或 保持                     │
 │  ⑨ 开火决策                → int fire (0/1)                        │
 └─────────────────────────────────┬─────────────────────────────────┘
                                   │ TargetCommand{float yaw, float pitch, int fire}
                                   ▼
                        18 字节小端帧 @ 100 Hz
                                   │
                              ┌────▼────┐
                              │  下位机  │
                              └────┬────┘
                                   │ 30 字节反馈 @ 下位机频率
                                   └──────► 回到 ⑦（弹速）和 ⑨（云台实际角度）
```

**一句话概括**：**找到两根灯条的四个端点 → 用这四点反推装甲板的三维位姿 → 让滤波器算出它在怎么动 → 补偿飞行时间算出该瞄哪 → 打成 18 字节发出去。**

---

# 零、线程与数据槽

程序是**四线程**，线程之间**只通过 `Latest<T>` 槽位传数据，不直接调用**。

| 线程 | 代码 | 干什么 | 读 | 写 |
| --- | --- | --- | --- | --- |
| `camera` | `AutoAimApp::cameraLoop` | 只管抓图 | — | `latest_frame_` |
| `process` | `AutoAimApp::processLoop` | **①~⑨ 全在这里（唯一的重活）** | `latest_frame_`、`latest_feedback_` | `latest_cmd_` |
| `tx` | `AutoAimApp::txLoop` | 100 Hz 定频发包 | `latest_cmd_` | 串口 |
| `rx` | `AutoAimApp::rxLoop` | 收反馈、组帧 | 串口 | `latest_feedback_` |

### 三个数据槽

| 槽 | 类型 | 谁写 | 谁读 |
| --- | --- | --- | --- |
| `latest_frame_` | `Latest<bsp::Frame>` | camera | process |
| `latest_cmd_` | `Latest<srm::TargetCommand>` | process | tx |
| `latest_feedback_` | `Latest<srm::GimbalFeedback>` | rx | process |

`Latest<T>`（[latest.h](modules/message_center/latest.h)）是个**带互斥锁 + 序号计数器**的"最新值"槽：

- `set()` 覆盖旧值并 `++seq_`
- `tryGet()` 返回 `std::optional<T>`（从没有人写过就返回 `nullopt`）
- `seq()` 给 `processLoop` 用来判断"图是不是新的"

`processLoop` 的循环（[autoaim_app.cpp:265](app/autoaim_app.cpp#L265)）其实就是：

```
看 latest_frame_.seq() 变了没 → 没变就 sleep 1ms → 变了就取出来处理
```

> 📌 **为什么这样设计**：抓图、处理、收发三个速度不一样。用"最新值"槽位，慢的一环会**丢帧**而不是**积压**，永远处理最新的那一帧。
> 📌 **所以 `txLoop` 里绝不能打印或写文件** —— 它是 100 Hz 定频的，一抖动就废。

### 启动顺序

`main` → `parseArgs` → 检查 `--hik` 有没有 SDK 支持 → `AutoAimApp::init()`（开相机 + 开串口）→ `threads::start()` 起四个线程 → 主线程每 5 秒打印一次统计，等 Ctrl-C。

统计里几个计数器的含义（排查时很有用）：

| 计数器 | 含义 | 一直涨说明什么 |
| --- | --- | --- |
| `frames_processed` | 处理了多少帧 | 就是 fps |
| `armors_found` | 检出装甲板的帧数 | 低 → 阈值/曝光不对 |
| `pnp_rejected` | 重投影超差被丢的帧数 | 高 → 内外参或灯条端点有问题 |
| `rx_resync` | 组帧错位重同步次数 | 一直涨 → 串口有干扰/波特率不对 |
| `stale_cmds` | 降级"保持"的次数 | 高 → 检测不稳 |
| `jump_rejected` | 跳变闸门拦下的次数 | 高 → 解算在抖，见第 ⑧ 站 |
| `fire_requests` | 请求开火次数 | — |

---

# 第一站 · 相机出图

**代码**：[hik_camera.cpp](bsp/camera/hik_camera.cpp)

### 打开流程（`HikCamera::open`）

| 步 | 调用 | 作用 |
| --- | --- | --- |
| 1 | `MV_CC_EnumDevices(MV_USB_DEVICE \| MV_GIGE_DEVICE, ...)` | 问系统装了哪些相机 |
| 2 | `MV_CC_CreateHandle` | 拿第一台设备的句柄 |
| 3 | `MV_CC_OpenDevice` | 打开设备 |
| 4 | `SetEnumValue("TriggerMode", 0)` B| **关触发，连续采集** |
| 5 | `SetEnumValue("ExposureAuto", 0 或 2)` B| 手动/自动曝光 |
| 6 | `SetFloatValue("ExposureTime", ...)`-- | 曝光微秒 |
| 7 | `SetFloatValue("Gain", ...)` --| 增益 |
| 8 | `SetEnumValue("PixelFormat", RGB8_Packed)` | 失败则退回 BGR8 软件转换 |
| 9 | `GetIntValueEx("PayloadSize"/"Width"/"Height")` | 拿分辨率和每帧字节数 |
| 10 | `MV_CC_StartGrabbing` | 开始取流 |

### 取帧（`HikCamera::grab`）

1. 缓冲区按 `payload_size_` 分配一次，之后复用
2. `MV_CC_GetOneFrameTimeout(handle, buf, payload_size, &info, wait_ms)` —— **阻塞等一帧**
3. 拿 `info.nWidth/nHeight` 组一个 `cv::Mat` **包**在缓冲区上（不拷贝）
4. 是 RGB8 原生 → `clone()` 拷一份；是 BGR8 → `cv::cvtColor(BGR2RGB)`
5. 打时间戳 `f.timestamp_ns = nowNs()` ← **注意是"拿到图"的时刻，不是"曝光"的时刻**

### 输出

```cpp
struct Frame {
  cv::Mat  image;         // RGB 8UC3 —— 注意是 RGB，不是 OpenCV 默认的 BGR
  uint64_t timestamp_ns;  // steady_clock 纳秒
};
```

> 📌 **全链路统一用 RGB**。`cv::imshow` 之前要 `cvtColor(RGB2BGR)`，否则红蓝颠倒（[autoaim_app.cpp:361](app/autoaim_app.cpp#L361) 就是这么干的）。
> 📌 `--hik` 而本次构建没有 SDK 支持时，`makeCamera()` 直接返回 `nullptr` 并打印提示；`main` 在更早的地方就拦下了（[main.cpp:51](app/main.cpp#L51)）。

### 本站在调的参数

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `--exposure` | 5000 μs | 曝光时间。**和阈值配对调** |
| `--gain` | 10.0 | 增益。放大噪声，尽量用曝光而不是增益 |
| `--auto-exposure` | 关 | 自动曝光。比赛时**建议关**，否则灯条亮度漂 |

---

# 第二站 · 二值化 + 找灯条

**代码**：[armor_detector.cpp:42-131](modules/detect/armor_detector.cpp#L42-L131)

### 2.1 二值化（`preprocessImage`）

```
RGB → cvtColor(RGB2GRAY) → threshold(binary_thres, 255, THRESH_BINARY)
```

**就两步，但这里是全链路最脆的一环。**

灰度公式下，纯红 `(255,0,0)` 的灰度只有 **76**。默认 `binary_thres = 160` → 纯红装甲板**完全抓不到**；
能抓到的是**过曝的 LED 灯条**（灰度接近 255）。

> 📌 所以 `--thres` 的真实含义是**"多亮才算灯条"**，它和**曝光 / 补光 / 环境光强**直接耦合。换场地必须重调。

### 2.2 找轮廓 → 拟合灯条（`findLights`）

对每个轮廓：

| 步 | 做法 |
| --- | --- |
| 1 | 轮廓点数 `< 5` 直接丢 |
| 2 | `boundingRect` 取正矩形；`minAreaRect` 取最小外接旋转矩形 |
| 3 | 把轮廓填进 mask、`findNonZero` 数像素 → 算**填充率** |
| 4 | `cv::fitLine(DIST_L2)` 拟合灯条**中轴线** |
| 5 | 用拟合直线与正矩形上下边的交点算出 **`top` / `bottom` 两个端点** |
| 6 | 算 `tilt_angle`（灯条相对竖直的倾角） |
| 7 | 构造 `Light` |
| 8 | 用 `isLight()` 和填充率**两道筛** |
| 9 | 在轮廓内累加 R 和 B 通道 → `sum_r > sum_b ? RED : BLUE` |

**筛子**（`isLight`）：

| 条件 | 参数 | 含义 |
| --- | --- | --- |
| `min_ratio < width/length < max_ratio` | 0.1 ~ 0.4 | 灯条是**细长**的 |
| `tilt_angle < max_angle` | 35° | 灯条接近竖直 |
| `fill_ratio > min_fill_ratio` | 0.8 | 轮廓要填满外接矩形（不是空心框） |

### 输出

```cpp
struct Light : public cv::Rect {   // 继承 cv::Rect，自带 x/y/width/height
  int         color;         // RED=0 / BLUE=1
  cv::Point2f top, bottom;   // ★ 灯条的上下两个端点 —— 后面所有几何的源头
  cv::Point2f center;
  double      length;        // = norm(top - bottom)
  double      width;         // = 像素数 / length   ← 注意是等效宽度，不是矩形的 width
  float       tilt_angle;
};
```

> ⚠️ **`Light::width` 和 `Light::Rect::width` 不是一回事**：前者是"面积÷长度"的等效宽度，后者是外接矩形宽。`isLight` 用的是前者。
> 📌 **`top` / `bottom` 是 PnP 的四个图像点的来源**。这一站端点拟合歪了，第 ④ 站的距离和角度全歪。

### 本站在调的参数

| 参数 | 默认 | 种类 |
| --- | --- | --- |
| `--thres`-- | 160 | **环境参数**，换场地必调 |
| `LightParams::min_ratio` / `max_ratio`B | 0.1 / 0.4 | 形状筛选 |
| `LightParams::max_angle`B | 35.0 | 倾角筛选 |
| `LightParams::min_fill_ratio` B| 0.8 | 填充率筛选 |
| `--color`（`detect_color`）B | 0（红） | 代码里固定 0，**命令行没有切换开关** |

---

# 第三站 · 配对成装甲板

**代码**：[armor_detector.cpp:133-198](modules/detect/armor_detector.cpp#L133-L198)

### 算法：`matchLights` —— O(n²) 两两配对

对每一对灯条 `(l1, l2)`：

| 筛子 | 条件 | 参数 |
| --- | --- | --- |
| 颜色 | 两根灯条颜色都必须等于 `detect_color` | — |
| 中间夹灯条 | `containLight`：两灯条的外接矩形内**不能**再有别的灯条 | — |
| 长度比 | 短的÷长的 `> min_light_ratio` | 0.7 |
| 中心距 | `norm(c1-c2) / 平均长度` 落在 `[0.8, 3.2)` 或 `[3.2, 5.5)` | 见下 |
| 水平度 | 两灯条中心连线与水平的夹角 `< max_angle` | 35° |

**关键：中心距是用「灯条长度」归一化的**，不是用像素：

```
center_distance = |c1 - c2| / ((len1 + len2) / 2)
```

| 归一化中心距 | 判定 |
| --- | --- |
| `[0.8, 3.2)` | `SMALL` |
| `[3.2, 5.5)` | `LARGE` |
| 其它 | `INVALID`（丢弃） |

> 📌 **这就是后面 PnP 物体点该用 small 还是 large 的依据**。现场只有一种板型，所以怎么判都不影响（`--armor-w` 会把两者设成同一个值）。

### 单灯条降级

如果**一块装甲板都没配出来**，但灯条数量 `<= max_single_light_count`（默认 3）且开了 `single_light_fallback`：

→ 每根灯条各自包成一个 `ArmorType::SINGLE` 的装甲板。

**SINGLE 的作用不是瞄准，是"我还看得见东西"** —— 它没有可靠的左右灯条，进不了 PnP。

### 选靶

`Detector::detect()` 返回 `vector<Armor>`，**app 层再挑一个**（[autoaim_app.cpp:138](app/autoaim_app.cpp#L138)）：

> 选**离画面中心最近**的那个 —— 用 `calculateDistanceToCenter`，即像素点到 `(cx, cy)` 的欧氏距离。

### 输出

```cpp
struct Armor {
  Light       left_light, right_light;  // 已按 x 排序，左小右大
  cv::Point2f center;                   // = 两灯条中心的中点
  ArmorType   type;                     // SMALL / LARGE / SINGLE / INVALID
  cv::Mat     number_img;               // 数字切片（分类器用）
  std::string number;                   // 识别出的数字
  float       confidence;
  std::string classfication_result;
};
```

> 📌 `number` 目前是空的 —— `number_classifier` 是个**空壳接口**（[number_classifier.h](modules/detect/number_classifier.h)），没有模型文件。跟踪器用 `number` 做同一性匹配，**现在所有装甲板的 `number` 都是空串**，等效于"只按位置匹配"。

---

# 第四站 · PnP 解算：二维像素 → 三维米

**代码**：[pnp_solver.cpp](modules/solver/pnp_solver.cpp)

### 输入：4 个图像点

按**固定顺序**取（顺序必须和物体点一一对应，[pnp_solver.cpp:48-51](modules/solver/pnp_solver.cpp#L48-L51)）：

| 序 | 图像点 |
| --- | --- |
| 0 | `left_light.bottom` |
| 1 | `left_light.top` |
| 2 | `right_light.top` |
| 3 | `right_light.bottom` |

### 物体点：由参数**算出来**的

构造时按 `ArmorGeometry` 生成（单位 mm→m）：

```cpp
half_y = width  / 2 / 1000      // 灯条中心距的一半
half_z = height / 2 / 1000      // 灯条长度的一半

P0 = (0, +half_y, -half_z)
P1 = (0, +half_y, +half_z)
P2 = (0, -half_y, +half_z)
P3 = (0, -half_y, -half_z)
```

**装甲板平面是 `x = 0` 的 Y-Z 平面**，法线朝 x 轴。

> 📌 **`width` 是"两根灯条中心之间的距离"，`height` 是"灯条的长度"** —— **不是**靶板的外观尺寸。
> 📌 这正是 `--armor-w` / `--armor-h` 必须实测反标的原因：填 140 而不是 132，5 m 处距离直接错 **+5.15%**。

### 解算

```cpp
cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_,
             rvec, tvec, /*useExtrinsicGuess=*/false, cv::SOLVEPNP_ITERATIVE);
```

| 输出 | 类型 | 含义 |
| --- | --- | --- |
| `rvec` | `cv::Mat` 3×1 CV_64F | 旋转向量（Rodrigues 形式） |
| `tvec` | `cv::Mat` 3×1 CV_64F | **相机系**下装甲板中心的平移量（米） |

### 把关：重投影误差

`reprojectionError()` 把物体点用解出的 `rvec/tvec` 再投影回图像，和原始 4 个点比 RMS：

```
err = sqrt( Σ|reproj_i - image_i|² / 4 )
```

`err > max_reprojection_error_px`（默认 **3.0 px**）→ **丢掉这帧**，`pnp_rejected++`，保持上次角度。

> 📌 这道闸门的实际作用：灯条端点拟合歪了、或者配错对了，解出来的位姿会"看着像但重投影对不上"，这一步把它拦下来。

### 本站在调的参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--fx --fy --cx --cy` --| 1000 / 1000 / 720 / 540 | **内参，全是占位符，必须标定** |
| `--armor-w --armor-h` --| 132 / 57 mm | **必须实测反标** |
| `dist_coeffs` | 全 0 | 畸变系数，代码里没暴露命令行开关 |
| `max_reprojection_error_px` | 3.0 px | 重投影闸门 |

---

# 第五站 · 相机系 → 云台系

**代码**：[coord_transform.cpp](modules/solver/coord_transform.cpp)

### 为什么需要这一站

相机**装在云台上**，云台会转。第 ④ 站解出的位置是"**相对相机**"的，而我们要发给下位机的是"**相对云台**"的角度。云台一转，相机跟着转，不换算的话瞄准立刻废。

### 算法

```
p_云台 = R_mount(yaw, pitch, roll) × R₀ × p_相机 + cam_origin
```

其中 `R₀` 是**固定的基变换**（写入代码的常量）：

```
R₀ = [ 0  0  1 ]
     [-1  0  0 ]
     [ 0 -1  0 ]
```

`R_mount` 由三个可调角构成：

```
R_mount = Rz(cam_yaw) × Ry(-cam_pitch) × Rx(cam_roll)
```

> ⚠️ 注意 pitch 前面有个**负号** —— 这是约定的一部分，**符号必须实测确认**。上游仓库从没算过 pitch，没有可继承的约定。
> 📌 还有平移 `cam_origin = (cam_x, cam_y, cam_z)`，默认全 0（认为相机光心和云台中心重合）。

### 姿态也要转

```cpp
cv::Rodrigues(rvec, R_cam);           // 旋转向量 → 旋转矩阵
R_gimbal = R_cam2gimbal(cfg) * R_cam; // 转到云台系
orientation = Eigen::Quaterniond(R_gimbal).normalized();
```

### 取 yaw 的正确写法

```cpp
double yawFromQuat(const Eigen::Quaterniond & q) {
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));   // ← 必须这么写
}
```

> ⚠️ **不要用 `Eigen::eulerAngles(2,1,0)(0)`** —— 它在 |yaw| > π 时会跳到另一组合法分支，返回 `yaw - π`。
> 漏掉的现象是"**靶板转到背面就丢目标**"。回归测试在 `tests/test_coord.cpp` 用例 7b。

### 输出

| 输出 | 类型 | 含义 |
| --- | --- | --- |
| `pos_gimbal` | `Eigen::Vector3d` | 云台系下装甲板中心位置（**米**） |
| `orientation` | `Eigen::Quaterniond` | 云台系下装甲板朝向 |

### 本站在调的参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--cam-pitch` --| **-15（占位符）** | **符号必须实测** |
| `--cam-yaw` --`--cam-roll` | 0 | 同上 |
| `cam_x/y/z` | 0 | 没暴露命令行开关 |

---

# 第六站 · EKF 跟踪

**代码**：[tracker.cpp](modules/tracker/tracker.cpp) + [extended_kalman_filter.cpp](modules/tracker/extended_kalman_filter.cpp)

### 这一站是分水岭

- **输入**：`ArmorMeasurement`（"这一帧它在哪、朝哪"）
- **输出**：9 维状态向量（"**它在怎么动、往哪动、多快**"）

没有它就没有提前量。

### 输入

```cpp
struct ArmorMeasurement {
  Eigen::Vector3d     position;                  // 云台系位置（米）
  Eigen::Quaterniond  orientation;               // 云台系朝向
  double              distance_to_image_center;  // 像素
  std::string         number;                    // 目前恒为空串
  std::string         type;
};
```

### 状态向量（9 维）

```
x = [ xc, vx, yc, vy, za, va, yaw, v_yaw, r ]
      0   1   2   3   4   5   6     7    8
```

| 下标 | 含义 |
| --- | --- |
| 0, 1 | 靶心 x 坐标、x 方向速度 |
| 2, 3 | 靶心 y 坐标、y 方向速度 |
| 4, 5 | 装甲板 z 坐标（高度）、z 方向速度 |
| 6, 7 | 装甲板当前 yaw、yaw 角速度 |
| 8 | **r —— 装甲板绕靶心转的半径** |

**"装甲板位置"是由状态推出来的**（[target_state.h:9](modules/tracker/target_state.h#L9)）：

```cpp
armorPositionFromState(x) = { xc - r*cos(yaw),  yc - r*sin(yaw),  za }
```

> 📌 这个式子就是"**靶心在 (xc,yc)，装甲板挂在半径 r 的圆周上、当前转到 yaw 角**"的数学表达。
> 转盘模式下 `r` 应该收敛到 0.15~0.5；自旋模式下 `r ≡ 0`（装甲板原地转头）。

### 过程模型 `f`

**匀速模型** —— 四个量各自线性外推：

```
x_new(0) += x(1) * dt      // 靶心 x
x_new(2) += x(3) * dt      // 靶心 y
x_new(4) += x(5) * dt      // 高度
x_new(6) += x(7) * dt      // yaw
```

### 观测模型 `h`

```
z = [ xc - r*cos(yaw),  yc - r*sin(yaw),  za,  yaw ]     (4 维)
```

四个观测量里，前三个是**装甲板位置**（不是靶心位置！），第四个是 **yaw**。

> 📌 这是整个 EKF 最精妙的地方：**观测的是装甲板，估计的是靶心**。中间靠 `r` 和 `yaw` 联系，所以靶心位置和旋转半径是"从装甲板轨迹里反推出来的"。

### 状态机（5 态）

```
       init()                matched ×>5            !matched
LOST ─────────► DETECTING ────────────────► TRACKING ──────────► TEMP_LOST
                    ▲                          ▲   ▲               │
                    │                          │   └───────────────┘
                    │                          │      matched（重捕）
                    │                          │
                    └── matched 丢了就回 LOST   └── lost_count > lost_thres → LOST
```

| 状态 | 含义 |
| --- | --- |
| `LOST` | 没目标 |
| `DETECTING` | 刚初始化，**连续匹配 `tracking_thres`(5) 帧才转 TRACKING** |
| `TRACKING` | 正常跟踪 |
| `TEMP_LOST` | 暂时丢了，**EKF 继续盲推**，超过 `lost_time_thres`(0.3 s) 才真丢 |
| `CHANGE_TARGET` | 换目标（见下方注意） |

`tracking()` 返回 `TRACKING || TEMP_LOST` —— **TEMP_LOST 也算"在跟踪"**，这样短暂丢帧时还能出解。

> 📌 `lost_thres = lost_time_thres / dt`，即"按帧数折算的 0.3 秒"。

### 匹配与跳变

每一帧 `update()`：

1. `ekf.predict()` 先按匀速模型推一步
2. 在所有候选里找 `number == tracked_id` 的（**现在全是空串，等效于找任意一个**）
3. 算**位置差**和 **yaw 差**：
   - 位置差 `< max_match_distance`(0.15 m) **且** yaw 差 `< max_match_yaw_diff`(1.0 rad) → **匹配成功，`ekf.update()`**
   - 只有一个候选、且 yaw 差超限 → **`handleArmorJump()`**（见下）
4. 半径约束 + AUTO 模式判定
5. 推进状态机

**`handleArmorJump`** —— 转盘模式下，前一块装甲板转过侧面、后一块转进视野时，`yaw` 会突然跳变。处理：

- 直接把状态里的 `yaw` 改成新观测值
- 如果推出位置和实测位置差太多（> `max_match_distance`），**把靶心位置也一起重算**
- `setStateInflated(state, jump_p_inflate=20)` —— **把协方差矩阵放大约 20 倍**，告诉滤波器"我不太确定了，接下来几帧多信观测"

> 📌 这个"协方差膨胀"是处理跳变的标准手法：不重新初始化（会丢速度信息），只是降低对先验的信心。

### 半径约束 `applyRadiusConstraint`

这是**把先验知识塞进滤波器**的地方：

| 模式 | 做法 |
| --- | --- |
| `SELF_SPIN` | `r *= 0.95`（每帧缩 5%，**慢慢把 r 逼到 0**） |
| `CAROUSEL` | `r` 软拉到 `[r_min=0.12, r_max=0.40]`：`r += 0.05 * (lim - r)` |

> 📌 **物理意义**：装甲板不可能在半径 3 米的圆上转，也不可能贴在靶心（转盘模式）。用这个约束把明显不合理的 `r` 慢慢拽回来。

### AUTO 模式怎么自动判

```
开局默认按 CAROUSEL 跑
先等 auto_grace_frames(120) 帧热身
之后：如果 r 连续 auto_switch_frames(60) 帧都 < r_min × 1.5 (= 0.18)
      → 判定为 SELF_SPIN，把 r 清 0
```

> ⚠️ **单向切换**：一旦切到 SELF_SPIN 就 `return`，不会再切回来。判定错了只能重启程序。
> 📌 想稳就**别用 auto**，现场看清是哪种直接 `--rotation carousel` 或 `--rotation self_spin`。

### 输出

| 输出 | 类型 |
| --- | --- |
| `targetState()` | `Eigen::VectorXd`，9 维 |
| `radius()` | `double`，即 `x(8)` |
| `vYaw()` | `double`，即 `x(7)` |
| `activeMode()` | 当前实际生效的模式 |

### ⚠️ 关于 `CHANGE_TARGET`

代码里 `tracker_state == CHANGE_TARGET` 只被**读取**（[tracker.cpp:261](modules/tracker/tracker.cpp#L261)、[322](modules/tracker/tracker.cpp#L322)），**没有任何地方给它赋值**。所以：

- `diff_count` 恒为 0 → `initChange()` **永远不会被调用**
- `CHANGE_TARGET` 是一条**走不到的分支**（换目标功能实际未生效）

不影响当前单一靶板的比赛场景，但要知道它在这儿。

### 本站在调的参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--rotation` | `auto` | `carousel` / `self_spin` / `auto` |
| `max_match_distance` | 0.15 m | 位置匹配门限 |
| `max_match_yaw_diff` | 1.0 rad | yaw 匹配门限（超了算跳变） |
| `tracking_thres` | 5 | 连续匹配几帧算"跟稳了" |
| `lost_time_thres` | 0.3 s | 盲推多久才算真丢 |
| `auto_switch_frames` / `auto_grace_frames` | 60 / 120 | AUTO 模式判定 |
| `r_min` / `r_max` / `r_pull` / `r_shrink` | 0.12 / 0.40 / 0.05 / 0.95 | 半径约束强度 |
| `s2qxyz_max/min`, `s2qyaw_max/min`, `s2qr` | 见配置 | **过程噪声**。调大=更信观测（跟得紧但更抖） |
| `r_xyz_factor`, `r_yaw` | 0.05 / 0.02 | **测量噪声**。调大=更信模型（更稳但更迟） |
| `jump_p_inflate` | 20.0 | 跳变时协方差放大幅度 |

> 📌 **噪声参数的调法**：`Q` 大 / `R` 小 → 跟得紧、抖；`Q` 小 / `R` 大 → 平滑、滞后。现场先用默认值，只在"明显抖"或"明显跟不上"时才动。

---

# 第七站 · 瞄准解算：提前量 + 抬枪口

**代码**：[aim_solver.cpp](modules/aim/aim_solver.cpp)

### 两条入口

| 入口 | 什么时候用 | 提前量 |
| --- | --- | --- |
| `solveFromState(targetState, v)` | 跟踪器 `tracking()` 为真 | **有**（用状态里的速度外推） |
| `solveFromMeasurement(pos_gimbal, v)` | 跟丢了 / 没开跟踪器 | **无**（`t_lead = 0`，直接瞄当前位置） |

> 📌 注意：**没开 `--enable-tracker` 时，弹速是写死的 22.0**（[autoaim_app.cpp:204](app/autoaim_app.cpp#L204)、[217](app/autoaim_app.cpp#L217)），压根不读反馈。

### 7.1 提前量（`solveFromState`）

```cpp
p_now    = armorPositionFromState(s);         // 当前装甲板位置
t_flight = p_now.norm() / v;                  // 飞行时间
t_lead   = min(t_flight + latency_s + gimbal_lag_s, max_lead_s);   // ← 夹取

// 按速度外推 t_lead 之后的状态
sp(0) += s(1) * t_lead;
sp(2) += s(3) * t_lead;
sp(4) += s(5) * t_lead;
sp(6) += clamp(s(7), ±max_v_yaw) * t_lead;    // ← yaw 角速度也夹取

p_pred = armorPositionFromState(sp);          // 预测点
```

**三段时间加起来**：

| 分量 | 参数 | 默认 | 含义 |
| --- | --- | --- | --- |
| 飞行时间 | — | 算出 | 子弹飞过去要多久 |
| 处理延迟 | `latency_s` | 0.05 s | 从曝光到发出指令 |
| 云台滞后 | `gimbal_lag_s` | 0.03 s | 下位机收到到云台转到位 |

**两道夹取**：

| 夹取 | 参数 | 默认 | 为什么 |
| --- | --- | --- | --- |
| `t_lead` 上限 | `max_lead_s` | 0.6 s | 防止弹速异常时外推飞出天际 |
| `v_yaw` 上下限 | `max_v_yaw` | 10 rad/s | 转盘重捕时 EKF 速度会炸（实测冲到 55） |

### 7.2 弹道（`ballisticPitch`）

解这个二次方程（`tanθ` 为未知数）：

```cpp
k    = g * d² / (2v²)
disc = d² - 4k(k + h)
if (disc < 0) return atan2(h, d);          // ★ 静默退回直瞄
u    = (d - sqrt(disc)) / (2k);
pitch = atan(u);
```

| 符号 | 含义 |
| --- | --- |
| `d` | 水平距离 |
| `h` | 高度差 |
| `v` | 弹速 |
| `g` | 重力（`gravity`，默认 9.8） |

### 7.3 组装 `AimResult`

```cpp
AimResult {
  double yaw_rad;          // = atan2(p.y, p.x)
  double pitch_rad;        // = 弹道角 + pitch_offset_deg
  double t_flight_s;       // = flight_coeff * dist / v
  double t_lead_s;
  Eigen::Vector3d aim_point;   // 最终瞄的那个点
  bool valid;
};
```

**有效性检查**：`p` 必须全有限，且 `min_aim_dist(0.5) <= |p| <= max_aim_dist(30)`。

### ⚠️ 本站是"弹速=0"缺陷的爆点

下位机反馈的 `bullet_speed` 实际是 **0**，而 `v = max(bullet_speed, min_bullet_speed=1.0)` → **`v` 变成 1.0 m/s**。两个后果，**都不报错**：

| 后果 | 机制 |
| --- | --- |
| **提前量几乎翻倍** | `t_flight` 从正确的 0.307 s（22 m/s @ 5m）变成 5 s，被 `max_lead_s = 0.6` 截断 |
| **弹道补偿静默失效** | `k = g·d²/(2v²)` 变得极大 → `disc < 0` → 直接 `return atan2(h,d)`，5 m 处**少补约 25 cm 下坠** |

> 打比方：**体重秤坏了显示 0，程序却当真按"你重 0 公斤"去算。**
> 详见 [FIELD_CHECKLIST.md](readme/FIELD_CHECKLIST.md) 第三部分。

### 本站在调的参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `latency_s` | 0.05 s | 处理延迟补偿 |
| `gimbal_lag_s` | 0.03 s | 云台滞后补偿 |
| `flight_coeff` | 1.0 | 飞行时间缩放（标定用） |
| `enable_ballistic` | true | 关掉就纯直瞄 |
| `gravity` | 9.8 | |
| `min_bullet_speed` | **1.0** | ★ 弹速=0 缺陷的来源 |
| `max_v_yaw` | 10.0 rad/s | ★ 转盘模式主要调这个 |
| `max_lead_s` | 0.6 s | ★ |
| `min_aim_dist` / `max_aim_dist` | 0.5 / 30 m | 有效距离窗口 |
| `pitch_offset_deg` | 0.0 | 枪口机械偏置 |
| `max_jump_deg` | 25.0 | **下一站用** |

---

# 第八站 · 跳变闸门

**代码**：[autoaim_app.cpp:225-238](app/autoaim_app.cpp#L225-L238)

```cpp
dyaw   = |remainder(aim.yaw   - last_yaw  , 2π)| * R2D;
dpitch = |aim.pitch           - last_pitch|       * R2D;

if (dyaw > max_jump_deg || dpitch > max_jump_deg) {
    jump_rejected_++;
    publishHoldCommand();      // ★ 保持上次角度，不发新的
    return;
}
```

| | |
| --- | --- |
| **进** | 本帧算出的 `AimResult` |
| **算法** | 和上一帧比，超阈值就判为解算异常 |
| **出** | 要么放行，要么降级"保持" |
| **参数** | `max_jump_deg`（25°）；yaw 用 `remainder` 处理跨 ±π |

> 📌 **这一站存在的唯一理由**：转盘模式下装甲板侧对相机时约 29% 的帧检测中断，EKF 盲推后重捕，速度估计会炸（实测 `v_yaw` 冲到 55 rad/s，真值 2.0），解出的角度会瞬间跳几十度。
> 📌 **用错旋转模式时，这个计数器会疯涨** —— `jump_rejected` 是现场判断"模式选错没有"的最好指标。

**降级行为**（`publishHoldCommand`）：保持**上一次的 yaw / pitch**、`fire_flag = 0`、`fire_.reset()`。

> ⚠️ 注意是"保持"而不是"清零回原点" —— 否则云台会猛甩，很容易打坏机械结构。

---

# 第九站 · 开火决策 + 打包发送

**代码**：[fire_decision.cpp](modules/aim/fire_decision.cpp) + [srm_protocol.cpp](modules/protocol/srm_protocol.cpp)

### 9.1 开火决策

**只有加了 `--enable-fire` 才会走到这儿**（[autoaim_app.cpp:241](app/autoaim_app.cpp#L241)）。

**四道闸，全过才返回 1**：

| 序 | 条件 | 参数 | 默认 |
| --- | --- | --- | --- |
| 1 | 反馈不过期 | `max_data_age_s` | 0.2 s |
| 2 | `aim.valid` | — | — |
| 3 | 还没打满 | `max_shots` | 50 |
| 4 | 云台实际角 **对准**目标角 | `yaw_tol_deg` / `pitch_tol_deg` | 1.5° / 1.5° |

对准之后还要**持续**：

| 条件 | 参数 | 默认 |
| --- | --- | --- |
| 保持对准多久 | `min_converge_s` | 0.10 s |
| 距上次开火多久 | `min_interval_s` | 0.9 s |
| 开火后连续几帧置 1 | `fire_hold_s` | 0.15 s |

> 📌 **闸门 1（反馈过期）是安全设计的关键**：串口断线时反馈会**冻在最后一帧**，此时绝不开火。
> 📌 **闸门 4 用的是下位机反馈的真实云台角度**，不是我们算的目标角度 —— 这是"闭环"，确保枪口真的转到位了。

### 9.2 打包：18 字节小端

`pack_target()` 的字节布局（[srm_protocol.cpp:57](modules/protocol/srm_protocol.cpp#L57)）：

| 偏移 | 长度 | 类型 | 值 |
| --- | --- | --- | --- |
| 0 | 2 | int16 | `body_len = 16` |
| 2 | 2 | int16 | `ID = 1`（云台） |
| 4 | 4 | float32 | `yaw_deg` |
| 8 | 4 | float32 | `pitch_deg` |
| 12 | 2 | int16 | `ID = 2`（射击） |
| 14 | 4 | int32 | `fire_flag` |

**全部小端**，逐字节手工组装（`put_i16/put_i32/put_f32`），不依赖主机字节序。

> ⚠️ **必须发完整的 ID 1 + ID 2** —— 省略记录时下位机不会清零旧字段。

### 9.3 接收：30 字节小端

| 偏移 | 长度 | 类型 | 字段 |
| --- | --- | --- | --- |
| 0 | 2 | int16 | `body_len = 28` |
| 2 | 2 | int16 | `ID = 1` |
| 4 | 4 | float32 | `yaw_deg` |
| 8 | 4 | float32 | `pitch_deg` |
| 12 | 4 | float32 | `roll_deg` |
| 16 | 4 | int32 | `mode` |
| 20 | 4 | int32 | `color`（机器人 ID） |
| 24 | 2 | int16 | `ID = 2` |
| 26 | 4 | float32 | `bullet_speed` |

**流式组帧**（`FrameParser::feed`）：

```
攒够 30 字节 → 看 body_len 是不是 28
               是 → 解析一帧，从中删掉 30 字节
               不是 → 从缓存头删掉 1 字节，重同步计数 +1，再看
```

> 📌 **USB 包边界 ≠ 协议帧边界**：一次 `read` 可能是半帧 / 整帧 / 多帧。协议**无魔数、无 CRC**，长度不对只能"删 1 字节再来"地重新同步。
> 📌 缓存超过 4096 字节直接清空重来。

### 9.4 发送线程

```cpp
period = 1000000 / tx_hz          // 100 Hz → 10 ms
每次：取 latest_cmd_ → pack_target → write_bytes → sleep_until(next)
```

**定频而不是"有结果就发"** —— 下位机期望稳定的 100 Hz 流。

> ⚠️ **启动时 `latest_cmd_` 还是空的**，`txLoop` 会发 `yaw=0, pitch=0, fire=0` 的默认帧。也就是说程序一起来就可能命令云台回 0 度 —— 上电前确认云台周围是安全的。

---

# 参数总表

按"在哪一站生效"排列。**默认值加粗表示是占位符，必须实测。**

> 📖 这一节说"**是什么**"。每个参数**具体怎么弄到那个数、怎么知道自己弄对了**，见文末 [参数怎么定 —— 逐个参数的具体取法](#参数怎么定--逐个参数的具体取法)。

**"什么时候调"一栏只有两种值**，区别是**改完要不要重新编译**：

| 值 | 含义 | 调它的动作 |
| --- | --- | --- |
| **编译后·启动时** | 有命令行开关（[config.cpp](app/config.cpp) 里 `parseArgs` 认它） | 改启动命令 → 重启程序 → 立刻生效。**不用重编** |
| **编译前·改代码** | 写死在源码默认值里（[config.h](app/config.h) / [tracker_config.h](modules/tracker/tracker_config.h) / [aim_solver.h](modules/aim/aim_solver.h) / [fire_decision.h](modules/aim/fire_decision.h)） | 改源码 → **重新编译** → 再启动才生效 |

> 📌 一句话：**带 `--` 的是"编译后"，不带 `--` 的是"编译前"。**
> 📌 检测类参数（`--thres` 那一族）配合 `--replay` + `--debug-view` 就能**离线反复试**，不用连云台、不用开火，也不必重编 —— 现场只是把试好的值抄进命令。

| 参数 | 默认 | 生效站 | 类型 | 什么时候调 |
| --- | --- | --- | --- | --- |
| `--exposure` | 5000 μs | ① 相机 | `float` | 编译后·启动时 |
| `--gain` | 10.0 | ① 相机 | `float` | 编译后·启动时 |
| `auto_exposure` | 关 | ① 相机 | `bool` | 编译前·改代码（**没有命令行开关**） |
| `--thres` | 160 | ② 找灯条 | `int` | 编译后·启动时 |
| `LightParams.min_ratio` / `max_ratio` | 0.1 / 0.4 | ② | `double` | 编译前·改代码 |
| `LightParams.max_angle` | 35.0 | ② | `double` | 编译前·改代码 |
| `LightParams.min_fill_ratio` | 0.8 | ② | `double` | 编译前·改代码 |
| `detect_color` | 0（红） | ③ 配对 | `int` | 编译前·改代码（**没有命令行开关**） |
| `ArmorParams.min_light_ratio` | 0.7 | ③ | `double` | 编译前·改代码 |
| `ArmorParams.*_center_distance` | 0.8 / 3.2 / 3.2 / 5.5 | ③ | `double` | 编译前·改代码 |
| `ArmorParams.max_angle` | 35.0 | ③ | `double` | 编译前·改代码 |
| `single_light_fallback` | 开 | ③ | `bool` | 编译前·改代码 |
| `max_single_light_count` | 3 | ③ | `int` | 编译前·改代码 |
| **`--fx` `--fy` `--cx` `--cy`** | **1000/1000/720/540** | ④ PnP | `double` | 编译后·启动时（值靠标定得出） |
| **`--armor-w`** | **132 mm** | ④ PnP | `float` | 编译后·启动时（值靠实测得出） |
| **`--armor-h`** | **57 mm** | ④ PnP | `float` | 编译后·启动时（值靠实测得出） |
| `max_reprojection_error_px` | 3.0 px | ④ | `double` | 编译前·改代码 |
| **`--cam-pitch`** | **-15°** | ⑤ 坐标变换 | `double` | 编译后·启动时（值靠实测得出） |
| `--cam-yaw` / `--cam-roll` | 0° / 0° | ⑤ | `double` | 编译后·启动时 |
| `--enable-tracker` | 关 | ⑥ 跟踪 | `bool` | 编译后·启动时 |
| `--rotation` | `auto` | ⑥ | 枚举 | 编译后·启动时 |
| `max_match_distance` | 0.15 m | ⑥ | `double` | 编译前·改代码 |
| `max_match_yaw_diff` | 1.0 rad | ⑥ | `double` | 编译前·改代码 |
| `tracking_thres` / `lost_time_thres` | 5 / 0.3 s | ⑥ | `int` / `double` | 编译前·改代码 |
| `r_min` / `r_max` / `r_pull` / `r_shrink` | 0.12 / 0.40 / 0.05 / 0.95 | ⑥ | `double` | 编译前·改代码 |
| `s2qxyz_max/min` | 0.1 / 0.05 | ⑥ | `double` | 编译前·改代码 |
| `s2qyaw_max/min` | 10.0 / 5.0 | ⑥ | `double` | 编译前·改代码 |
| `s2qr` | 80.0 | ⑥ | `double` | 编译前·改代码 |
| `r_xyz_factor` / `r_yaw` | 0.05 / 0.02 | ⑥ | `double` | 编译前·改代码 |
| `jump_p_inflate` | 20.0 | ⑥ | `double` | 编译前·改代码 |
| `latency_s` / `gimbal_lag_s` | 0.05 / 0.03 s | ⑦ 瞄准 | `double` | 编译前·改代码 |
| `enable_ballistic` / `gravity` | 开 / 9.8 | ⑦ | `bool`/`double` | 编译前·改代码 |
| **`min_bullet_speed`** | **1.0 m/s** | ⑦ | `double` | 编译前·改代码 |
| **`max_v_yaw`** | **10.0 rad/s** | ⑦ | `double` | 编译前·改代码 |
| **`max_lead_s`** | **0.6 s** | ⑦ | `double` | 编译前·改代码 |
| `min_aim_dist` / `max_aim_dist` | 0.5 / 30 m | ⑦ | `double` | 编译前·改代码 |
| `pitch_offset_deg` | 0.0 | ⑦ | `double` | 编译前·改代码 |
| **`max_jump_deg`** | **25.0°** | ⑧ 闸门 | `double` | 编译前·改代码 |
| `--enable-fire` | **关** | ⑨ 开火 | `bool` | 编译后·启动时 |
| `yaw_tol_deg` / `pitch_tol_deg` | 1.5° / 1.5° | ⑨ | `double` | 编译前·改代码 |
| `min_converge_s` / `fire_hold_s` | 0.10 / 0.15 s | ⑨ | `double` | 编译前·改代码 |
| **`min_interval_s`** | **0.9 s** | ⑨ | `double` | 编译前·改代码 |
| `max_shots` | 50 | ⑨ | `int` | 编译前·改代码 |
| `max_data_age_s` | 0.2 s | ⑨ | `double` | 编译前·改代码 |
| `require_tracking` | 关 | ⑨ | `bool` | 编译前·改代码（由 `--enable-tracker` 自动置位，不能直接传） |

## 参数分三类，别混着调

| 类别 | 有哪些 | 特征 | 怎么定 |
| --- | --- | --- | --- |
| **物理参数** | 内参、外参、`--armor-w/h` | 填错 = **系统性偏差**，所有帧一起错 | **必须实测**，不能靠调 |
| **环境参数** | `--thres`、曝光、增益 | 换场地立刻失效 | 每次现场**重调** |
| **算法调参** | 噪声、夹取、`--rotation` | 影响跟得紧不紧、稳不稳 | 调试期慢慢拧 |

> 📌 **这三类和"什么时候调"是对应的**：**物理参数 / 环境参数**几乎都开好了命令行开关（**编译后·启动时**，换个值重启就行）；**算法调参**绝大多数写死在源码里（**编译前·改代码**，拧一次要重编一次）。这也正是为什么"现场能救的"只有曝光、阈值、内参、板型、安装角、`--rotation` 这几个 —— 剩下的必须**提前在实验室调好**。

---

# 参数怎么定 —— 逐个参数的具体取法

> 上面那张总表说"这个参数**是什么**"，这一节说"**怎么弄到那个数，以及怎么知道自己弄对了**"。
> 一句话原则：**先分清这个参数是"测"出来的还是"试"出来的，两者不能混。**

---

## 0. 测 vs 试 —— 最重要的一个区分

| | **有唯一正确答案** | **没有正确答案** |
| --- | --- | --- |
| 打比方 | 配眼镜的度数 | 电视的亮度和音量 |
| 特征 | 错了就是**系统性偏差**：所有帧一起错，错得一致，**不报任何警** | 只影响"跟得稳不稳""反应快不快" |
| 只能怎么弄 | **测**（拿尺子量 / 拍照片标定） | **试**（看现象、做取舍） |
| 想靠"调"蒙过去 | 永远蒙不对，只是**错得不明显** | 正常，本来就是要试 |
| 有哪些 | 内参、外参、`--armor-w/h` | 阈值、曝光、噪声、所有夹取值 |

> ⚠️ **最容易犯的错，就是把"测"的参数当成"调"的参数。**
> 内参填 1000（默认占位符）程序照样跑、照样跟踪、照样开火 —— 只是所有距离都错，而你在画面上**一点都看不出来**。
> 打比方：**体温计坏了，你量十次都是 36.5，量得再认真也没用。**

---

## 1. 四种拿值的办法

| 办法 | 打比方 | 管哪些 | 工具 |
| --- | --- | --- | --- |
| **拿尺子量** | 量身高 | `--armor-w`、`--armor-h`、`r_min/r_max` | 卷尺 |
| **拍照片反解** | 验光配镜 | 内参、外参、`--armor-w/h`（复核） | `calib_intrinsic.py`、`calib_extrinsic.py` |
| **看着画面拧** | 调电视亮度 | `--thres`、`--exposure`、`--gain` | `tool_detect_view`、`--debug-view` |
| **看现象定** | 试鞋码 | `--rotation`、`max_v_yaw`、噪声参数 | `--debug-dump` |

### 现成的工具（都在 [tools/](tools/)）

```bash
# 1) 离线看检测效果（有 binary_thres / color 两个滑块，可以拖着试）
./build/tools/tool_detect_view <图像目录>

# 2) 标内参：棋盘格照片 → --fx --fy --cx --cy
python3 tools/calib_intrinsic.py --dir calib_imgs --cols 9 --rows 6 --square-mm 25

# 3) 录一段"云台扫几个角度"的片子（标外参用，需要串口）
./build/tools/tool_hik_record --out bags/calib --secs 60 \
    --device /dev/serial/by-id/usb-XXX --every 5

# 4) 标外参：图像 + 云台角度 → --cam-yaw --cam-pitch --cam-roll
python3 tools/calib_extrinsic.py --images bags/calib --angles bags/calib/angles.csv \
    --fx <内参> --fy <内参> --cx <内参> --cy <内参>

# 5) 造一份"真值已知"的合成数据（没车没相机时先跑通流程）
python3 tools/make_test_bag.py bags/syn --frames 200 --mode carousel
```

> 📌 `calib_intrinsic.py` / `calib_extrinsic.py` 的输出**直接就是能粘贴到命令行的参数**，不用自己换算。
> 📌 `tool_detect_view` 只吃**图像目录或单张图**（不是视频、不是 bag）。

---

## 2. 顺序不能跳：上游错了，下游全废

```
①曝光/相机 → ②阈值找灯条 → ③配对 → ④内外参+板型 → ⑤坐标变换 → ⑥跟踪 → ⑦瞄准 → ⑧闸门 → ⑨开火
└────────── 先让这段"准" ──────────┘   └────── 再让这段"稳" ──────┘
```

**只有 ①~⑤ 准了，才有资格谈 ⑥~⑨。**
前面没准的时候去拧 EKF 的噪声，等于给一副度数不对的眼镜调鼻托 —— 手感有变化，但你还是看不清。

> 📌 判据：如果 ④ 的 **5 米验收**（见下）还没过，**不要动 ⑥⑦⑧⑨ 的任何参数**。

---

## 3. 逐站详解

### ① 相机：`--exposure` / `--gain` / `auto_exposure`

**这一站的目标只有一个：让灯条过曝，让别的东西不过曝。**

背景：灰度公式下纯红 `(255,0,0)` 的灰度只有 **76**。所以真正能被 `--thres` 抓到的，不是"红色的装甲板"，而是**过曝到接近 255 的 LED 灯条** —— 曝光决定的就是"灯条有多白"。

| 参数 | 怎么定 |
| --- | --- |
| `--exposure` | **从低往高加，加到灯条刚好全白**（画面上是纯白实心条）为止。再高就开始糊 |
| `--gain` | 曝光不够时才加。**能不加就不加** —— 增益放大噪声，轮廓边缘会变毛，`isLight` 的填充率判据会掉 |
| `auto_exposure` | **编译前改**（[config.h:28](app/config.h#L28)；`tool_hik_record` 里也硬编码 false）。比赛**必须关**：自动曝光会让亮度随画面内容漂，阈值随之失效 |

**怎么做**：`--replay` 放一段现场图 + `--debug-view`，改 `--exposure` 重启，看灯条白不白。
**验收**：`armors_found` 在涨，画面里没有大片糊掉的白。

> ⚠️ 曝光太长的代价是**运动模糊**。转盘靶在动，长曝光会把灯条拖成一条糊 —— 第 ② 站第 4 步的端点拟合直接废。
> **宁可短曝光 + 补光，不要长曝光。**

### ② 二值化 + 灯条筛选

| 参数 | 默认 | 怎么定 |
| --- | --- | --- |
| `--thres` | 160 | ★ **唯一"现场必调"的参数**。用 `tool_detect_view` 拖滑块 |
| `LightParams.min_ratio` / `max_ratio` | 0.1 / 0.4 | 灯条的宽长比。**没有工具**，改 [armor_detector.h:19](modules/detect/armor_detector.h#L19) 重编再看 |
| `LightParams.max_angle` | 35.0 | 灯条相对竖直的倾角上限 |
| `LightParams.min_fill_ratio` | 0.8 | 轮廓填满外接矩形的比例 |

**`--thres` 的具体调法**（`tool_detect_view <图像目录>`，拖 `binary_thres` 滑块）：

- **往低拖**：背景开始变白，冒出大量假轮廓 → **太低**
- **往高拖**：灯条断开 / 整根消失 → **太高**
- **取法**：找到"灯条完整成条 且 背景全黑"这一段区间，**取靠上（偏严）那一端** —— 离背景噪声远一点
- **验收**：`armors_found` 涨、画面里假框少

> 📌 这个值**和曝光是一对**：曝光加了，阈值就该跟着往上提。所以**先定曝光，再定阈值**；反过来会来回改。

**灯条筛选取值**：这三个是"**长什么样才算灯条**"的模板。判据是**"被丢掉的"和"被留下的"分别长什么样**：

| 参数 | 含义 | 取舍 |
| --- | --- | --- |
| `ratio` 0.1~0.4 | 灯条是细长的 | **转盘靶侧对时灯条被压扁**（宽度变小），ratio 会漂 → 放宽才能保住侧对帧，但会放进斜着的干扰轮廓 |
| `max_angle` 35° | 灯条接近竖直 | 正对时接近 0°，侧对时能到几十度。**这个值直接决定"要不要放过侧对帧"** —— 转盘模式必须放宽 |
| `min_fill_ratio` 0.8 | 实心矩形 | 反光、杂光晕形状不规则 → 填充率低。**通常不用动** |

### ③ 配对参数

| 参数 | 默认 | 怎么定 |
| --- | --- | --- |
| `min_light_ratio` | 0.7 | 两根灯条长度比。侧对时会差很多，调低会配错对 |
| `center_distance` 四点 | 0.8 / 3.2 / 3.2 / 5.5 | ★ **可以算出来**，见下 |
| `ArmorParams.max_angle` | 35.0 | 两灯条中心连线与水平的夹角，**要配合外参一起看** |
| `single_light_fallback` / `max_single_light_count` | 开 / 3 | 降级开关，**不是调优参数** |

**`center_distance` 为什么默认值就能用 —— 因为它是几何推出来的：**

```
归一化中心距 = 两灯条中心距 ÷ 平均灯条长度 = --armor-w ÷ --armor-h
             = 132 ÷ 57 = 2.32   → 落在 [0.8, 3.2) → SMALL ✓
```

**这就是它的验收方法**：拿你**实测的** `--armor-w` 除以 `--armor-h`，结果应该落在对应区间的**中间**，不能贴边。贴边说明板型量错了，或者该放宽区间（透视会让图像上的比值偏离物理比值）。

**`single_light_fallback`**：SINGLE 装甲板**没有左右灯条、进不了 PnP**，它唯一的作用是"我还看得见东西"（让 `armors_found` 不为 0）。所以：想在只有一根灯条时也知道"有东西" → 开着；嫌 SINGLE 污染统计或误选到它 → 关掉。

### ④ PnP 四件套 —— 全链路最重要的一站

#### `--fx --fy --cx --cy`：只能标定，不能试

**做**：打印一张棋盘格，**务必贴在硬板/玻璃上**（软纸会标歪，而且歪得很隐蔽）。举着它对着相机，前后左右上下各拍一些，15~30 张：

```bash
python3 tools/calib_intrinsic.py --dir calib_imgs --cols 9 --rows 6 --square-mm 25
```

> ⚠️ `--cols` / `--rows` 是**内角点**数，不是方格数。一张 10×7 格的棋盘，内角点是 9×6。数错了**一张都找不到**。

**三层验收**：

| 验收 | 判据 |
| --- | --- |
| 1. 脚本自报 | 重投影误差 < 1.0 px（`--max-error` 默认就是 1.0） |
| 2. **数量级 sanity check** ★ | `fx ≈ 镜头焦距(mm) ÷ 像素尺寸(mm)`。CS016-10UC 是 IMX273、像素 **3.45 µm**：<br>6 mm 镜头 → `6 ÷ 0.00345` = **1740**（相机原配）<br>12 mm 镜头 → `12 ÷ 0.00345` = **3478**（换过镜头）<br>标出来在 1000 附近、或者离上面两个数都很远 → 棋盘格没找到或方格尺寸填错，**这次标定整个废了** |
| 3. 光心位置 | `--debug-view` 画面正中那个**黄十字**（[autoaim_app.cpp:364](app/autoaim_app.cpp#L364)）画的就是代码认为的 `(cx, cy)`。标定后它应该还在中央附近（偏几十像素正常）；**跑到画面边上 = 标错了** |

> 📌 第 2 条最值钱 —— 它**不需要任何数据**就能告诉你"这次标定是不是完全跑偏了"。
> 想确认到底装的哪个镜头：`lsusb` 看不出，直接问队里，或者拿标定出来的 `fx` 反推 `焦距 = fx × 0.00345`。
> （`tools/make_test_bag.py` 里写死 `FX = 3478`，那是按"换了 12 mm 镜头"造的合成数据。）

#### `--armor-w` / `--armor-h`：先量，再反标

> ⚠️ **不是靶板外形尺寸**，是**灯条几何**：
> - `--armor-w` = 两根灯条**中心**之间的距离
> - `--armor-h` = **灯条本身的长度**

| 方法 | 做法 | 用途 |
| --- | --- | --- |
| **A. 卷尺直接量** | 量实物，把毫米数填进去 | **只给初值** |
| **B. 5 米反标** ★ | 靶板放在卷尺量准的 **5.00 m** 正前方，看 `--debug-view` 左上角的 **`d=?.??m`**（[autoaim_app.cpp:258](app/autoaim_app.cpp#L258)），改参数直到读数和卷尺一致 | **唯一的总验收** |

**为什么 B 是总验收**：它同时验证了内参、外参、板型、坐标变换这一整条链。任何一环错了，5 m 处都对不上。

> 📌 灵敏度参考：填 140 而不是 132，5 m 处距离直接错 **+5.15%**（约 26 cm）。
> 📌 方法 B 需要相机能出图。**相机还打不开的时候，方法 A 就是你能做的全部。**

#### `max_reprojection_error_px` 3.0：这是质检闸门，不是精度旋钮

| 方向 | 后果 |
| --- | --- |
| 收太紧（0.5） | 正常帧也被丢 → `pnp_rejected` 涨 → 频繁降级"保持" |
| 放太松（10） | 歪掉的解也放行 → 瞄准跳 |

**验收**：正常跟踪时 `pnp_rejected` 应**接近 0**，只有转盘侧对帧偶尔涨。

> ⚠️ **这个值涨了，说明上游有问题**（端点拟合歪了 / 配对错了 / 内参不对）。
> **修上游，不要靠放松这个值来掩盖** —— 那等于把烟雾报警器拆了。

### ⑤ 坐标变换：`--cam-pitch` / `--cam-yaw` / `--cam-roll`

**只能反解，符号必须实测。**

```bash
# 第 0 步：先确认实现在这台机器上是对的（不需要任何数据）
python3 tools/calib_extrinsic.py --selftest

# 然后：录"云台扫几个角度、每个角度停一下"，再反解
python3 tools/calib_extrinsic.py --images bags/calib --angles bags/calib/angles.csv \
    --fx <内参> --fy <内参> --cx <内参> --cy <内参>
```

**这一站的验收要特别小心 —— 残差漂亮不代表准：**

| 验收 | 判据 |
| --- | --- |
| 看**标准差**，别看残差 ★ | 脚本会打印每个参数的估计标准差。**标准差大的那个就是没标住**，别硬用 |
| 条件数警告 | 文档实测：残差 RMS 0.008°（看起来完美），但 `cam_pitch` 的**参数误差是残差的 80 倍**。这个反问题条件数很差 |
| 独立性检验 ★ | **云台扫一个角度，看它是不是朝对方向转。** 符号搞反的表现是**云台朝反方向抬** —— 现场只有一次机会，必须在实验室报掉 |

**两条实用建议**：

- **`--fix-yaw 0`**：相机在水平方向顺着炮口装是最常见的做法，把自由度让给别的参数反而更稳（`cam_yaw` 只通过 pitch 扫动的二阶耦合才可观测）
- **多扫几个 yaw 角度**：扫动范围越大、样本越多，条件数越好

**`cam_x/y/z` 要不要测**：量光心到云台转轴的距离，一般几厘米。5 m 外这点位移引入的角度误差 ≈ `atan(0.03 ÷ 5) ≈ 0.34°` —— 和其他误差同量级，**通常可以不管**。

---

### ⑥ 跟踪

#### `--rotation`：看现象定，不是参数拟合

| 判法 | 做法 |
| --- | --- |
| **用眼睛** | **站侧面看**：装甲板在圆周上跑 = `CAROUSEL`；位置不动、只是面朝方向在转 = `SELF_SPIN` |
| **看数字** | `--debug-dump` 看 `r`：稳定收敛到 **0.15~0.5** = CAROUSEL；持续趋近 0 = SELF_SPIN |

> ⚠️ **别用 `auto`**：判定是**单向的**，一旦切到 SELF_SPIN 就 `return`、不会再切回来，判错了只能重启程序。
> 📌 用错模式的代价：实测 **6 cm** 位置误差。

#### `max_match_distance` 0.15 m：可以算

这是"两帧之间允许目标跑多远"的门限。**每帧位移 = 速度 ÷ 帧率**：

```
转盘：角速度 2 rad/s × 半径 0.3 m = 0.6 m/s
即使用 30 fps 算：0.6 ÷ 30 = 0.02 m/帧   ← 门限 0.15 m 有 7 倍余量
```

**验收**：`--debug-dump` 里的 `posdiff` 应该**稳定小于**它。经常接近或超过 → 丢帧太多，或该放宽。

#### `max_match_yaw_diff` 1.0 rad：也有依据

超过它就算"跳变"，走 `handleArmorJump`。取值要卡在中间：

```
下界：相邻帧的 yaw 变化 = 2 rad/s ÷ 30 fps = 0.067 rad
上界：装甲板换块时的大跳 = π ≈ 3.14 rad
∴ 1.0 rad 落在合理区间
```

**验收**：`--debug-dump` 里的 `yawdiff`。

#### `tracking_thres` 5 / `lost_time_thres` 0.3 s

| 参数 | 物理含义 | 怎么定 |
| --- | --- | --- |
| `tracking_thres` 5 | 连续匹配几帧才敢出"带提前量"的解 | 是"**信任门**"。5 帧 @60fps ≈ 83 ms。调小 = 出解快但易跟错；调大 = 稳但反应慢 |
| `lost_time_thres` 0.3 s | 盲推多久才算真丢 | 上界由**实际遮挡时长**定。转盘侧对约 29% 帧中断，看 `--debug-dump` 里中断持续多久 |

#### `r_min` / `r_max` 0.12 / 0.40：可以拿卷尺量

`r` 就是**装甲板绕靶心转的半径** —— 转盘模式下它**就是转盘的半径**，量得到。这两个值是"物理上不可能更大/更小"的先验，负责把不合理的 `r` 慢慢拽回来。**只要你的实测值落在区间里就行。**

#### 噪声参数：没有正确答案，只有取舍

> 📌 打比方：**汽车减震的软硬**。"跟得紧"和"稳"是对立的，只能取一个你接受的折中。

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `s2qxyz_max/min` | 0.1 / 0.05 | 位置/速度的**过程噪声** `Q` |
| `s2qyaw_max/min` | 10.0 / 5.0 | yaw 的过程噪声 |
| `s2qr` | 80.0 | `r` 的过程噪声 |
| `r_xyz_factor` | 0.05 | 位置**测量噪声**，**和距离成正比** |
| `r_yaw` | 0.02 | yaw 的测量噪声 |

`Q` ↑ → 更信观测 → **跟得紧但抖**；`R` ↑ → 更信模型 → **稳但滞后**。

**两个容易看不懂的地方**（看 [tracker.cpp:88-135](modules/tracker/tracker.cpp#L88-L135)）：

1. **每个 `s2q` 都有一对 max/min，而且是自适应的**：
   ```
   位置噪声 x = exp(-|v_yaw|) × (max - min) + min
   yaw 噪声  y = exp(-|v_xy|)  × (max - min) + min
   ```
   —— **目标转得越快，位置的 `Q` 越小；平移越快，yaw 的 `Q` 越小**。所以当前实际生效的是 max 还是 min，取决于 `v_yaw` 和 `v_xy`。
2. **`r_xyz_factor` 是"相对误差"，不是绝对米数**：`R = r_xyz_factor × |z|`。所以**远距离会自动放宽**，你不用按距离分段调。

**调法（唯一可操作的）**：

- **别看画面，看 `--debug-dump` 里的 `r` 和 `v_yaw` 抖不抖**
- **一次只动一个参数** —— 一批一起换，出了效果也不知道是谁的功劳
- 三族的量纲不同（分别带 `dt²`/`dt³`/`dt⁴`，见 [tracker.cpp:100-105](modules/tracker/tracker.cpp#L100-L105)），**数值差很多是正常的，不要横向比大小**

#### `jump_p_inflate` 20.0

跳变时把协方差放大 = 告诉滤波器"**我不太确定了，接下来几帧多信观测**"。调大 → 跳变后收敛更快，但**坏观测也更容易被当真**。用默认值。

#### `auto_switch_frames` / `auto_grace_frames`（60 / 120）

只有 `--rotation auto` 才生效。既然上面建议了别用 auto，**这两个参数形同不存在**。

---

### ⑦ 瞄准

#### `latency_s` 0.05 / `gimbal_lag_s` 0.03：可以反标

| 参数 | 含义 | 量级 sanity check |
| --- | --- | --- |
| `latency_s` | 从**曝光**到**发出指令** | 曝光 5 ms + 传输 + 处理 → **几十 ms** 是合理量级 |
| `gimbal_lag_s` | 下位机收到 → 云台转到位 | 由云台响应速度决定 |

**反标法（唯一可靠的办法）**：让**已知角速度**的目标（转盘）转起来，看命中点是偏前还是偏后：

```
提前量误差 ≈ 目标角速度 × (latency 误差)
→ 用已知的 ω 和你观察到的偏角，反推 Δlatency
```

> 📌 **这两个参数不值得花一天**：0.08 s 的总延迟，在 2 rad/s 的转盘上大约对应 **9°** 角误差 —— 是会影响命中，但**不如先把 5 米距离验收过了**。

#### `flight_coeff` 1.0

"飞行时间缩放"，用来吸收弹速不准和气动阻力。**默认 1.0 = 完全相信 `v`。**

> ⚠️ **弹速 = 0 的缺陷没修之前，调这个毫无意义。** 见 [FIELD_CHECKLIST.md](readme/FIELD_CHECKLIST.md) 第三部分。

#### `enable_ballistic` / `gravity`

- `gravity` 9.8：**物理常量，不用调**
- `enable_ballistic`（默认开）：**开关，不是参数**。如果发现弹道补偿因弹速问题在静默失效，**关掉当纯直瞄反而更可预测**

#### `min_bullet_speed` 1.0：这是兜底值，不是调优参数

它现在的角色是"下位机发 0 时的兜底"，而 1.0 太小 → `v` 变成 1.0 m/s → 提前量翻倍 + 弹道静默失效。

> ⚠️ **该修的是上游**（让 `v` 不要被一个 0 覆盖掉），**不是把这个值调大**。
> 调大到 22 是个"万能兜底"，但同时**把下位机的问题永远掩盖了** —— 以后下位机真发 0 你也看不出来。

#### `max_v_yaw` 10.0：★ 转盘模式最需要调

**依据是实测的**：

```
转盘真实角速度 ≈ 2 rad/s
但装甲板侧对时检测中断，EKF 盲推后重捕，速度估计会炸到 55 rad/s（真值 2.0）
```

**所以 10 rad/s = 真值的 5 倍** —— 大到能容纳正常波动和跳变瞬态，小到能砍掉爆炸的估计。
**调法**：`--debug-dump` 看 `v_yaw` 的正常范围，取它的 **3~5 倍** 作为上限。

#### `max_lead_s` 0.6：有余量，别乱动

```
正确的 t_lead = t_flight + latency + gimbal_lag
             = 5 m ÷ 22 m/s + 0.05 + 0.03
             = 0.227 + 0.08 = 0.31 s
∴ 上限 0.6 s 正好留了一倍余量 ✓
```

**验收**：弹速可信时，`t_lead` 不该被这个值截断。

> ⚠️ **现在弹速 = 0 → `t_flight` 变成 5 s → 天天被截断到 0.6**，提前量从 0.31 变成 0.6、约翻倍 —— **这就是那个缺陷的现象**。

#### `min_aim_dist` / `max_aim_dist` 0.5 / 30

**合理性检查，不是精度参数**：解出来的距离必须落在这个窗口里，否则判 `invalid`。

- `max` 取 30 是"别误杀"（比赛场地最多十几米）
- `min` 0.5 防的是"解算飞到相机前面"
- **基本不用动**

#### `pitch_offset_deg` 0.0：枪口零位，只能打靶标

让云台瞄一个已知位置的点，看**落点系统性偏上/偏下多少**，换算成角度填进去。这是机械零位校准，**没打过靶就填 0**。

#### `max_jump_deg` 25

同 `max_reprojection_error_px` 的道理 —— **它疯涨说明模式选错了或解算在抖**。

> ⚠️ **`jump_rejected` 是现场判断"旋转模式选错没有"的最好指标。**
> 涨了就去改 `--rotation`，**不是把这个值调大**。

---

### ⑧ / ⑨ 开火

#### `--enable-fire`：这不是参数，是安全开关

流程：**先不开火跑通全部 → 确认瞄得准 → 才加**。第一次上场也不要加。

#### `yaw_tol_deg` / `pitch_tol_deg` 1.5：可以和靶板尺寸对着算

容差是"云台实际角对准目标角到什么程度才开火"。**换算成落点横向误差 = 距离 × tan(容差)**：

```
5 m 处：5 × tan(1.5°) = 0.131 m = 13.1 cm
装甲板宽度（两灯条中心距）：        13.2 cm
```

**两者同量级 → 1.5° 已经是"擦边"级别，不能再放松。**

> 📌 收紧会提高命中率，但**更难收敛 —— 可能一次都打不出去**（尤其云台有抖动时）。这是个明确的取舍。

#### `min_converge_s` / `fire_hold_s` 0.10 / 0.15

决定"开火那一刻的姿态"：

- `min_converge_s`：对准之后要**持续**保持多久才第一次开火
- `fire_hold_s`：开火后连续几帧置 1。0.15 s @ 100 Hz = 15 帧。**越长越可能多打几发，但枪口可能已经偏了**

#### `min_interval_s` 0.9：★ 唯一能用规则反推的参数

```
比赛规则：每轮 1 分钟内尽量打满 50 发（没发完也按 50 计分，零发射记 0）
→ 间隔上限 = 60 ÷ 50 = 1.2 s
∴ 默认 0.9 s 有 25% 余量
```

**按实际供弹速度调**：打不出去就说明供弹慢，加大这个值。

#### `max_shots` 50

规则给的，不用调。

#### `max_data_age_s` 0.2：安全参数

**依据**：串口断线时反馈会**冻在最后一帧**（不是变成 0），此时绝不开火 —— 只能靠时间戳识破。

| 方向 | 后果 |
| --- | --- |
| 调小 | 更安全，但串口一抖就停火 |
| 调大 | 更宽容，但**更危险** |

#### `require_tracking`

由 `--enable-tracker` **自动置位**（[config.cpp:164](app/config.cpp#L164)），不能直接传。含义是"没在跟踪就不许开火"。注意**跟踪中断的瞬间会停火**。

---

## 4. 只有一天时间，按这个顺序做

| 顺序 | 做什么 | 需要什么 | 大概耗时 |
| --- | --- | --- | --- |
| 1 | **量 `--armor-w` / `--armor-h`** | 卷尺 + 靶板 | 10 分钟 |
| 2 | **标内参** | 棋盘格 + 能出图的相机 | 1~2 小时 |
| 3 | **5 米距离验收** | 上面的 + 卷尺 | 30 分钟 |
| 4 | **标外参** | 上面 + 云台能转 + 串口 | 2~3 小时 |
| 5 | 定 `--exposure` / `--thres` | 现场光照 | 现场 30 分钟 |
| 6 | 定 `--rotation` | 看清靶板怎么动 | 10 分钟 |
| 7 | 其余（噪声、夹取、开火） | 前面的都准了 | 慢慢试 |

**1~4 决定"准不准"，5~7 决定"稳不稳"。顺序反了就是白干。**

---

## 5. 这些根本不用调

| 参数 | 为什么 |
| --- | --- |
| `gravity` | 物理常量 9.8 |
| `max_shots` | 规则给的 50 |
| `ArmorParams.center_distance` | 用 `--armor-w ÷ --armor-h` **算出来验证**，不用试 |
| `LightParams.min_fill_ratio` | 灯条本来就是实心矩形，0.8 够用 |
| `min_aim_dist` / `max_aim_dist` | 只是合理性窗口，填宽一点就行 |
| `cam_x/y/z` | 几厘米的偏心在 5 m 外只有 0.34°，可以不管 |
| `jump_p_inflate` | 标准手法，默认值就好 |
| `auto_switch_frames` / `auto_grace_frames` | 前提是你不用 `--rotation auto` |

---

## 6. 一页速查：每个参数各归哪一类

| 参数 | 怎么弄 | 验收 / 依据 |
| --- | --- | --- |
| `--exposure` `--gain` | 看着画面拧 | 灯条全白不糊；`armors_found` 涨 |
| `auto_exposure` | 编译前改代码 | 比赛必须关 |
| `--thres` | 拖滑块试 | 灯条成条、背景全黑、假框少 |
| `LightParams.*` | 改代码重编 | 被丢/被留的轮廓长什么样 |
| `ArmorParams.center_distance` | **算**：`--armor-w ÷ --armor-h` | = 2.32，落在区间中间 |
| `--fx --fy --cx --cy` | 棋盘格标定 | 重投影 <1px；**fx ≈ 1740(6mm) 或 3478(12mm)**；黄十字在中央 |
| `--armor-w` `--armor-h` | 卷尺量 → **5 米反标** | `d=5.00m` 和卷尺一致 |
| `max_reprojection_error_px` | 别动 | `pnp_rejected` ≈ 0 |
| `--cam-pitch/yaw/roll` | 云台扫角度反解 | 看**标准差**；扫一下看方向对不对 |
| `--rotation` | 看现象（侧站 / 看 `r`） | 别用 auto |
| `max_match_distance` | 算：速度 ÷ 帧率 | `posdiff` 稳定小于它 |
| `max_match_yaw_diff` | 卡在 0.067 和 3.14 之间 | `yawdiff` |
| `tracking_thres` `lost_time_thres` | 信任门 / 按遮挡时长 | — |
| `r_min` `r_max` | **卷尺量转盘半径** | 实测值落在区间内 |
| 噪声一族 | 试，一次动一个 | `--debug-dump` 看 `r`/`v_yaw` 抖不抖 |
| `latency_s` `gimbal_lag_s` | 已知 ω 反标 | 几十 ms 量级 |
| `min_bullet_speed` | **修上游，别调这个** | — |
| `max_v_yaw` | `v_yaw` 正常值的 3~5 倍 | 真值 2，爆炸 55 |
| `max_lead_s` | 有余量，别动 | 正确 `t_lead` ≈ 0.31 s |
| `pitch_offset_deg` | 打靶标 | 落点系统性偏移 |
| `max_jump_deg` | 别动 | `jump_rejected` ≈ 0 |
| `yaw_tol_deg` `pitch_tol_deg` | 用 `距离 × tan(容差)` 校核 | 5 m 处 ≈ 13 cm ≈ 板宽 |
| `min_interval_s` | 规则反推 | ≤ 1.2 s，默认 0.9 有余量 |
| `max_data_age_s` | 安全取舍 | 别调大 |

---

