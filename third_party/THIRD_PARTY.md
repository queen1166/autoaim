# 第三方代码与许可

本工程的部分模块派生自开源项目。按 MIT 许可要求，此处逐文件声明来源，
且每个派生文件头部都保留了原始版权声明。

规则书 §1 明确允许使用开源项目与预训练模型（无需申报）；
§5 禁止的是 SRM 队内既有代码 —— 本工程**不含任何** `control-2026` 的代码。

---

## rm_auto_aim

- **仓库**：https://github.com/FaterYU/rm_auto_aim
- **许可**：MIT License
- **版权**：Copyright (c) 2022 ChenJun；Copyright (c) 2024 Zheng Yu
- **上游本身也是 MIT 项目**（`armor_detector/CMakeLists.txt` 声明），
  其源头可追溯到 RM 视觉圈公开的装甲板识别实现

### 逐文件对应

| 本工程文件 | 派生自 | 改动摘要 |
| --- | --- | --- |
| `modules/detect/armor.h` | `armor_detector/include/armor_detector/armor.hpp` | 命名空间；`ARMOR_TYPE_STR` 改 `inline`；新增 `ArmorType::SINGLE` |
| `modules/detect/armor_detector.h` | `armor_detector/include/armor_detector/detector.hpp` | 删 debug 消息成员；补 `<memory>`；`classifier` 改可空 |
| `modules/detect/armor_detector.cpp` | `armor_detector/src/detector.cpp` | 删 ROS debug 填充；classifier 加空指针保护；新增单灯条降级 |
| `modules/detect/number_classifier.h` | `armor_detector/include/armor_detector/number_classifier.hpp` | 改成抽象接口，隔离 ONNX 依赖（实现待补） |
| `modules/solver/pnp_solver.h` | `armor_detector/include/armor_detector/pnp_solver.hpp` | 删未用的 ROS include；尺寸参数化 |
| `modules/solver/pnp_solver.cpp` | `armor_detector/src/pnp_solver.cpp` | 求解器 `IPPE` → `ITERATIVE`（见下）；新增 `reprojectionError` |
| `modules/tracker/extended_kalman_filter.h` | `armor_tracker/include/armor_tracker/extended_kalman_filter.hpp` | 仅命名空间，逻辑逐字未改 |
| `modules/tracker/extended_kalman_filter.cpp` | `armor_tracker/src/extended_kalman_filter.cpp` | 仅命名空间，逻辑逐字未改 |
| `modules/tracker/tracker.h` | `armor_tracker/include/armor_tracker/tracker.hpp` | 去 ROS 类型；新增双模式 |
| `modules/tracker/tracker.cpp` | `armor_tracker/src/tracker.cpp`<br>`armor_tracker/src/tracker_node.cpp:34-125`（EKF 模型） | 去 ROS；半径硬钳位改软约束；单靶板短路 |

### 本工程对上游的实质改动

1. **求解器 `SOLVEPNP_IPPE` → `SOLVEPNP_ITERATIVE`。**
   400 次蒙特卡洛实测（靶板 140×125 @ 5m，像素噪声 1px）：两者中位精度相当，
   但 IPPE 的 p95 误差比 ITERATIVE 差 10 倍以上（12mm 镜头：127cm vs 11cm），
   存在解分支翻转导致的重尾。100Hz 下这是致命的。

2. **半径 `r` 从硬钳位改成软约束 + `RotationMode` 双模式。**
   上游把 `r` 硬钳在 `[0.12, 0.4]`。这对"装甲板绕中心公转"是对的，
   但对"绕自身轴自转"（真值 `r = 0`）会持续把 `xc` 往物理上不存在的圆心拉。
   实测：用错模式时位置误差 6cm，`r` 被撑在下界附近。

3. **`updateArmorsNum()` 短路为 1 块装甲板。**
   上游靠装甲板数字判 4/2/3 块；校内赛靶板无数字，会误入 `NORMAL_4` 分支
   并 `swap(dz, another_r)` 污染状态。

4. **新增单灯条降级（`ArmorType::SINGLE`）。**
   只找到一个合格灯条时给方位角兜底。单灯条解不出距离和姿态，**不要用它闭环开火**。

5. **保留但标注了一处上游笔误。**
   `tracker.cpp` 的 `u_q` 里 `q_y_vy` 用的是 `x` 而不是 `y`。
   量纲上不一致，但那是**已知能工作**的行为，非 ROS 移植不该顺手改数值。
   代码里有注释说明。

---

## 本工程原创部分

以下模块**不派生自任何外部项目**，是为本次校内赛新写的：

| 文件 | 说明 |
| --- | --- |
| `bsp/serial/serial_port.{h,cpp}` | termios 封装 |
| `modules/protocol/srm_protocol.{h,cpp}` | 校内赛自瞄协议 18B/30B 编解码 |
| `modules/solver/coord_transform.{h,cpp}` | 相机系→云台系（上游靠 URDF，此处实测反解） |
| `modules/aim/aim_solver.{h,cpp}` | 瞄准层与提前量（上游没有这一层） |
| `modules/aim/fire_decision.{h,cpp}` | 开火决策（上游没有） |
| `modules/tracker/target_state.h` | 状态向量契约 |
| `modules/tracker/armor_measurement.h` | 替代 ROS 消息类型 |
| `modules/tracker/tracker_config.h` | 配置集中 |
| `tools/protocol_check.py` | 独立第二实现，交叉验证字节序 |
| `tests/*` | 全部离线测试 |
| `app/main_stage1.cpp` | 阶段 1 云台扫动测试 |

---

## 待补

- `models/mlp.onnx` + `models/label.txt` —— 装甲板数字分类模型，同样来自
  `rm_auto_aim`（MIT）。校内赛只有一个无数字靶板，默认**不启用**，
  所以暂未拷入。若将来启用，此文件需一并标注来源。

---

**SRM 队内代码：本工程未使用任何 `control-2026` 的代码。**
