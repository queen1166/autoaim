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

## 二、比赛机器上的一次性准备（硬件相关）

7. **海康 MVS SDK**。这是最高优先级风险项，装之前先问赛务。允许的话从海康官网下载 .deb
   装到 `/opt/MVS`，然后重新配置和编译：

   ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ```

   配置时出现"海康 MVS SDK: 已找到"就成功。没装时 `--hik` 会在启动时被拒绝并提示。

8. **串口权限**。先用 `ls /dev/serial/by-id/` 记下真实设备名；把用户
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
| **EKF 噪声、瞄准夹取、开火阈值都没有开关** | 这些全是"编译前"的，改源码重编才生效 |

---

