# 远程调试工具

PC 端调试软件，提供 **命令行版** 和 **GUI 版** 两种。

## 0. 选哪个？

| 工具 | 文件 | 用途 |
|---|---|---|
| **可视化面板**（推荐） | `tools/debug_panel_gui.py` | 全功能 GUI，WASD 按钮 + 滑条调 PID + 实时仪表 |
| 命令行版 | `tools/debug_panel.py` | 只用键盘就能跑，SSH/CI 环境也能用 |

两者共用协议层 `tools/esp_link.py`，与 ESP32 端 `components/debug_comm/` 一一对应。

## 1. 安装依赖

```bash
pip install pyserial pyside6
```

> 如果机器上有多个 Python（ESP-IDF 自带一个、conda 一个、Microsoft Store 一个…），请确认 `python` 指向你想要的那个，例如：
> ```bash
> C:/Users/86181/miniconda3/python.exe -m pip install pyserial pyside6
> ```

## 2. 烧录 ESP32 固件

```bash
idf.py -p COMx flash
```

> ⚠️ 烧完先关掉 `idf.py monitor`，串口同一时间只能有一个 reader。

## 3. 启动 GUI

```bash
python tools/debug_panel_gui.py
```

界面分三块：

| 区域 | 控件 | 作用 |
|---|---|---|
| **顶栏** | 串口下拉 / 波特率 / 连接 / 断开 | 选择并打开串口 |
| **底盘控制**（左） | WASD 按钮、Q/E 自转、Vx/Vy/Vw 显示 | 平移与自转 |
| **电机调参**（右） | 3 张电机卡片（目标/实测 RPM、kp/ki/kd 滑条） | 实时调 PID |
| **底栏** | 紧急停止、恢复默认 PID、Ping | 全局动作 |

### 快捷键

| 键 | 作用 |
|---|---|
| `WASD` | 平移（前/后/左/右） |
| `Q / E` | 顺/逆自转 |
| `1` `2` `3` | 高亮选中对应电机卡 |
| `+ / -` | 选中电机 kp ±0.05 |
| `[ / ]` | 选中电机 ki ±0.05 |
| `, / .` | 选中电机 kd ±0.05 |
| `Space` | 紧急停止 |

> GUI 模式下也可直接点击 WASD 按钮（按住持续生效），所有按钮和键盘共享同一组状态。

## 4. 启动命令行版

```bash
python tools/debug_panel.py --port COM7
```

按键提示在屏幕顶部实时显示。

## 5. 协议

帧格式（小端）：

```
[0xAA] [CMD] [LEN] [PAYLOAD...] [CHK]
```

`CHK = (0xAA + CMD + LEN + sum(payload)) & 0xFF`

| CMD | 方向 | 含义 | Payload |
|-----|------|------|---------|
| 0x01 | PC→ESP | 设置底盘速度 | Vx, Vy, Vw (3 float) |
| 0x02 | PC→ESP | 设置某个电机 PID | idx(u8) + kp, ki, kd (3 float) |
| 0x03 | PC→ESP | 设置最大速度 | Vx_MAX, Vy_MAX, Vw_MAX (3 float) |
| 0x04 | PC→ESP | 紧急停止 | 空 |
| 0x05 | PC→ESP | ping / 请求状态 | 空 |
| 0x10 | ESP→PC | 状态上报 | 3×target_rpm + 3×real_rpm + 9×pid + 3×vel = 72B |

详细常量见 `components/debug_comm/include/debug_comm.h`。

## 6. 故障排查

- **窗口打不开 / `ModuleNotFoundError: No module named 'PySide6'`**  
  多 Python 装到了别处。用 `where python` 看哪个 `python` 在 PATH 最前面，给它装依赖：
  ```bash
  C:/Users/86181/miniconda3/python.exe -m pip install pyserial pyside6
  C:/Users/86181/miniconda3/python.exe tools/debug_panel_gui.py
  ```

- **串口下拉为空**  
  按一下「刷新列表」。如果没有列出任何串口，先用 ESP-IDF 驱动装好 USB-UART。

- **RPM 一直是 0 但控制有效**  
  检查 `CONFIG_ENCODER_COUNTS_PER_REVOLUTION`（在 `sdkconfig` 里）是否设置对了电机每转一圈的实际四倍频计数；不设置也能给 `counts_per_second`，但 RPM 算不出来。

- **GUI 启动但串口面板无响应**  
  关掉 `idf.py monitor` 再试。