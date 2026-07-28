# TB6612 电机驱动模块

## 它是“库”吗？

是的。更准确地说，它是一个 **ESP-IDF component（组件）**。ESP-IDF
构建项目时会把它编译成库，再链接进最终固件。

如果你熟悉 Python，可以这样类比：

| Python | 这里的 C / ESP-IDF |
|---|---|
| `import tb6612` | `#include "tb6612.h"` |
| 一个 Python package 文件夹 | `components/tb6612/` |
| 类或对象 | `tb6612_t` 结构体变量 |
| 对象的方法 | `tb6612_init()`、`tb6612_set_speed()` 等函数 |
| `_private_method()` | `tb6612.c` 中的 `static` 函数 |
| `requirements.txt` 中的依赖 | `CMakeLists.txt` 的 `REQUIRES` |
| 抛出异常 | 返回 `esp_err_t` 错误码 |

它不是从网上安装的第三方 Python 包，而是这个项目自己维护的本地模块。

## 这个模块解决什么问题

TB6612 是电机驱动芯片，ESP32 不能直接带动直流电机，所以需要它把
ESP32 的小信号转换成电机需要的电流。

这个模块把底层细节封装起来，调用者只需要：

1. 告诉它引脚和 PWM 参数；
2. 初始化；
3. 传入 `-1000 ~ 1000` 控制方向和输出；
4. 需要时读取编码器速度；
5. 最后释放资源。

模块内部负责：

- 配置 IN1、IN2 方向 GPIO；
- 用 LEDC 产生 PWM；
- 换向前先停 1 ms，减小冲击；
- 用 PCNT 对 AB 相编码器进行四倍频计数；
- 根据真实采样时间换算 counts/s 和 RPM；
- 初始化失败时清理已经申请的资源。

## 文件结构

```text
components/tb6612/
├── include/
│   └── tb6612.h      # 公开接口：使用者主要读这个文件
├── tb6612.c          # 内部实现：GPIO、PWM、编码器和清理逻辑
├── CMakeLists.txt    # 告诉 ESP-IDF 如何编译和链接这个模块
└── README.md         # 当前说明
```

## 先认识硬件引脚

一路电机通常使用这些信号：

| 信号 | 方向 | 作用 |
|---|---|---|
| `IN1` | ESP32 → TB6612 | 和 IN2 一起决定转向/停止方式 |
| `IN2` | ESP32 → TB6612 | 和 IN1 一起决定转向/停止方式 |
| `PWM` | ESP32 → TB6612 | 用占空比决定输出强弱 |
| `Encoder A` | 编码器 → ESP32 | 编码器 A 相脉冲 |
| `Encoder B` | 编码器 → ESP32 | 编码器 B 相脉冲 |
| `STBY` | 外部接线 | 本模块不控制，必须在硬件上拉高 |

TB6612 的方向真值在本模块中约定为：

| IN1 | IN2 | 行为 |
|---:|---:|---|
| 1 | 0 | 正转 |
| 0 | 1 | 反转 |
| 0 | 0 | 滑行停止（coast） |
| 1 | 1 | 主动刹车（brake） |

## 最小使用示例

```c
#include "tb6612.h"

/* static 表示这个电机对象只在当前 .c 文件中使用。 */
static tb6612_t motor = TB6612_CONFIG_DEFAULT_WITH_ENCODER(
    GPIO_NUM_16,  /* IN1 */
    GPIO_NUM_17,  /* IN2 */
    GPIO_NUM_18,  /* PWM */
    GPIO_NUM_41,  /* Encoder A */
    GPIO_NUM_42,  /* Encoder B */
    1320          /* 输出轴转一圈的四倍频计数 */
);

void example(void)
{
    /* ESP_ERROR_CHECK 类似“失败就立刻报错并停止”。 */
    ESP_ERROR_CHECK(tb6612_init(&motor));

    ESP_ERROR_CHECK(tb6612_set_speed(&motor, 500));   /* 正转 50% */
    ESP_ERROR_CHECK(tb6612_set_speed(&motor, -300));  /* 反转 30% */

    tb6612_speed_t speed = {0};
    ESP_ERROR_CHECK(tb6612_read_speed(&motor, &speed));

    if (speed.rpm_valid) {
        printf("speed = %.2f RPM\n", (double)speed.rpm);
    }

    ESP_ERROR_CHECK(tb6612_brake(&motor));
    ESP_ERROR_CHECK(tb6612_deinit(&motor));
}
```

注意：`tb6612_set_speed(&motor, 500)` 中的 `500` 是 **PWM 50%**，不是
500 RPM。当前项目在上层 `components/motor/` 中使用 PID，把目标 RPM
换算成这里需要的 PWM 千分比。

## `&motor` 是什么意思

C 函数收到的是 `tb6612_t *motor`，星号表示“指向电机对象的指针”。
调用时写 `&motor`，意思是“把 motor 这个对象的地址交给函数”。

可以粗略类比 Python：

```python
motor.set_speed(500)
```

这里的 C 写法是：

```c
tb6612_set_speed(&motor, 500);
```

C 不会像 Python 一样自动把对象传给方法，所以需要显式写出 `&motor`。

## 一个电机对象里保存了什么

`tb6612_t` 分为三部分：

1. **方向和 PWM 配置**
   - IN1、IN2、PWM GPIO；
   - PWM 频率、分辨率、timer 和 channel。
2. **编码器配置**
   - A/B 相 GPIO；
   - 毛刺滤波时间；
   - 每圈计数；
   - 是否反转速度符号。
3. **内部状态**
   - 是否已初始化；
   - 当前方向；
   - PCNT 编码器上下文。

以下划线开头的 `_initialized`、`_direction`、`_encoder_context` 应视为
私有属性，不要从其他模块直接修改。

## PWM、RPM 和编码器是什么关系

```text
上层 PID 输出
    ↓  -1000 ~ 1000
tb6612_set_speed()
    ↓  方向 GPIO + PWM 占空比
TB6612 驱动电机
    ↓
编码器产生 A/B 脉冲
    ↓
ESP32 PCNT 计数
    ↓
tb6612_read_speed()
    ↓  counts/s 和 RPM
上层 PID 再次计算
```

PWM 是给电机的“控制量”，RPM 是编码器测出的“实际结果”。两者并不天然
一一对应，因为电池电压、负载和摩擦都会改变相同 PWM 下的实际转速。

## 为什么编码器需要 A、B 两相

A、B 两个方波相差约四分之一周期。程序观察“谁先变化”，就能判断旋转
方向；同时统计 A/B 的上升沿和下降沿，可以在一个完整周期中得到 4 次
计数，所以叫四倍频。

本模块使用两个 PCNT channel：

- A 通道观察 A 的边沿，用 B 的电平判断加一还是减一；
- B 通道观察 B 的边沿，用 A 的电平判断加一还是减一。

如果实测正负号与预期相反，不必交换接线，可以把
`encoder_invert_direction` 设为 `true`。

## `coast` 和 `brake` 的区别

- `tb6612_coast()`：撤掉驱动力，电机靠惯性慢慢停；
- `tb6612_brake()`：主动短接电机端子，通常停得更快。

`tb6612_set_speed(..., 0)` 当前选择的是 `coast()`。

## 多电机使用规则

每个电机创建一个独立的 `tb6612_t`：

```c
static tb6612_t motor1 = { /* ... channel 0 ... */ };
static tb6612_t motor2 = { /* ... channel 1 ... */ };
static tb6612_t motor3 = { /* ... channel 2 ... */ };
```

必须遵守：

- 每个同时工作的电机使用不同的 `pwm_channel`；
- PWM 频率和分辨率相同时，可以共用一个 `pwm_timer`；
- GPIO 不能在同一路或不同外设之间冲突；
- 一个编码器只能由一个任务调用 `tb6612_read_speed()`，因为读取后会清零；
- 电机电源、TB6612、编码器和 ESP32 必须共地。

本项目的三个真实配置位于 `components/motor/motor.c`，不是本文件夹中的
默认宏。

## 常见错误码

| 错误码 | 常见原因 |
|---|---|
| `ESP_OK` | 成功 |
| `ESP_ERR_INVALID_ARG` | NULL、GPIO 冲突、PWM 参数错误、速度超范围 |
| `ESP_ERR_INVALID_STATE` | 重复初始化，或初始化前调用设速/测速 |
| `ESP_ERR_NOT_SUPPORTED` | 没配置编码器却调用了测速 |
| `ESP_ERR_NO_MEM` | 创建编码器上下文时内存不足 |

如果使用 `ESP_ERROR_CHECK(...)`，一旦返回值不是 `ESP_OK`，ESP-IDF 会打印
错误并终止当前程序流程，方便开发阶段尽快发现问题。

## 推荐阅读顺序

1. 先读本 README；
2. 再读 `include/tb6612.h`，只看结构体和 6 个公开函数；
3. 在 `tb6612.c` 中先找同名的 6 个公开函数；
4. 最后再看 `init_encoder()` 中的 PCNT 正交解码细节。

不需要第一次就理解 `pcnt_channel_set_edge_action()` 的每个枚举。先记住：
它们共同负责让正转得到正计数、反转得到负计数即可。

## 排查问题

### 电机完全不转

- 检查 TB6612 `STBY` 是否拉高；
- 检查电机是否使用独立电源；
- 检查 ESP32 与电机电源是否共地；
- 检查 IN1、IN2、PWM 引脚是否和代码一致；
- 确认 `tb6612_init()` 返回 `ESP_OK`。

### 电机方向相反

- 电机本身方向相反：交换电机两根线，或调整上层正负方向；
- 只有测速符号相反：设置 `encoder_invert_direction = true`。

### RPM 一直是 0

- 确认 A/B 相确实有脉冲；
- 确认 `encoder_counts_per_revolution > 0`；
- 查看 `counts_per_second` 是否也为 0；
- 检查编码器供电、共地和 GPIO。

### 多个电机初始化失败

最常见原因是多个实例误用了同一个 LEDC `pwm_channel`，或者 GPIO 发生冲突。
