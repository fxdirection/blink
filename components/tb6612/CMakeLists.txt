# 这是 ESP-IDF component 的构建声明。
#
# 可以类比 Python 包：
#   SRCS         -> 这个包需要执行/编译的实现文件
#   INCLUDE_DIRS -> 对外公开模块接口的目录
#   REQUIRES     -> 这个包依赖的其他包
#
# ESP-IDF 会把当前文件夹识别为名为 "tb6612" 的 component，编译后链接进固件。
idf_component_register(
    # tb6612.c 是模块实现。
    SRCS "tb6612.c"

    # 其他模块可以通过 #include "tb6612.h" 使用公开接口。
    INCLUDE_DIRS "include"

    # 驱动需要 GPIO、LEDC PWM、PCNT 编码器、微秒延时和系统计时功能。
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_pcnt
        esp_rom
        esp_timer
        pid
)
