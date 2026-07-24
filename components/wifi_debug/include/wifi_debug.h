/**
 * @file wifi_debug.h
 * @brief ESP32-S3 SoftAP + TCP server，复用 debug_comm 帧协议
 *
 * 启动后 ESP32-S3 自己成为 WiFi 热点（默认 SSID/PASSWORD 见 Kconfig），
 * PC/手机连上后用 TCP 连接 192.168.4.1:8888（或 espcar.local:8888 via mDNS），
 * 直接收发与 UART 同一套二进制帧，命令字含义完全一致。
 */
#ifndef WIFI_DEBUG_H
#define WIFI_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 启动 SoftAP + TCP server；失败不阻塞 main（UART 调试仍可用） */
void wifi_debug_start(void);

/** 当前是否有客户端连接 */
bool wifi_debug_has_client(void);

#ifdef __cplusplus
}
#endif
#endif /* WIFI_DEBUG_H */