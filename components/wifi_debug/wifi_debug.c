/**
 * @file wifi_debug.c
 * @brief ESP32-S3 SoftAP + (可选) STA + TCP server，复用 debug_comm 帧协议
 *
 *  - SoftAP: SSID = CONFIG_WIFI_AP_SSID,  默认 IP 192.168.4.1
 *  - STA:    可选连一个外部 WiFi，连上后会同时保留 SoftAP (AP+STA)
 *  - TCP server: 0.0.0.0:CONFIG_WIFI_TCP_PORT（默认 8888）
 *  - mDNS: espcar.local
 *  - 每个 TCP client 一个任务，跑同一套帧解析/发送逻辑
 */
#include "wifi_debug.h"
#include "debug_comm.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_netif_ip_addr.h"

#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "chassis.h"
#include "motor.h"
#include "pid.h"

#define TAG "wifi_dbg"

/* ------- 配置（Kconfig 提供；这里给默认） ------- */
#ifndef CONFIG_WIFI_AP_SSID
#define CONFIG_WIFI_AP_SSID       "ESP32-Car"
#endif
#ifndef CONFIG_WIFI_AP_PASSWORD
#define CONFIG_WIFI_AP_PASSWORD   "12345678"
#endif
#ifndef CONFIG_WIFI_AP_CHANNEL
#define CONFIG_WIFI_AP_CHANNEL    6
#endif
#ifndef CONFIG_WIFI_TCP_PORT
#define CONFIG_WIFI_TCP_PORT      8888
#endif
#ifndef CONFIG_WIFI_ENABLE_STA
#define CONFIG_WIFI_ENABLE_STA    0
#endif
#ifndef CONFIG_WIFI_STA_SSID
#define CONFIG_WIFI_STA_SSID      ""
#endif
#ifndef CONFIG_WIFI_STA_PASSWORD
#define CONFIG_WIFI_STA_PASSWORD  ""
#endif
#ifndef CONFIG_WIFI_MDNS_HOST
#define CONFIG_WIFI_MDNS_HOST     "espcar"
#endif

/* 当前已连接 client（只支持一个调试 client，避免多个 PC 抢控制） */
static int s_client_fd = -1;
static volatile bool s_running = false;

/* ============ TCP 帧收发 ============ */
static inline void put_f32_le(uint8_t *p, float v)
{
    union { float f; uint32_t u; } conv = { .f = v };
    p[0] = (uint8_t)(conv.u      );
    p[1] = (uint8_t)(conv.u >>  8);
    p[2] = (uint8_t)(conv.u >> 16);
    p[3] = (uint8_t)(conv.u >> 24);
}
static inline float get_f32_le(const uint8_t *p)
{
    union { float f; uint32_t u; } conv = {
        .u = ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
             ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)
    };
    return conv.f;
}

static void send_frame(int fd, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t hdr[3] = { 0xAA, cmd, len };
    uint8_t chk = 0;
    for (int i = 0; i < 3; ++i) chk += hdr[i];
    for (int i = 0; i < len; ++i) chk += payload[i];
    send(fd, hdr, sizeof(hdr), 0);
    if (payload && len) send(fd, payload, len, 0);
    send(fd, &chk, 1, 0);
}

/* 推送 STATE_REPORT */
static void send_state_to(int fd)
{
    extern ROBOT_CHASSIS ROBOT_CHASSI;
    extern MOTOR_REAL_INFO chassisMotorRealInfo[3];
    extern PID_T pid_chassis_rpm[3];

    uint8_t payload[72];
    uint8_t *p = payload;
    for (int i = 0; i < 3; ++i) put_f32_le(p + i*4, ROBOT_CHASSI.Motor_Target_RPM[i]);
    p += 12;
    for (int i = 0; i < 3; ++i) put_f32_le(p + i*4, chassisMotorRealInfo[i].REAL_RPM);
    p += 12;
    for (int i = 0; i < 3; ++i) {
        put_f32_le(p + 0, pid_chassis_rpm[i].kp);
        put_f32_le(p + 4, pid_chassis_rpm[i].ki);
        put_f32_le(p + 8, pid_chassis_rpm[i].kd);
        p += 12;
    }
    put_f32_le(p + 0, ROBOT_CHASSI.Vx);
    put_f32_le(p + 4, ROBOT_CHASSI.Vy);
    put_f32_le(p + 8, ROBOT_CHASSI.Vw);
    p += 12;
    send_frame(fd, 0x10, payload, (uint8_t)(p - payload));
}

/* 复用 debug_comm 的解析器：把帧 dispatch 到 apply_* */
static void handle_frame(int fd, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd) {
    case 0x01: { /* SET_VELOCITY */
        if (len != 12) return;
        debug_comm_apply_velocity(
            get_f32_le(payload+0), get_f32_le(payload+4), get_f32_le(payload+8));
        break;
    }
    case 0x02: { /* SET_PID */
        if (len != 13 || payload[0] >= 3) return;
        debug_comm_apply_pid(payload[0],
            get_f32_le(payload+1), get_f32_le(payload+5), get_f32_le(payload+9));
        break;
    }
    case 0x03: { /* SET_MAX_VEL */
        if (len != 12) return;
        debug_comm_apply_max_velocity(
            get_f32_le(payload+0), get_f32_le(payload+4), get_f32_le(payload+8));
        break;
    }
    case 0x04: /* STOP */
        debug_comm_apply_emergency_stop();
        break;
    case 0x05: /* PING -> 推一帧状态 */
        send_state_to(fd);
        break;
    default:
        break;
    }
}

typedef struct {
    enum { ST_HEAD, ST_CMD, ST_LEN, ST_PAYLOAD, ST_CHK } state;
    uint8_t cmd, len, idx, chk;
    uint8_t payload[64];
} parser_t;

static void parser_reset(parser_t *p) { memset(p, 0, sizeof(*p)); p->state = ST_HEAD; }

static void parser_feed(parser_t *p, uint8_t b)
{
    switch (p->state) {
    case ST_HEAD: if (b == 0xAA) { p->state = ST_CMD; p->chk = 0xAA; } break;
    case ST_CMD:  p->cmd = b; p->chk += b; p->state = ST_LEN; break;
    case ST_LEN:
        p->len = b; p->chk += b;
        if (p->len > 64) parser_reset(p);
        else if (p->len == 0) p->state = ST_CHK;
        else { p->state = ST_PAYLOAD; p->idx = 0; }
        break;
    case ST_PAYLOAD:
        p->payload[p->idx++] = b; p->chk += b;
        if (p->idx >= p->len) p->state = ST_CHK;
        break;
    case ST_CHK:
        if (p->chk == b) handle_frame(-1 /* unused */, p->cmd, p->payload, p->len);
        parser_reset(p);
        break;
    }
}

/* 单个 client 的任务 */
static void client_task(void *arg)
{
    int fd = (int)(intptr_t)arg;
    parser_t p; parser_reset(&p);
    uint8_t buf[128];
    s_client_fd = fd;
    ESP_LOGI(TAG, "client connected fd=%d", fd);

    while (s_running) {
        int n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) {
            ESP_LOGW(TAG, "client disconnected (n=%d errno=%d)", n, errno);
            break;
        }
        for (int i = 0; i < n; ++i) parser_feed(&p, buf[i]);
    }
    close(fd);
    if (s_client_fd == fd) s_client_fd = -1;
    vTaskDelete(NULL);
}

/* TCP server accept 循环 */
static void server_task(void *arg)
{
    (void)arg;
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) { ESP_LOGE(TAG, "socket: errno=%d", errno); vTaskDelete(NULL); return; }

    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(CONFIG_WIFI_TCP_PORT);
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind: errno=%d", errno); close(listen_fd); vTaskDelete(NULL); return;
    }
    listen(listen_fd, 1);
    ESP_LOGI(TAG, "TCP server listening on port %d", CONFIG_WIFI_TCP_PORT);

    while (s_running) {
        struct sockaddr_in cli; socklen_t cl = sizeof(cli);
        int cfd = accept(listen_fd, (struct sockaddr *)&cli, &cl);
        if (cfd < 0) { if (errno == EINTR) continue; break; }

        char ipstr[16];
        inet_ntop(AF_INET, &cli.sin_addr, ipstr, sizeof(ipstr));
        ESP_LOGI(TAG, "accept from %s:%d", ipstr, ntohs(cli.sin_port));

        /* 单 client 模式：已连一个就先踢掉 */
        if (s_client_fd >= 0) {
            ESP_LOGW(TAG, "kicking previous client");
            close(s_client_fd);
            s_client_fd = -1;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        xTaskCreate(client_task, "tcp_client", 4096, (void *)(intptr_t)cfd, 4, NULL);
    }
    close(listen_fd);
    vTaskDelete(NULL);
}

/* ============ WiFi 事件 ============ */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "STA disconnected, retry...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&e->ip_info.ip));
    }
}

/* ============ SoftAP + 可选 STA ============ */
static void wifi_init_apsta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
#if CONFIG_WIFI_ENABLE_STA
    esp_netif_create_default_wifi_sta();
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {0};

    /* SoftAP 配置 */
    strncpy((char *)wifi_cfg.ap.ssid, CONFIG_WIFI_AP_SSID, sizeof(wifi_cfg.ap.ssid));
    wifi_cfg.ap.ssid_len = strlen(CONFIG_WIFI_AP_SSID);
    strncpy((char *)wifi_cfg.ap.password, CONFIG_WIFI_AP_PASSWORD, sizeof(wifi_cfg.ap.password));
    wifi_cfg.ap.channel = CONFIG_WIFI_AP_CHANNEL;
    wifi_cfg.ap.max_connection = 1;
    wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    if (strlen(CONFIG_WIFI_AP_PASSWORD) < 8) wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;

#if CONFIG_WIFI_ENABLE_STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    strncpy((char *)wifi_cfg.sta.ssid,     CONFIG_WIFI_STA_SSID,     sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password, CONFIG_WIFI_STA_PASSWORD, sizeof(wifi_cfg.sta.password));
#else
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
#if CONFIG_WIFI_ENABLE_STA
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
#endif
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP SSID=%s password=%s ip=192.168.4.1",
             CONFIG_WIFI_AP_SSID, CONFIG_WIFI_AP_PASSWORD);
#if CONFIG_WIFI_ENABLE_STA
    ESP_LOGI(TAG, "STA  SSID=%s", CONFIG_WIFI_STA_SSID);
#endif
}

/* mDNS：让 PC 可以用 espcar.local 解析。IDF v6 已移除 mDNS 组件，
   PC 直接用 IP 192.168.4.1 连接即可 */

void wifi_debug_start(void)
{
    if (s_running) return;
    s_running = true;

    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_apsta();

    xTaskCreate(server_task, "tcp_server", 4096, NULL, 5, NULL);
}

bool wifi_debug_has_client(void)
{
    return s_client_fd >= 0;
}