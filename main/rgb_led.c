#include "rgb_led.h"

#include <stdbool.h>
#include <stddef.h>

#include "led_strip.h"
#include "sdkconfig.h"

#define RGB_GPIO               CONFIG_BLINK_GPIO
#define RGB_LED_COUNT          1
#define RGB_RMT_RESOLUTION_HZ  (10 * 1000 * 1000)
#define RGB_RMT_MEM_SYMBOLS    64

static led_strip_handle_t s_rgb_strip;

esp_err_t rgb_init(void)
{
    if (s_rgb_strip != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_GPIO,
        .max_leds = RGB_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    const led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RGB_RMT_RESOLUTION_HZ,
        .mem_block_symbols = RGB_RMT_MEM_SYMBOLS,
        .flags = {
            .with_dma = false,
        },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_rgb_strip);
    if (err != ESP_OK) {
        s_rgb_strip = NULL;
        return err;
    }

    // Send an all-zero frame after power-up so the LED starts in a known state.
    err = led_strip_clear(s_rgb_strip);
    if (err != ESP_OK) {
        led_strip_del(s_rgb_strip);
        s_rgb_strip = NULL;
    }
    return err;
}

esp_err_t rgb_set(uint8_t red, uint8_t green, uint8_t blue)
{
    if (s_rgb_strip == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = led_strip_set_pixel(s_rgb_strip, 0, red, green, blue);
    if (err != ESP_OK) {
        return err;
    }

    return led_strip_refresh(s_rgb_strip);
}

esp_err_t rgb_off(void)
{
    if (s_rgb_strip == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return led_strip_clear(s_rgb_strip);
}

esp_err_t rgb_deinit(void)
{
    if (s_rgb_strip == NULL) {
        return ESP_OK;
    }

    esp_err_t err = led_strip_del(s_rgb_strip);
    if (err == ESP_OK) {
        s_rgb_strip = NULL;
    }
    return err;
}
