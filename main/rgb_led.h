#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize the single WS2812 connected to CONFIG_BLINK_GPIO.
 */
esp_err_t rgb_init(void);

/**
 * @brief Set the WS2812 color. Components use normal RGB order (0-255).
 */
esp_err_t rgb_set(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Turn the WS2812 off.
 */
esp_err_t rgb_off(void);

/**
 * @brief Release the RMT LED strip object.
 */
esp_err_t rgb_deinit(void);
