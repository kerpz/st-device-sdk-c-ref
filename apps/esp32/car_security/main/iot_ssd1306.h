#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* Default display size */
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

/**
 * @brief Initialize SSD1306 display (includes I2C init)
 *
 * @param width   Display width (usually 128)
 * @param height  Display height (usually 64, must be multiple of 8)
 *
 * @return ESP_OK on success
 */
esp_err_t ssd1306_init(uint8_t width, uint8_t height);

/**
 * @brief Clear internal framebuffer (does NOT update display)
 */
void ssd1306_clear(void);

/**
 * @brief Set or clear a single pixel in framebuffer
 */
void ssd1306_pixel(uint8_t x, uint8_t y, bool on);

/**
 * @brief Flush framebuffer to OLED
 */
esp_err_t ssd1306_flush(void);
