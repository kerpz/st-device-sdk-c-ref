/* ***************************************************************************
 *
 * Copyright 2025 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 * Author: Philip Bordado <p.bordado@samsung.com>
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "device_control.h"
#include "iot_adc.h"
#include "iot_ir_nec.h"

static esp_lcd_panel_handle_t oled_panel = NULL;

// Simple 8x8 font for basic characters
static const uint8_t font8x8[128][8] = {
    // Space
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // '!'
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18, 0x00},
    // '"'
    {0x6C, 0x6C, 0x6C, 0x00, 0x00, 0x00, 0x00, 0x00},
    // '#'
    {0x6C, 0x6C, 0xFE, 0x6C, 0xFE, 0x6C, 0x6C, 0x00},
    // '$'
    {0x18, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x18, 0x00},
    // '%'
    {0x00, 0x66, 0x6C, 0x18, 0x30, 0x66, 0x46, 0x00},
    // '&'
    {0x38, 0x6C, 0x6C, 0x38, 0x6D, 0x66, 0x3B, 0x00},
    // '''
    {0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00},
    // '('
    {0x0C, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0C, 0x00},
    // ')'
    {0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x00},
    // '*'
    {0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00},
    // '+'
    {0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x00, 0x00},
    // ','
    {0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30, 0x00},
    // '-'
    {0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00},
    // '.'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00},
    // '/'
    {0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x80, 0x00},
    // '0'
    {0x3C, 0x66, 0x6E, 0x7E, 0x76, 0x66, 0x3C, 0x00},
    // '1'
    {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00},
    // '2'
    {0x3C, 0x66, 0x06, 0x1C, 0x30, 0x60, 0x7E, 0x00},
    // '3'
    {0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00},
    // '4'
    {0x0C, 0x1C, 0x3C, 0x6C, 0x7E, 0x0C, 0x0C, 0x00},
    // '5'
    {0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00},
    // '6'
    {0x1C, 0x30, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00},
    // '7'
    {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x00},
    // '8'
    {0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00},
    // '9'
    {0x3C, 0x66, 0x66, 0x3E, 0x06, 0x0C, 0x38, 0x00},
    // ':'
    {0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00},
    // ';'
    {0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x30, 0x00},
    // '<'
    {0x0C, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0C, 0x00},
    // '='
    {0x00, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00, 0x00},
    // '>'
    {0x30, 0x18, 0x0C, 0x06, 0x0C, 0x18, 0x30, 0x00},
    // '?'
    {0x3C, 0x66, 0x06, 0x1C, 0x18, 0x00, 0x18, 0x00},
    // '@'
    {0x3C, 0x66, 0x6E, 0x6E, 0x60, 0x66, 0x3C, 0x00},
    // 'A'
    {0x3C, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00},
    // 'B'
    {0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00},
    // 'C'
    {0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00},
    // 'D'
    {0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00},
    // 'E'
    {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x7E, 0x00},
    // 'F'
    {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x00},
    // 'G'
    {0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3C, 0x00},
    // 'H'
    {0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00},
    // 'I'
    {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00},
    // 'J'
    {0x3E, 0x0C, 0x0C, 0x0C, 0x0C, 0x6C, 0x38, 0x00},
    // 'K'
    {0x66, 0x6C, 0x78, 0x70, 0x78, 0x6C, 0x66, 0x00},
    // 'L'
    {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00},
    // 'M'
    {0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63, 0x00},
    // 'N'
    {0x66, 0x76, 0x7E, 0x7E, 0x6E, 0x66, 0x66, 0x00},
    // 'O'
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00},
    // 'P'
    {0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00},
    // 'Q'
    {0x3C, 0x66, 0x66, 0x66, 0x6E, 0x66, 0x3E, 0x00},
    // 'R'
    {0x7C, 0x66, 0x66, 0x7C, 0x78, 0x6C, 0x66, 0x00},
    // 'S'
    {0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0x00},
    // 'T'
    {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00},
    // 'U'
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00},
    // 'V'
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00},
    // 'W'
    {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00},
    // 'X'
    {0x66, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x66, 0x00},
    // 'Y'
    {0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x00},
    // 'Z'
    {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00},
    // '['
    {0x3C, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3C, 0x00},
    // '\'
    {0x80, 0xC0, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x00},
    // ']'
    {0x3C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x3C, 0x00},
    // '^'
    {0x18, 0x3C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00},
    // '_'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00},
    // '`'
    {0x30, 0x18, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00},
    // 'a'
    {0x00, 0x00, 0x3C, 0x06, 0x3E, 0x66, 0x3E, 0x00},
    // 'b'
    {0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x7C, 0x00},
    // 'c'
    {0x00, 0x00, 0x3C, 0x66, 0x60, 0x66, 0x3C, 0x00},
    // 'd'
    {0x06, 0x06, 0x3E, 0x66, 0x66, 0x66, 0x3E, 0x00},
    // 'e'
    {0x00, 0x00, 0x3C, 0x66, 0x7E, 0x60, 0x3C, 0x00},
    // 'f'
    {0x1C, 0x30, 0x30, 0x7C, 0x30, 0x30, 0x30, 0x00},
    // 'g'
    {0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x3C},
    // 'h'
    {0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00},
    // 'i'
    {0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x3C, 0x00},
    // 'j'
    {0x0C, 0x00, 0x1C, 0x0C, 0x0C, 0x0C, 0x6C, 0x38},
    // 'k'
    {0x60, 0x60, 0x66, 0x6C, 0x78, 0x6C, 0x66, 0x00},
    // 'l'
    {0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00},
    // 'm'
    {0x00, 0x00, 0x6C, 0x7E, 0x6B, 0x63, 0x63, 0x00},
    // 'n'
    {0x00, 0x00, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00},
    // 'o'
    {0x00, 0x00, 0x3C, 0x66, 0x66, 0x66, 0x3C, 0x00},
    // 'p'
    {0x00, 0x00, 0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60},
    // 'q'
    {0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x06},
    // 'r'
    {0x00, 0x00, 0x6E, 0x70, 0x60, 0x60, 0x60, 0x00},
    // 's'
    {0x00, 0x00, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x00},
    // 't'
    {0x30, 0x30, 0x7C, 0x30, 0x30, 0x30, 0x1C, 0x00},
    // 'u'
    {0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3E, 0x00},
    // 'v'
    {0x00, 0x00, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00},
    // 'w'
    {0x00, 0x00, 0x63, 0x63, 0x6B, 0x7F, 0x36, 0x00},
    // 'x'
    {0x00, 0x00, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x00},
    // 'y'
    {0x00, 0x00, 0x66, 0x66, 0x66, 0x3E, 0x06, 0x3C},
    // 'z'
    {0x00, 0x00, 0x7E, 0x0C, 0x18, 0x30, 0x7E, 0x00},
    // '{'
    {0x0E, 0x18, 0x18, 0x70, 0x18, 0x18, 0x0E, 0x00},
    // '|'
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00},
    // '}'
    {0x70, 0x18, 0x18, 0x0E, 0x18, 0x18, 0x70, 0x00},
    // '~'
    {0x00, 0x00, 0x00, 0x62, 0x66, 0x0C, 0x00, 0x00},
};

void i2c_scanner(void)
{
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 0; addr < 128; addr++)
    {
        if (addr % 16 == 0 && addr != 0)
        {
            printf("\n%02x: ", addr);
        }

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            printf("%02x ", addr);
        }
        else
        {
            printf("-- ");
        }
    }
    printf("\nI2C scan complete\n");
}

void oled_init(void)
{
    uint8_t oled_addresses[] = {0x3C, 0x3D};
    bool oled_found = false;

    for (int i = 0; i < sizeof(oled_addresses) && !oled_found; i++)
    {
        printf("Trying OLED at address 0x%02X\n", oled_addresses[i]);

        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = oled_addresses[i],
            .control_phase_bytes = 1, // 1 byte for control
            .dc_bit_offset = 0,       // D/C bit is bit 0 (0=command, 1=data)
            .flags = {
                .disable_control_phase = 0, // Enable control phase
            },
            .on_color_trans_done = NULL,
            .user_ctx = NULL,
        };

        esp_err_t ret = esp_lcd_new_panel_io_i2c(I2C_NUM_0, &io_config, &io_handle);
        if (ret != ESP_OK)
        {
            printf("Failed to create LCD panel IO at 0x%02X: %d\n", oled_addresses[i], ret);
            continue;
        }

        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = -1,
            .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
            .bits_per_pixel = 1,
        };

        ret = esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &oled_panel);
        if (ret != ESP_OK)
        {
            printf("Failed to create SSD1306 panel at 0x%02X: %d\n", oled_addresses[i], ret);
            continue;
        }

        ret = esp_lcd_panel_reset(oled_panel);
        if (ret != ESP_OK)
        {
            printf("Failed to reset OLED panel at 0x%02X: %d\n", oled_addresses[i], ret);
            continue;
        }

        ret = esp_lcd_panel_init(oled_panel);
        if (ret != ESP_OK)
        {
            printf("Failed to init OLED panel at 0x%02X: %d\n", oled_addresses[i], ret);
            continue;
        }

        ret = esp_lcd_panel_disp_on_off(oled_panel, true);
        if (ret != ESP_OK)
        {
            printf("Failed to turn on OLED display at 0x%02X: %d\n", oled_addresses[i], ret);
            continue;
        }

        // Try to mirror the display to see if that helps visibility
        ret = esp_lcd_panel_mirror(oled_panel, false, false);
        if (ret != ESP_OK)
        {
            printf("Failed to set OLED mirror: %d\n", ret);
        }

        // Try inverting colors to make display visible
        ret = esp_lcd_panel_invert_color(oled_panel, true);
        if (ret != ESP_OK)
        {
            printf("Failed to invert OLED colors: %d\n", ret);
        }

        printf("OLED initialized successfully at address 0x%02X\n", oled_addresses[i]);
        oled_found = true;

        // Test the display with a pattern - alternating vertical stripes
        uint8_t test_buffer[1024];
        for (int y = 0; y < 8; y++)
        { // 8 pages of 8 rows each
            for (int x = 0; x < 128; x++)
            {
                // Create vertical stripes - alternate every 8 pixels
                test_buffer[y * 128 + x] = (x / 8) % 2 ? 0xFF : 0x00;
            }
        }
        printf("Drawing test pattern (vertical stripes)...\n");
        ret = esp_lcd_panel_draw_bitmap(oled_panel, 0, 0, 128, 64, test_buffer);
        if (ret != ESP_OK)
        {
            printf("Failed to draw test pattern: %d\n", ret);
        }
        else
        {
            printf("Test pattern drawn successfully\n");
        }
        vTaskDelay(pdMS_TO_TICKS(3000)); // Show test pattern for 3 seconds

        // Clear the display
        memset(test_buffer, 0x00, 1024); // All pixels off
        esp_lcd_panel_draw_bitmap(oled_panel, 0, 0, 128, 64, test_buffer);
        vTaskDelay(pdMS_TO_TICKS(500)); // Brief pause
    }

    if (!oled_found)
    {
        printf("No OLED found at addresses 0x3C or 0x3D\n");
    }
}

void oled_display_text(const char *text, int line)
{
    if (oled_panel == NULL || text == NULL)
        return;

    esp_err_t ret;

    // Create a buffer for the display (128x64 monochrome = 1024 bytes)
    uint8_t buffer[1024] = {0};

    // Calculate starting Y position based on line (each line is 8 pixels high)
    int start_y = line * 8;
    if (start_y >= 64)
        start_y = 56; // Ensure we don't go beyond display height

    // Draw each character
    int x_pos = 0;
    for (int i = 0; text[i] != '\0' && x_pos < 128; i++)
    {
        char c = text[i];
        if (c < 0 || c >= 128)
            c = '?'; // Replace invalid chars with ?

        // Get font data for this character (8 bytes, each byte is 8 vertical pixels)
        const uint8_t *char_bitmap = font8x8[(uint8_t)c];

        // Draw the 8x8 character bitmap to the buffer
        // SSD1306 bitmap format: each byte = 8 vertical pixels in a column
        for (int char_x = 0; char_x < 8 && x_pos + char_x < 128; char_x++)
        {
            // For each column of the character
            uint8_t column_data = char_bitmap[char_x];

            // Set the bits in the display buffer
            for (int bit = 0; bit < 8; bit++)
            {
                if (column_data & (1 << bit))
                {
                    int pixel_x = x_pos + char_x;
                    int pixel_y = start_y + bit;

                    if (pixel_x < 128 && pixel_y < 64)
                    {
                        // Calculate which byte and bit position in the buffer
                        int byte_index = (pixel_y / 8) * 128 + pixel_x;
                        int bit_position = pixel_y % 8;

                        if (byte_index < 1024)
                        {
                            buffer[byte_index] |= (1 << bit_position);
                        }
                    }
                }
            }
        }

        // Move to next character position
        x_pos += 8;
    }

    // Draw the buffer to the OLED
    ret = esp_lcd_panel_draw_bitmap(oled_panel, 0, 0, 128, 64, buffer);
    if (ret != ESP_OK)
    {
        printf("Failed to draw text bitmap: %d\n", ret);
    }
    else
    {
        printf("Text bitmap drawn successfully\n");
    }

    // Also print to console for debugging
    printf("OLED Display: %s (line %d)\n", text, line);
}

void change_lock_state(int lock_state)
{
    if (lock_state == SWITCH_OFF)
    {
        gpio_set_level(GPIO_OUTPUT_MAINLED, MAINLED_GPIO_OFF);
        oled_display_text("LOCK: UNLOCKED", 1);
    }
    else
    {
        gpio_set_level(GPIO_OUTPUT_MAINLED, MAINLED_GPIO_ON);
        oled_display_text("LOCK: LOCKED", 1);
    }
}

int get_button_event(int *button_event_type, int *button_event_count)
{
    static uint32_t button_count = 0;
    static uint8_t button_last_state = BUTTON_GPIO_RELEASED;
    static TimeOut_t button_timeout;
    static TickType_t long_press_tick = pdMS_TO_TICKS(BUTTON_LONG_THRESHOLD_MS);
    static TickType_t button_delay_tick = pdMS_TO_TICKS(BUTTON_DELAY_MS);

    uint8_t gpio_level = 0;

    gpio_level = gpio_get_level(GPIO_INPUT_BUTTON);
    if (button_last_state != gpio_level)
    {
        /* wait debounce time to ignore small ripple of currunt */
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_TIME_MS));
        gpio_level = gpio_get_level(GPIO_INPUT_BUTTON);
        if (button_last_state != gpio_level)
        {
            printf("Button event, val: %d, tick: %lu\n", gpio_level, (uint32_t)xTaskGetTickCount());
            button_last_state = gpio_level;
            if (gpio_level == BUTTON_GPIO_PRESSED)
            {
                button_count++;
            }
            vTaskSetTimeOutState(&button_timeout);
            button_delay_tick = pdMS_TO_TICKS(BUTTON_DELAY_MS);
            long_press_tick = pdMS_TO_TICKS(BUTTON_LONG_THRESHOLD_MS);
        }
    }
    else if (button_count > 0)
    {
        if ((gpio_level == BUTTON_GPIO_PRESSED) && (xTaskCheckForTimeOut(&button_timeout, &long_press_tick) != pdFALSE))
        {
            *button_event_type = BUTTON_LONG_PRESS;
            *button_event_count = 1;
            button_count = 0;
            return true;
        }
        else if ((gpio_level == BUTTON_GPIO_RELEASED) && (xTaskCheckForTimeOut(&button_timeout, &button_delay_tick) != pdFALSE))
        {
            *button_event_type = BUTTON_SHORT_PRESS;
            *button_event_count = button_count;
            button_count = 0;
            return true;
        }
    }

    return false;
}

void led_blink(int switch_state, int delay, int count)
{
    for (int i = 0; i < count; i++)
    {
        vTaskDelay(delay / portTICK_PERIOD_MS);
        // change_switch_state(1 - switch_state);
        vTaskDelay(delay / portTICK_PERIOD_MS);
        // change_switch_state(switch_state);
    }
}

void change_led_mode(int noti_led_mode)
{
    static TimeOut_t led_timeout;
    static TickType_t led_tick = -1;
    static int last_led_mode = -1;
    static int led_state = SWITCH_OFF;

    if (last_led_mode != noti_led_mode)
    {
        last_led_mode = noti_led_mode;
        vTaskSetTimeOutState(&led_timeout);
        led_tick = 0;
    }

    switch (noti_led_mode)
    {
    case LED_ANIMATION_MODE_IDLE:
        break;
    case LED_ANIMATION_MODE_SLOW:
        if (xTaskCheckForTimeOut(&led_timeout, &led_tick) != pdFALSE)
        {
            led_state = 1 - led_state;
            // change_switch_state(led_state);
            vTaskSetTimeOutState(&led_timeout);
            if (led_state == SWITCH_ON)
            {
                led_tick = pdMS_TO_TICKS(200);
            }
            else
            {
                led_tick = pdMS_TO_TICKS(800);
            }
        }
        break;
    case LED_ANIMATION_MODE_FAST:
        if (xTaskCheckForTimeOut(&led_timeout, &led_tick) != pdFALSE)
        {
            led_state = 1 - led_state;
            // change_switch_state(led_state);
            vTaskSetTimeOutState(&led_timeout);
            led_tick = pdMS_TO_TICKS(100);
        }
        break;
    default:
        break;
    }
}

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

void iot_gpio_init(void)
{
    adc_setup();

    rmt_ir_rx_init();

    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;

    io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_MAINLED;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_MAINLED_0;
    // gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_NOUSE1;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_NOUSE2;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1 << GPIO_OUTPUT_ALARM;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;

    io_conf.pin_bit_mask = 1 << GPIO_INPUT_BUTTON;
    io_conf.pull_down_en = (BUTTON_GPIO_RELEASED == 0);
    io_conf.pull_up_en = (BUTTON_GPIO_RELEASED == 1);
    gpio_config(&io_conf);
    gpio_set_intr_type(GPIO_INPUT_BUTTON, GPIO_INTR_ANYEDGE);

    io_conf.pin_bit_mask = 1 << GPIO_INPUT_MOTION;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_set_intr_type(GPIO_INPUT_MOTION, GPIO_INTR_ANYEDGE);

    io_conf.pin_bit_mask = 1 << GPIO_INPUT_DOOR;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_set_intr_type(GPIO_INPUT_DOOR, GPIO_INTR_NEGEDGE);

    // gpio_pad_select_gpio(GPIO_INPUT_DOOR);
    // gpio_set_direction(GPIO_INPUT_DOOR, GPIO_MODE_INPUT);
    // gpio_pulldown_en(GPIO_INPUT_DOOR);
    // gpio_pullup_dis(GPIO_INPUT_DOOR);
    // gpio_set_intr_type(GPIO_INPUT_DOOR, GPIO_INTR_POSEDGE);

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_INPUT_DOOR, gpio_isr_handler, (void *)GPIO_INPUT_DOOR);

    gpio_set_level(GPIO_OUTPUT_MAINLED, MAINLED_GPIO_ON);
    // gpio_set_level(GPIO_OUTPUT_MAINLED_0, 0);

    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK)
    {
        printf("I2C param config failed: %d\n", ret);
        return;
    }

    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK)
    {
        printf("I2C driver install failed: %d\n", ret);
        return;
    }

    printf("I2C driver installed successfully on SDA=%d, SCL=%d\n", GPIO_I2C_SDA, GPIO_I2C_SCL);

    // Small delay to ensure I2C is stable
    vTaskDelay(pdMS_TO_TICKS(10));

    // Scan I2C bus for devices
    i2c_scanner();

    // Initialize OLED
    oled_init();
    oled_display_text("Hello", 1);

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}