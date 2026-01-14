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

#include "device_control.h"
#include "iot_adc.h"
#include "iot_ir_nec.h"
#include "iot_ssd1306.h"

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

void change_lock_state(int lock_state)
{
    if (lock_state == SWITCH_OFF)
    {
        gpio_set_level(GPIO_OUTPUT_MAINLED, MAINLED_GPIO_OFF);
    }
    else
    {
        gpio_set_level(GPIO_OUTPUT_MAINLED, MAINLED_GPIO_ON);
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
        .master.clk_speed = 400000, // 100000,
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
    ssd1306_init(SSD1306_WIDTH, SSD1306_HEIGHT);
    ssd1306_clear();

    // Draw diagonal line
    // for (int i = 0; i < 64; i++)
    //    ssd1306_pixel(i, i, true);
    ssd1306_text(0, 0, "HELLO WORLD", true);
    ssd1306_text(0, 16, "ESP32 SSD1306", true);

    ssd1306_flush();

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}