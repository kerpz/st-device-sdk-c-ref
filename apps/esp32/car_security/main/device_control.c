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

#include "device_control.h"
#include "iot_adc.h"

static bool nec_decode(rmt_item32_t *items, int item_count, uint32_t *nec_code);

static bool nec_repeat(rmt_item32_t *items, int item_count)
{
    if (item_count < 2)
        return false;
    // Repeat: 9ms high, 2.25ms low, 560us high
    if (items[0].duration0 < 8000 || items[0].duration0 > 10000)
        return false;
    if (items[0].duration1 < 2000 || items[0].duration1 > 2500)
        return false;
    if (item_count > 1 && (items[1].duration0 < 400 || items[1].duration0 > 700))
        return false;
    return true;
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

static void rmt_ir_rx_init()
{
    rmt_config_t rmt_rx_config = {
        .rmt_mode = RMT_MODE_RX,
        .channel = RMT_CHANNEL_0,
        .gpio_num = GPIO_INPUT_IR,
        .mem_block_num = 1,
        .clk_div = 80, // 1MHz
        .rx_config.filter_en = true,
        .rx_config.filter_ticks_thresh = 100,
        .rx_config.idle_threshold = 12000, // 12ms
    };
    rmt_config(&rmt_rx_config);
    rmt_driver_install(rmt_rx_config.channel, 1000, 0);
}

static bool nec_decode(rmt_item32_t *items, int item_count, uint32_t *nec_code)
{
    // NEC protocol: 9ms AGC, 4.5ms space, then 32 bits (8 addr, 8 ~addr, 8 cmd, 8 ~cmd)
    if (item_count < 34)
        return false; // Minimum items for NEC

    // Check AGC burst: ~9ms high
    if (items[0].duration0 < 8000 || items[0].duration0 > 10000)
        return false;
    // Check space: ~4.5ms low
    if (items[0].duration1 < 4000 || items[0].duration1 > 5000)
        return false;

    uint32_t code = 0;
    int bit_index = 0;
    for (int i = 1; i < item_count && bit_index < 32; i++)
    {
        // Check mark duration ~560us
        if (items[i].duration0 < 400 || items[i].duration0 > 700)
            continue; // Invalid mark
        uint32_t duration1 = items[i].duration1;
        if (duration1 > 1500 && duration1 < 1800)
        {                                    // Bit 1: ~1.69ms
            code |= (1 << (31 - bit_index)); // MSB first
        }
        else if (duration1 > 400 && duration1 < 700)
        { // Bit 0: ~0.56ms
          // code bit remains 0
        }
        else
        {
            return false; // Invalid space
        }
        bit_index++;
    }

    if (bit_index == 32)
    {
        *nec_code = code;
        return true;
    }
    return false;
}

static void ir_rx_task(void *arg)
{
    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handle(RMT_CHANNEL_0, &rb);
    rmt_rx_start(RMT_CHANNEL_0, 1);

    while (1)
    {
        size_t rx_size = 0;
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, portMAX_DELAY);
        if (item)
        {
            // Decode NEC IR data
            uint32_t nec_code = 0;
            int item_count = rx_size / sizeof(rmt_item32_t);
            static uint8_t last_addr = 0;
            static uint8_t last_cmd = 0;
            if (nec_decode(item, item_count, &nec_code))
            {
                uint8_t addr = (nec_code >> 24) & 0xFF;
                uint8_t cmd = (nec_code >> 8) & 0xFF;
                last_addr = addr;
                last_cmd = cmd;
                printf("NEC code: 0x%08" PRIx32 ", Addr: 0x%02X, Cmd: 0x%02X\n", nec_code, (unsigned int)addr, (unsigned int)cmd);
                // Handle specific commands here
                if (addr == 0x07)
                { // Samsung TV
                    switch (cmd)
                    {
                    case 0x02:
                        printf("Samsung Power\n");
                        break;
                    case 0x07:
                        printf("Samsung Volume Up\n");
                        break;
                    case 0x0B:
                        printf("Samsung Volume Down\n");
                        break;
                    case 0x12:
                        printf("Samsung Channel Up\n");
                        break;
                    case 0x10:
                        printf("Samsung Channel Down\n");
                        break;
                    case 0x0F:
                        printf("Samsung Mute\n");
                        break;
                    case 0x1A:
                        printf("Samsung Source\n");
                        break;
                    default:
                        printf("Unknown Samsung command: 0x%02X\n", cmd);
                        break;
                    }
                }
                else if (addr == 0x20)
                { // Hisense TV
                    switch (cmd)
                    {
                    case 0x0C:
                        printf("Hisense Power\n");
                        break;
                    case 0x10:
                        printf("Hisense Volume Up\n");
                        break;
                    case 0x11:
                        printf("Hisense Volume Down\n");
                        break;
                    case 0x20:
                        printf("Hisense Channel Up\n");
                        break;
                    case 0x21:
                        printf("Hisense Channel Down\n");
                        break;
                    case 0x0D:
                        printf("Hisense Mute\n");
                        break;
                    case 0x0F:
                        printf("Hisense Source\n");
                        break;
                    default:
                        printf("Unknown Hisense command: 0x%02X\n", cmd);
                        break;
                    }
                }
                else if (cmd == 0x12)
                { // Generic power button
                    printf("Power button pressed\n");
                }
            }
            else if (nec_repeat(item, item_count))
            {
                printf("NEC repeat - long press detected\n");
                // Handle long press with last command
                if (last_addr == 0x07)
                {
                    switch (last_cmd)
                    {
                    case 0x07:
                        printf("Samsung Volume Up (long press)\n");
                        break;
                    case 0x0B:
                        printf("Samsung Volume Down (long press)\n");
                        break;
                    // Add more as needed
                    default:
                        printf("Samsung long press: 0x%02X\n", last_cmd);
                        break;
                    }
                }
                else if (last_addr == 0x20)
                {
                    switch (last_cmd)
                    {
                    case 0x10:
                        printf("Hisense Volume Up (long press)\n");
                        break;
                    case 0x11:
                        printf("Hisense Volume Down (long press)\n");
                        break;
                    // Add more
                    default:
                        printf("Hisense long press: 0x%02X\n", last_cmd);
                        break;
                    }
                }
            }
            vRingbufferReturnItem(rb, (void *)item);
        }
    }
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            printf("GPIO[%" PRIu32 "] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (io_num == GPIO_INPUT_IR)
            {
                // Handle IR signal
                printf("IR signal detected on GPIO%d\n", GPIO_INPUT_IR);
                // Add IR decoding logic here
            }
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
    // start IR rx task
    xTaskCreate(ir_rx_task, "ir_rx_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_INPUT_DOOR, gpio_isr_handler, (void *)GPIO_INPUT_DOOR);

    gpio_set_level(GPIO_OUTPUT_MAINLED, MAINLED_GPIO_ON);
    // gpio_set_level(GPIO_OUTPUT_MAINLED_0, 0);

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}