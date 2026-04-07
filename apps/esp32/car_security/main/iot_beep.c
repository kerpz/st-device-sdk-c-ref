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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_random.h"

#include "iot_beep.h"

// ================= STATE =================
static uint32_t duty = 512; // 0–1023
static TaskHandle_t alarm_task_handle = NULL;
static bool alarm_running = false;

// ================= LOW LEVEL =================
static void set_freq(int freq)
{
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void set_volume(uint32_t vol)
{
    if (vol > 1023)
        vol = 1023;
    duty = vol;
}

// ================= SIREN MODES =================

// 🚨 WAIL (smooth sweep)
void siren_wail()
{
    for (int f = 700; f <= 1800 && alarm_running; f += (f < 1200 ? 8 : 20))
    {
        set_freq(f + (esp_random() % 10));
        vTaskDelay(pdMS_TO_TICKS(12));
    }

    for (int f = 1800; f >= 700 && alarm_running; f -= (f > 1200 ? 20 : 8))
    {
        set_freq(f + (esp_random() % 10));
        vTaskDelay(pdMS_TO_TICKS(12));
    }
}

// 🚨 YELP (fast sweep)
void siren_yelp()
{
    for (int f = 800; f <= 2000 && alarm_running; f += 40)
    {
        set_freq(f);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for (int f = 2000; f >= 800 && alarm_running; f -= 40)
    {
        set_freq(f);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// 🚨 HI-LO
void siren_hilo()
{
    set_freq(900);
    vTaskDelay(pdMS_TO_TICKS(250));

    set_freq(1400);
    vTaskDelay(pdMS_TO_TICKS(350));
}

// 🚨 PHASER
void siren_phaser()
{
    for (int f = 1000; f <= 2000 && alarm_running; f += 80)
    {
        set_freq(f);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    for (int f = 2000; f >= 1000 && alarm_running; f -= 80)
    {
        set_freq(f);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// 🚨 AIRHORN
void siren_airhorn()
{
    set_freq(1000);
    vTaskDelay(pdMS_TO_TICKS(80));

    set_freq(1400);
    vTaskDelay(pdMS_TO_TICKS(120));
}

// ================= TASK =================
static void alarm_task(void *arg)
{
    while (alarm_running)
    {
        // Sequence
        for (int i = 0; i < 2 && alarm_running; i++)
            siren_wail();

        for (int i = 0; i < 6 && alarm_running; i++)
            siren_yelp();

        for (int i = 0; i < 10 && alarm_running; i++)
            siren_phaser();

        for (int i = 0; i < 8 && alarm_running; i++)
            siren_hilo();
    }

    // Ensure silent when exiting
    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
    alarm_task_handle = NULL;
    vTaskDelete(NULL);
}

// ================= CONTROL =================
void start_alarm(void)
{
    if (alarm_running)
        return;

    alarm_running = true;
    xTaskCreate(alarm_task, "alarm_task", 2048, NULL, 5, &alarm_task_handle);
}

void stop_alarm(void)
{
    alarm_running = false;

    if (alarm_task_handle)
    {
        vTaskDelay(pdMS_TO_TICKS(50)); // allow task to exit
    }

    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
}

// ================= INIT =================
void beep_setup(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = GPIO_BUZZER,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&channel);
}