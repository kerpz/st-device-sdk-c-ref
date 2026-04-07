#ifndef IOT_BEEP_H
#define IOT_BEEP_H
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
#include <stdint.h>
#include <stdbool.h>

// ================= CONFIG =================
#ifndef GPIO_BUZZER
#define GPIO_BUZZER 25
#endif

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT

void beep_setup(void);

// Control
void start_alarm(void);
void stop_alarm(void);

// Optional controls
void set_volume(uint32_t vol);

// Modes (optional direct use)
void siren_wail(void);
void siren_yelp(void);
void siren_hilo(void);
void siren_phaser(void);
void siren_airhorn(void);

#endif