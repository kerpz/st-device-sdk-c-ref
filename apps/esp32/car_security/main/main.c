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

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "st_dev.h"
#include "device_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_wifi.h"

#include "iot_uart_cli.h"
#include "iot_cli_cmd.h"
#include "iot_ota.h"
#include "iot_dht11.h"
#include "iot_adc.h"
#include "iot_ssd1306.h"
#include "iot_beep.h"

#include "caps_lock.h"
#include "caps_voltageMeasurement.h"
#include "caps_temperatureMeasurement.h"
#include "caps_relativeHumidityMeasurement.h"
#include "caps_firmwareUpdate.h"
#include "caps_motionSensor.h"
#include "caps_alarm.h"
#include "caps_contactSensor.h"
// #include "caps_signalStrength.h"

// onboarding_config_start is null-terminated string
extern const uint8_t onboarding_config_start[] asm("_binary_onboarding_config_json_start");
extern const uint8_t onboarding_config_end[] asm("_binary_onboarding_config_json_end");

// device_info_start is null-terminated string
extern const uint8_t device_info_start[] asm("_binary_device_info_json_start");
extern const uint8_t device_info_end[] asm("_binary_device_info_json_end");

static iot_status_t g_iot_status = IOT_STATUS_IDLE;
static iot_stat_lv_t g_iot_stat_lv;

IOT_CTX *iot_ctx = NULL;

// #define SET_PIN_NUMBER_CONFRIM

static int noti_led_mode = LED_ANIMATION_MODE_IDLE;

static caps_lock_data_t *cap_lock_data;
static caps_voltageMeasurement_data_t *cap_voltage_data;
static caps_temperatureMeasurement_data_t *cap_temperature_data;
static caps_relativeHumidityMeasurement_data_t *cap_humidity_data;
static caps_firmwareUpdate_data_t *cap_ota_data;
static caps_motionSensor_data_t *cap_motion_data;
static caps_alarm_data_t *cap_alarm_data;
static caps_contactSensor_data_t *cap_door_data;
// static caps_signalStrength_data_t *cap_signalStrength_data;

TaskHandle_t ota_task_handle = NULL;

int monitor_enable = true;
int monitor_period_ms = 60000; // 1 minute

float voltage = -1.0;
float temperature = -1.0;
float humidity = -1.0;

static int get_lock_state(void)
{
    const char *lock_value = cap_lock_data->get_lock_value(cap_lock_data);
    int lock_state = SWITCH_OFF;

    if (!lock_value)
    {
        return -1;
    }

    if (!strcmp(lock_value, caps_helper_lock.attr_lock.value_locked))
    {
        lock_state = SWITCH_ON;
    }
    else if (!strcmp(lock_value, caps_helper_lock.attr_lock.value_unlocked))
    {
        lock_state = SWITCH_OFF;
    }
    return lock_state;
}

static void cap_lock_cmd_cb(struct caps_lock_data *caps_data)
{
    int lock_state = get_lock_state();
    change_lock_state(lock_state);
}

static void cap_alarm_siren_cmd_cb(struct caps_alarm_data *caps_data)
{
    // Handle alarm command callback
    start_alarm();
}
static void cap_alarm_off_cmd_cb(struct caps_alarm_data *caps_data)
{
    // Handle alarm command callback
    stop_alarm();
}

static char *get_current_firmware_version(void)
{
    char *current_version = NULL;

    unsigned char *device_info = (unsigned char *)device_info_start;
    unsigned int device_info_len = device_info_end - device_info_start;

    ota_err_t err = ota_api_get_firmware_version_load(device_info, device_info_len, &current_version);
    if (err != OTA_OK)
    {
        printf("ota_api_get_firmware_version_load is failed : %d\n", err);
    }

    return current_version;
}

#define OTA_UPDATE_MAX_RETRY_COUNT 100

static void ota_update_task(void *pvParameter)
{
    printf("\n Starting OTA...\n");

    static int count = 0;

    while (1)
    {

        ota_err_t ret = ota_update_device();
        if (ret != OTA_OK)
        {
            printf("Firmware Upgrades Failed (%d) \n", ret);
            vTaskDelay(600 * 1000 / portTICK_PERIOD_MS);
            count++;
        }
        else
        {
            break;
        }

        if (count > OTA_UPDATE_MAX_RETRY_COUNT)
            break;
    }

    printf("Prepare to restart system!");
    ota_restart_device();
}

void ota_polling_task(void *arg)
{
    while (1)
    {

        vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);

        if (g_iot_status != IOT_STATUS_CONNECTING || g_iot_stat_lv != IOT_STAT_LV_DONE)
        {
            continue;
        }

        if (ota_task_handle != NULL)
        {
            printf("Device is updating.. \n");
            continue;
        }

        ota_check_for_update((void *)arg);

        /* Set polling period */
        unsigned int polling_day = ota_get_polling_period_day();
        unsigned int task_delay_sec = polling_day * 24 * 3600;
        vTaskDelay(task_delay_sec * 1000 / portTICK_PERIOD_MS);
    }
}

static void cap_update_cmd_cb(struct caps_firmwareUpdate_data *caps_data)
{
    ota_nvs_flash_init();

    xTaskCreate(&ota_update_task, "ota_update_task", 8096, NULL, 5, &ota_task_handle);
}

static void capability_init()
{
    cap_lock_data = caps_lock_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_lock_data)
    {
        const char *lock_init_value = caps_helper_lock.attr_lock.value_locked;

        cap_lock_data->cmd_lock_usr_cb = cap_lock_cmd_cb;
        cap_lock_data->cmd_unlock_usr_cb = cap_lock_cmd_cb;

        cap_lock_data->set_lock_value(cap_lock_data, lock_init_value);
    }

    cap_voltage_data = caps_voltageMeasurement_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_voltage_data)
    {
        cap_voltage_data->set_voltage_unit(cap_voltage_data, "V");
        cap_voltage_data->set_voltage_value(cap_voltage_data, voltage);
        cap_voltage_data->attr_voltage_send(cap_voltage_data);
    }

    cap_temperature_data = caps_temperatureMeasurement_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_temperature_data)
    {
        cap_temperature_data->set_temperature_unit(cap_temperature_data, "C");
        cap_temperature_data->set_temperature_value(cap_temperature_data, temperature);
        cap_temperature_data->attr_temperature_send(cap_temperature_data);
    }

    cap_humidity_data = caps_relativeHumidityMeasurement_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_humidity_data)
    {
        // cap_humidity_data->set_temperature_unit(cap_humidity_data, "%%");
        cap_humidity_data->set_humidity_value(cap_humidity_data, humidity);
        cap_humidity_data->attr_humidity_send(cap_humidity_data);
    }

    cap_ota_data = caps_firmwareUpdate_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_ota_data)
    {
        char *firmware_version = get_current_firmware_version();

        cap_ota_data->set_currentVersion_value(cap_ota_data, firmware_version);
        cap_ota_data->cmd_updateFirmware_usr_cb = cap_update_cmd_cb;

        free(firmware_version);
    }

    cap_motion_data = caps_motionSensor_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_motion_data)
    {
        const char *motion_init_value = "inactive";
        cap_motion_data->set_motion_value(cap_motion_data, motion_init_value);
    }

    cap_alarm_data = caps_alarm_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_alarm_data)
    {
        const char *alarm_init_value = "off";
        cap_alarm_data->set_alarm_value(cap_alarm_data, alarm_init_value);
        cap_alarm_data->cmd_siren_usr_cb = cap_alarm_siren_cmd_cb;
        cap_alarm_data->cmd_off_usr_cb = cap_alarm_off_cmd_cb;
    }

    cap_door_data = caps_contactSensor_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_door_data)
    {
        const char *door_init_value = "open";
        cap_door_data->set_contact_value(cap_door_data, door_init_value);
    }

    /*
    cap_signalStrength_data = caps_signalStrength_initialize(iot_ctx, "main", NULL, NULL);
    if (cap_signalStrength_data)
    {
        cap_signalStrength_data->set_rssi_unit(cap_signalStrength_data, "dBm");
        cap_signalStrength_data->set_rssi_value(cap_signalStrength_data, -50.0);
        cap_signalStrength_data->attr_rssi_send(cap_signalStrength_data);
        cap_signalStrength_data->set_lqi_value(cap_signalStrength_data, 100);
        cap_signalStrength_data->attr_lqi_send(cap_signalStrength_data);
    }
    */
}

static void iot_status_cb(iot_status_t status,
                          iot_stat_lv_t stat_lv, void *usr_data)
{
    g_iot_status = status;
    g_iot_stat_lv = stat_lv;

    printf("status: %d, stat: %d\n", g_iot_status, g_iot_stat_lv);

    switch (status)
    {
    case IOT_STATUS_NEED_INTERACT:
        noti_led_mode = LED_ANIMATION_MODE_FAST;
        break;
    case IOT_STATUS_IDLE:
    case IOT_STATUS_CONNECTING:
        noti_led_mode = LED_ANIMATION_MODE_IDLE;
        // change_switch_state(get_switch_state());
        change_lock_state(get_lock_state());
        break;
    default:
        break;
    }
}

#if defined(SET_PIN_NUMBER_CONFRIM)
void *pin_num_memcpy(void *dest, const void *src, unsigned int count)
{
    unsigned int i;
    for (i = 0; i < count; i++)
        *((char *)dest + i) = *((char *)src + i);
    return dest;
}
#endif

static void connection_start(void)
{
    iot_pin_t *pin_num = NULL;
    int err;

#if defined(SET_PIN_NUMBER_CONFRIM)
    pin_num = (iot_pin_t *)malloc(sizeof(iot_pin_t));
    if (!pin_num)
        printf("failed to malloc for iot_pin_t\n");

    // to decide the pin confirmation number(ex. "12345678"). It will use for easysetup.
    //    pin confirmation number must be 8 digit number.
    pin_num_memcpy(pin_num, "12345678", sizeof(iot_pin_t));
#endif

    // process on-boarding procedure. There is nothing more to do on the app side than call the API.
    err = st_conn_start(iot_ctx, (st_status_cb)&iot_status_cb, IOT_STATUS_ALL, NULL, pin_num);
    if (err)
    {
        printf("fail to start connection. err:%d\n", err);
    }
    if (pin_num)
    {
        free(pin_num);
    }
}

static void connection_start_task(void *arg)
{
    connection_start();
    vTaskDelete(NULL);
}

static void iot_noti_cb(iot_noti_data_t *noti_data, void *noti_usr_data)
{
    printf("Notification message received\n");

    if (noti_data->type == IOT_NOTI_TYPE_DEV_DELETED)
    {
        printf("[device deleted]\n");
    }
    else if (noti_data->type == IOT_NOTI_TYPE_RATE_LIMIT)
    {
        printf("[rate limit] Remaining time:%d, sequence number:%d\n",
               noti_data->raw.rate_limit.remainingTime, noti_data->raw.rate_limit.sequenceNumber);
    }
}

void button_event(IOT_CAP_HANDLE *handle, int type, int count)
{
    if (type == BUTTON_SHORT_PRESS)
    {
        printf("Button short press, count: %d\n", count);
        switch (count)
        {
        case 1:
            if (g_iot_status == IOT_STATUS_NEED_INTERACT)
            {
                st_conn_ownership_confirm(iot_ctx, true);
                noti_led_mode = LED_ANIMATION_MODE_IDLE;
                // change_switch_state(get_switch_state());
            }
            /*
            else
            {
                if (get_switch_state() == SWITCH_ON)
                {
                    change_switch_state(SWITCH_OFF);
                    cap_switch_data->set_switch_value(cap_switch_data, caps_helper_switch.attr_switch.value_off);
                    cap_switch_data->attr_switch_send(cap_switch_data);
                }
                else
                {
                    change_switch_state(SWITCH_ON);
                    cap_switch_data->set_switch_value(cap_switch_data, caps_helper_switch.attr_switch.value_on);
                    cap_switch_data->attr_switch_send(cap_switch_data);
                }
            }
            */
            break;
        case 2:
            monitor_enable = !monitor_enable;
            printf("change monitor mode to %d\n", monitor_enable);
            break;
        case 5:
            /* clean-up provisioning & registered data with reboot option*/
            st_conn_cleanup(iot_ctx, true);

            break;
        default:
            // led_blink(get_switch_state(), 100, count);
            break;
        }
    }
    else if (type == BUTTON_LONG_PRESS)
    {
        printf("Button long press, iot_status: %d\n", g_iot_status);
        // led_blink(get_switch_state(), 100, 3);
        st_conn_cleanup(iot_ctx, false);
        xTaskCreate(connection_start_task, "connection_task", 2048, NULL, 10, NULL);
    }
}

void sensor_callback(sensor_event_t event)
{
    switch (event)
    {

    case SENSOR_EVENT_PIR_ACTIVE:
        printf("🚨 Motion detected\n");
        cap_motion_data->set_motion_value(cap_motion_data, "active");
        cap_motion_data->attr_motion_send(cap_motion_data);
        break;

    case SENSOR_EVENT_PIR_INACTIVE:
        printf("No motion\n");
        cap_motion_data->set_motion_value(cap_motion_data, "inactive");
        cap_motion_data->attr_motion_send(cap_motion_data);
        break;

    case SENSOR_EVENT_DOOR_OPEN:
        // printf("🚪 Door OPEN\n");
        cap_door_data->set_contact_value(cap_door_data, "open");
        cap_door_data->attr_contact_send(cap_door_data);
        break;

    case SENSOR_EVENT_DOOR_CLOSE:
        // printf("🔒 Door CLOSED\n");
        cap_door_data->set_contact_value(cap_door_data, "closed");
        cap_door_data->attr_contact_send(cap_door_data);
        break;

    default:
        break;
    }
}

static void app_main_task(void *arg)
{
    IOT_CAP_HANDLE *handle = (IOT_CAP_HANDLE *)arg;

    int button_event_type;
    int button_event_count;

    TimeOut_t monitor_timeout;
    TickType_t monitor_period_tick = pdMS_TO_TICKS(monitor_period_ms);

    vTaskSetTimeOutState(&monitor_timeout);

    char buf[16];
    for (;;)
    {
        if (get_button_event(&button_event_type, &button_event_count))
        {
            button_event(handle, button_event_type, button_event_count);
        }
        if (noti_led_mode != LED_ANIMATION_MODE_IDLE)
        {
            change_led_mode(noti_led_mode);
        }

        if (monitor_enable && (xTaskCheckForTimeOut(&monitor_timeout, &monitor_period_tick) != pdFALSE))
        {
            vTaskSetTimeOutState(&monitor_timeout);
            monitor_period_tick = pdMS_TO_TICKS(monitor_period_ms);

            /* emulate sensor value for example */
            get_adc_readings(&voltage);
            cap_voltage_data->set_voltage_value(cap_voltage_data, voltage);
            cap_voltage_data->attr_voltage_send(cap_voltage_data);

            get_dht11_readings(&temperature, &humidity);
            cap_temperature_data->set_temperature_value(cap_temperature_data, temperature);
            cap_temperature_data->attr_temperature_send(cap_temperature_data);

            cap_humidity_data->set_humidity_value(cap_humidity_data, humidity);
            cap_humidity_data->attr_humidity_send(cap_humidity_data);

            // Update signal strength
            /*
            if (cap_signalStrength_data)
            {
                wifi_ap_record_t ap_info;
                if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
                {
                    cap_signalStrength_data->set_rssi_value(cap_signalStrength_data, ap_info.rssi);
                    cap_signalStrength_data->attr_rssi_send(cap_signalStrength_data);
                }
            }
            */

            cap_door_data->set_contact_value(cap_door_data, "closed");
            cap_door_data->attr_contact_send(cap_door_data);

            // cap_motion_data->set_motion_value(cap_motion_data, "active");
            // cap_motion_data->attr_motion_send(cap_motion_data);

            // display
            ssd1306_clear();
            // ----- LEFT COLUMN -----
            snprintf(buf, sizeof(buf), "T: %.1fC", temperature);
            ssd1306_text(0, 0, buf, true);

            int sig = 0;
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
            {
                sig = ap_info.rssi;
            }
            snprintf(buf, sizeof(buf), "S: %ddBm", sig);
            ssd1306_text(0, 16, buf, true);

            // ----- RIGHT COLUMN -----
            snprintf(buf, sizeof(buf), "H: %.0f%%", humidity);
            ssd1306_text(72, 0, buf, true);

            // float volt = get_voltage();
            snprintf(buf, sizeof(buf), "V: %.2fV", voltage);
            ssd1306_text(72, 16, buf, true);

            // ----- BOTTOM ROW: timestamp -----
            snprintf(buf, sizeof(buf), "UPDATED: %02d:%02d", 12, 3); // replace with RTC
            ssd1306_text(0, 32, buf, true);

            // Push framebuffer to OLED
            ssd1306_flush();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    /**
      SmartThings Device SDK(STDK) aims to make it easier to develop IoT devices by providing
      additional st_iot_core layer to the existing chip vendor SW Architecture.

      That is, you can simply develop a basic application
      by just calling the APIs provided by st_iot_core layer like below.

      // create a iot context
      1. st_conn_init();

      // create a handle to process capability
      2. st_cap_handle_init(); (called in function 'capability_init')

      // register a callback function to process capability command when it comes from the SmartThings Server.
      3. st_cap_cmd_set_cb(); (called in function 'capability_init')

      // process on-boarding procedure. There is nothing more to do on the app side than call the API.
      4. st_conn_start(); (called in function 'connection_start')
     */

    unsigned char *onboarding_config = (unsigned char *)onboarding_config_start;
    unsigned int onboarding_config_len = onboarding_config_end - onboarding_config_start;
    unsigned char *device_info = (unsigned char *)device_info_start;
    unsigned int device_info_len = device_info_end - device_info_start;

    int iot_err;

    iot_gpio_init();

    // create a iot context
    iot_ctx = st_conn_init(onboarding_config, onboarding_config_len, device_info, device_info_len);
    if (iot_ctx != NULL)
    {
        iot_err = st_conn_set_noti_cb(iot_ctx, iot_noti_cb, NULL);
        if (iot_err)
            printf("fail to set notification callback function\n");
    }
    else
    {
        printf("fail to create the iot_context\n");
    }

    // create a handle to process capability and initialize capability info
    capability_init();

    register_iot_cli_cmd();
    uart_cli_main();
    xTaskCreate(app_main_task, "app_main_task", 4096, NULL, 10, NULL);

    xTaskCreate(ota_polling_task, "ota_polling_task", 8096, (void *)cap_ota_data, 5, NULL);

    // connect to server
    connection_start();
}
