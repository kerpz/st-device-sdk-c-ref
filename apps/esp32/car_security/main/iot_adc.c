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
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "iot_adc.h"

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;

void adc_setup(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = DEFAULT_ADC_TARGET,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, DEFAULT_ADC_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = DEFAULT_ADC_TARGET,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));
}

bool get_adc_readings(float *voltage)
{
    int adc_value = 0;
    int mv_output = 0;

    // read ADC
    esp_err_t ret = adc_oneshot_read(adc_handle, DEFAULT_ADC_CHANNEL, &adc_value);
    if (ret != ESP_OK)
    {
        printf("ADC read failed: %d\n", ret);
        return false;
    }

    // convert raw value to voltage
    adc_cali_raw_to_voltage(cali_handle, adc_value, &mv_output);

    *voltage = mv_output / 1000.0f;
    printf("ADC raw = %d, Voltage = %.3f V\n", adc_value, *voltage);

    return true;
}
