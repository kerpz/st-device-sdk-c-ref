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
#ifndef _IOT_ADC_H_
#define _IOT_ADC_H_

/* GPIO36 */
#define ADC_TARGET ADC_UNIT_1     // ADC1
#define ADC_CHANNEL ADC_CHANNEL_0 // Channel 0

void adc_setup(void);
int get_adc_readings(float *voltage);

#endif /* _IOT_ADC_H_ */