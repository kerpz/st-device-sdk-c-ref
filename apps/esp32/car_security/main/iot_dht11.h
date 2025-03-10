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
#ifndef _IOT_DHT11_H_
#define _IOT_DHT11_H_

#define GPIO_DHT11 19

enum dht11_status
{
  DHT11_CRC_ERROR = -2,
  DHT11_TIMEOUT_ERROR,
  DHT11_OK
};

int get_dht11_readings(float *temperature, float *humidity);

#endif /* _IOT_DHT11_H_ */