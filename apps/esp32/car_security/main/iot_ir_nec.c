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
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

#include "iot_ir_nec.h"

static bool nec_decode(rmt_item32_t *items, int item_count, uint32_t *nec_code);
static bool nec_repeat(rmt_item32_t *items, int item_count);

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
    {                                  // Bit 1: ~1.69ms
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

void rmt_ir_rx_init()
{
  rmt_config_t rmt_rx_config = {
      .rmt_mode = RMT_MODE_RX,
      .channel = RMT_CHANNEL_0,
      .gpio_num = GPIO_IR_RX,
      .mem_block_num = 1,
      .clk_div = 80, // 1MHz
      .rx_config.filter_en = true,
      .rx_config.filter_ticks_thresh = 100,
      .rx_config.idle_threshold = 12000, // 12ms
  };
  rmt_config(&rmt_rx_config);
  rmt_driver_install(rmt_rx_config.channel, 1000, 0);

  // Start the IR RX task
  xTaskCreate(ir_rx_task, "ir_rx_task", 2048, NULL, 10, NULL);
}