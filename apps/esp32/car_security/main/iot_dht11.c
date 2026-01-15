#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include <stdio.h>
#include <stdbool.h>

#include "iot_dht11.h"

static int _waitOrTimeout(uint32_t timeout_us, int level)
{
  uint32_t elapsed = 0;

  while (gpio_get_level(GPIO_DHT11) == level)
  {
    if (elapsed++ >= timeout_us)
    {
      return DHT11_TIMEOUT_ERROR;
    }
    ets_delay_us(1);
  }
  return elapsed;
}

static int _checkResponse(void)
{
  if (_waitOrTimeout(80, 0) == DHT11_TIMEOUT_ERROR)
    return DHT11_TIMEOUT_ERROR;

  if (_waitOrTimeout(80, 1) == DHT11_TIMEOUT_ERROR)
    return DHT11_TIMEOUT_ERROR;

  return DHT11_OK;
}

dht11_status_t get_dht11_readings(float *temperature, float *humidity)
{
  uint8_t data[5] = {0};

  /* Configure GPIO */
  gpio_set_direction(GPIO_DHT11, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GPIO_DHT11, GPIO_PULLUP_ONLY);

  /* Start signal */
  gpio_set_level(GPIO_DHT11, 0);
  ets_delay_us(20000); // 20ms LOW

  gpio_set_level(GPIO_DHT11, 1);
  ets_delay_us(40); // 20–40µs HIGH

  gpio_set_direction(GPIO_DHT11, GPIO_MODE_INPUT);

  if (_checkResponse() != DHT11_OK)
  {
    printf("DHT11 response timeout\n");
    return false;
  }

  /* Read 40 bits */
  for (int i = 0; i < 40; i++)
  {
    if (_waitOrTimeout(50, 0) == DHT11_TIMEOUT_ERROR)
      return false;

    int high_time = _waitOrTimeout(70, 1);
    if (high_time == DHT11_TIMEOUT_ERROR)
      return false;

    if (high_time > 28)
    {
      data[i / 8] |= (1 << (7 - (i % 8)));
    }
  }

  /* Checksum */
  uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
  if (data[4] != checksum)
  {
    printf("DHT11 checksum error\n");
    return false;
  }

  *humidity = (float)data[0];
  *temperature = (float)data[2];

  printf("Temperature: %d°C, Humidity: %d%%\n",
         data[2], data[0]);

  /* Return pin to idle */
  gpio_set_direction(GPIO_DHT11, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_DHT11, 1);

  return true;
}
