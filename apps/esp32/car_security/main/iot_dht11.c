#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "rom/ets_sys.h"

#include "iot_dht11.h"

static int _waitOrTimeout(uint16_t microSeconds, int level)
{
  int micros_ticks = 0;
  while (gpio_get_level(GPIO_DHT11) == level)
  {
    if (micros_ticks++ > microSeconds)
      return DHT11_TIMEOUT_ERROR;
    ets_delay_us(1);
  }
  return micros_ticks;
}

static int _checkResponse()
{
  /* Wait for next step ~80us*/
  if (_waitOrTimeout(80, 0) == DHT11_TIMEOUT_ERROR)
    return DHT11_TIMEOUT_ERROR;

  /* Wait for next step ~80us*/
  if (_waitOrTimeout(80, 1) == DHT11_TIMEOUT_ERROR)
    return DHT11_TIMEOUT_ERROR;

  return DHT11_OK;
}

int get_dht11_readings(float *temperature, float *humidity)
{
  int data[5] = {0};

  gpio_set_direction(GPIO_DHT11, GPIO_MODE_OUTPUT);
  // Wake up device, 250ms of high
  // gpio_set_level(GPIO_DHT11, 1);
  // ets_delay_us(250000);
  // Hold low for 20ms
  gpio_set_level(GPIO_DHT11, 0);
  ets_delay_us(20000);
  // High for 40ns
  gpio_set_level(GPIO_DHT11, 1);
  ets_delay_us(40);
  // Set DHT_PIN pin as an input
  gpio_set_direction(GPIO_DHT11, GPIO_MODE_INPUT);

  if (_checkResponse() == DHT11_TIMEOUT_ERROR)
  {
    printf("dht11 timeout.");
    return false;
  }

  /* Read response */
  for (int i = 0; i < 40; i++)
  {
    /* Initial data */
    if (_waitOrTimeout(50, 0) == DHT11_TIMEOUT_ERROR)
    {
      printf("dht11 timeout.");
      return false;
    }

    if (_waitOrTimeout(70, 1) > 28)
    {
      /* Bit received was a 1 */
      data[i / 8] |= (1 << (7 - (i % 8)));
    }
  }

  if (data[4] == (data[0] + data[1] + data[2] + data[3])) // checksum
  {
    *temperature = (float)data[2];
    *humidity = (float)data[0];
    printf("Temperature =  %d *C, Humidity = %d %%\r\n", (int)data[2], (int)data[0]);
    return true;
  }
  else
    printf("dht11 checksum error.");
  return false;
}
// https://github.com/Anacron-sec/esp32-DHT11/blob/master/dht11.c