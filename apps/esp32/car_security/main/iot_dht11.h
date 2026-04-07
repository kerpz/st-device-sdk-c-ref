#ifndef _IOT_DHT11_H_
#define _IOT_DHT11_H_

#include <stdint.h>

#ifndef GPIO_DHT11
#define GPIO_DHT11 23
#endif

typedef enum
{
  DHT11_OK = 0,
  DHT11_TIMEOUT_ERROR = -1,
  DHT11_CRC_ERROR = -2
} dht11_status_t;

/**
 * @brief Read temperature and humidity from DHT11
 *
 * @param[out] temperature Temperature in Celsius
 * @param[out] humidity    Relative humidity in %
 *
 * @return dht11_status_t
 */
dht11_status_t get_dht11_readings(float *temperature, float *humidity);

#endif /* _IOT_DHT11_H_ */
