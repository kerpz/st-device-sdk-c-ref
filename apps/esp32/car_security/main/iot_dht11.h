#define GPIO_DHT11 18

enum dht11_status
{
  DHT11_CRC_ERROR = -2,
  DHT11_TIMEOUT_ERROR,
  DHT11_OK
};

int get_dht11_readings(float *temperature, float *humidity);