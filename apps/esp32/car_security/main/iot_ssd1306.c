#include "iot_ssd1306.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "SSD1306"

/* ================= I2C CONFIG ================= */
#define SSD1306_I2C_PORT I2C_NUM_0
#define SSD1306_SDA_PIN GPIO_NUM_21
#define SSD1306_SCL_PIN GPIO_NUM_22
#define SSD1306_FREQ_HZ 400000
#define SSD1306_ADDR 0x3C
#define SSD1306_TIMEOUT_MS 100

/* ================= SSD1306 COMMANDS ================= */
#define OLED_CTRL_CMD 0x00
#define OLED_CTRL_DATA 0x40

#define OLED_DISPLAY_OFF 0xAE
#define OLED_DISPLAY_ON 0xAF
#define OLED_SET_MUX_RATIO 0xA8
#define OLED_SET_OFFSET 0xD3
#define OLED_SET_START_LINE 0x40
#define OLED_SET_CLK_DIV 0xD5
#define OLED_SET_CHARGE_PUMP 0x8D
#define OLED_SET_ADDR_MODE 0x20
#define OLED_SET_CONTRAST 0x81
#define OLED_DISPLAY_RAM 0xA4
#define OLED_NORMAL_DISPLAY 0xA6
#define OLED_SEG_REMAP 0xA1
#define OLED_COM_SCAN_DEC 0xC8
#define OLED_SET_COM_PINS 0xDA

#define OLED_PAGE_ADDR 0xB0
#define OLED_COL_LOW 0x00
#define OLED_COL_HIGH 0x10

/* ================= STRUCTURES ================= */
typedef struct
{
  uint8_t *data;
} ssd1306_page_t;

typedef struct
{
  uint8_t width;
  uint8_t height;
  uint8_t pages;
  ssd1306_page_t *page;
} ssd1306_t;

static ssd1306_t oled;

/* ================= LOW LEVEL I2C ================= */
static esp_err_t ssd1306_i2c_write(const uint8_t *data, size_t len)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, (uint8_t *)data, len, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(
      SSD1306_I2C_PORT,
      cmd,
      pdMS_TO_TICKS(SSD1306_TIMEOUT_MS));

  i2c_cmd_link_delete(cmd);
  return ret;
}

/* ================= I2C INIT ================= */
/*
static esp_err_t ssd1306_i2c_init(void)
{
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = SSD1306_SDA_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = SSD1306_SCL_PIN,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = SSD1306_FREQ_HZ};

  ESP_ERROR_CHECK(i2c_param_config(SSD1306_I2C_PORT, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(SSD1306_I2C_PORT, conf.mode, 0, 0, 0));
  return ESP_OK;
}
  */

/* ================= INIT ================= */
esp_err_t ssd1306_init(uint8_t width, uint8_t height)
{
  if (height % 8 != 0)
    return ESP_ERR_INVALID_ARG;

  ESP_LOGI(TAG, "SSD1306 init %dx%d", width, height);

  // ESP_ERROR_CHECK(ssd1306_i2c_init());

  oled.width = width;
  oled.height = height;
  oled.pages = height / 8;

  oled.page = calloc(oled.pages, sizeof(ssd1306_page_t));
  if (!oled.page)
    return ESP_ERR_NO_MEM;

  for (int i = 0; i < oled.pages; i++)
  {
    oled.page[i].data = calloc(width, 1);
    if (!oled.page[i].data)
      return ESP_ERR_NO_MEM;
  }

  uint8_t init_cmd[] = {
      OLED_CTRL_CMD,
      OLED_DISPLAY_OFF,
      OLED_SET_MUX_RATIO, height - 1,
      OLED_SET_OFFSET, 0x00,
      OLED_SET_START_LINE,
      OLED_SEG_REMAP,
      OLED_COM_SCAN_DEC,
      OLED_SET_COM_PINS, 0x12,
      OLED_SET_ADDR_MODE, 0x02, // Page addressing
      OLED_SET_CONTRAST, 0xFF,
      OLED_SET_CLK_DIV, 0x80,
      OLED_DISPLAY_RAM,
      OLED_NORMAL_DISPLAY,
      OLED_SET_CHARGE_PUMP, 0x14,
      OLED_DISPLAY_ON};

  return ssd1306_i2c_write(init_cmd, sizeof(init_cmd));
}

/* ================= FRAMEBUFFER ================= */
void ssd1306_clear(void)
{
  for (int p = 0; p < oled.pages; p++)
    memset(oled.page[p].data, 0x00, oled.width);
}

void ssd1306_pixel(uint8_t x, uint8_t y, bool on)
{
  if (x >= oled.width || y >= oled.height)
    return;

  uint8_t page = y / 8;
  uint8_t bit = 1 << (y % 8);

  if (on)
    oled.page[page].data[x] |= bit;
  else
    oled.page[page].data[x] &= ~bit;
}

/* ================= FLUSH ================= */
esp_err_t ssd1306_flush(void)
{
  for (uint8_t p = 0; p < oled.pages; p++)
  {
    uint8_t cmd[] = {
        OLED_CTRL_CMD,
        OLED_PAGE_ADDR | p,
        OLED_COL_LOW,
        OLED_COL_HIGH};
    ESP_ERROR_CHECK(ssd1306_i2c_write(cmd, sizeof(cmd)));

    uint8_t buf[SSD1306_WIDTH + 1];
    buf[0] = OLED_CTRL_DATA;
    memcpy(&buf[1], oled.page[p].data, oled.width);

    ESP_ERROR_CHECK(ssd1306_i2c_write(buf, sizeof(buf)));
  }
  return ESP_OK;
}
