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
// #define SSD1306_SDA_PIN GPIO_NUM_21
// #define SSD1306_SCL_PIN GPIO_NUM_22
// #define SSD1306_FREQ_HZ 400000
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

/* ================= 5x7 ASCII FONT =================
 * Characters 32 (space) to 127 (~)
 * Each character is 5 bytes wide, LSB = top pixel
 * Public domain
 */
static const uint8_t font5x7[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32 ' '
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33 '!'
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34 '"'
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35 '#'
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36 '$'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37 '%'
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38 '&'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39 '''
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40 '('
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41 ')'
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // 42 '*'
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43 '+'
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44 ','
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45 '-'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46 '.'
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47 '/'
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48 '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49 '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50 '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51 '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52 '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53 '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54 '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55 '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56 '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57 '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58 ':'
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59 ';'
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60 '<'
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61 '='
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62 '>'
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63 '?'
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // 64 '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 65 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 70 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 71 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 77 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 87 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 89 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90 'Z'
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // 91 '['
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92 '\'
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // 93 ']'
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94 '^'
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 95 '_'
};

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

void ssd1306_char(uint8_t x, uint8_t y, char c, bool on)
{
  if (c < 32 || c > 127)
    c = '?';

  const uint8_t *glyph = font5x7[c - 32];

  for (int col = 0; col < 5; col++)
  {
    uint8_t line = glyph[col];
    for (int row = 0; row < 7; row++)
    {
      if (line & (1 << row))
      {
        ssd1306_pixel(x + col, y + row, on);
      }
    }
  }
}

void ssd1306_text(uint8_t x, uint8_t y, const char *str, bool on)
{
  while (*str)
  {
    ssd1306_char(x, y, *str, on);
    x += 6; // 5px font + 1px spacing

    if (x + 5 >= oled.width)
    {
      x = 0;
      y += 8;
    }
    if (y + 7 >= oled.height)
      break;

    str++;
  }
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
