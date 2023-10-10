#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

//display driver
#define TFT_SPI_HOST VSPI_HOST
#define TFT_SPI_MOSI 19
#define TFT_SPI_MISO -1
#define TFT_SPI_CLK 18
#define TFT_SPI_CS 5
#define TFT_PIN_DC 16
#define TFT_PIN_RST 23
#define TFT_PIN_BUSY 35
#define TFT_PIN_BCKL 4

#define TFT_PIXEL_CLOCK_HZ 135 * 240 * 30 //30FPS
#define TFT_CMD_BITS 8
#define TFT_PARAM_BITS 8

#define EXAMPLE_LCD_H_RES              135
#define EXAMPLE_LCD_V_RES              240

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16 //16

// void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active);

// void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len);

// void lcd_spi_pre_transfer_callback(spi_transaction_t *t);

// uint32_t lcd_get_id(spi_device_handle_t spi);

// void lcd_init(spi_device_handle_t spi);

// void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata);

// void send_line_finish(spi_device_handle_t spi);

// void display_pretty_colors(spi_device_handle_t spi);