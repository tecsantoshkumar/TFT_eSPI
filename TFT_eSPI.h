#ifndef _TFT_ESPI_H_
#define _TFT_ESPI_H_

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// === Pin definitions ===
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4

#define LCD_HOST        SPI2_HOST
#define SPI_FREQUENCY   40000000

#define TFT_WIDTH   480
#define TFT_HEIGHT  320

// === ILI9488 Commands ===
#define ILI9488_SWRESET  0x01
#define ILI9488_SLPOUT   0x11
#define ILI9488_DISPON   0x29
#define ILI9488_CASET    0x2A
#define ILI9488_PASET    0x2B
#define ILI9488_RAMWR    0x2C
#define ILI9488_MADCTL   0x36
#define ILI9488_COLMOD   0x3A

// === Public API ===
esp_err_t TFT_init(void);
void TFT_fillScreen(uint16_t color);
void TFT_drawPixel(uint16_t x, uint16_t y, uint16_t color);
void TFT_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

#endif
