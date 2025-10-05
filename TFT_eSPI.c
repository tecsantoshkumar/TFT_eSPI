#include "TFT_eSPI.h"
#include "esp_log.h"

static const char *TAG = "TFT_eSPI";
static spi_device_handle_t spi;

// === Internal helper functions ===
static void TFT_sendCmd(uint8_t cmd)
{
    gpio_set_level(TFT_DC, 0); // Command mode
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(spi, &t);
}

static void TFT_sendData(const uint8_t *data, int len)
{
    gpio_set_level(TFT_DC, 1); // Data mode
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(spi, &t);
}

static void TFT_reset(void)
{
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
}

// === Public driver functions ===

esp_err_t TFT_init(void)
{
    esp_err_t ret;

    // --- Configure GPIOs ---
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TFT_DC) | (1ULL << TFT_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // --- SPI bus config ---
    spi_bus_config_t buscfg = {
        .miso_io_num = TFT_MISO,
        .mosi_io_num = TFT_MOSI,
        .sclk_io_num = TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_WIDTH * 3 * 2 + 8
    };
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    // --- SPI device config ---
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_FREQUENCY,
        .mode = 0,
        .spics_io_num = TFT_CS,
        .queue_size = 7
    };
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    if (ret != ESP_OK) return ret;

    // --- ILI9488 init sequence ---
    TFT_reset();

    TFT_sendCmd(ILI9488_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(120));

    TFT_sendCmd(ILI9488_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));

    TFT_sendCmd(ILI9488_COLMOD);
    uint8_t color_mode = 0x66; // 18-bit color
    TFT_sendData(&color_mode, 1);

    TFT_sendCmd(ILI9488_MADCTL);
    uint8_t rotation = 0x48; // RGB + row/col exchange
    TFT_sendData(&rotation, 1);

    TFT_sendCmd(ILI9488_DISPON);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ILI9488 Initialized OK");
    return ESP_OK;
}

void TFT_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    TFT_sendCmd(ILI9488_CASET);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    TFT_sendData(data, 4);

    TFT_sendCmd(ILI9488_PASET);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    TFT_sendData(data, 4);

    TFT_sendCmd(ILI9488_RAMWR);
}

void TFT_drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    TFT_setWindow(x, y, x, y);
    uint8_t data[3];
    data[0] = (color >> 8) & 0xF8; // R
    data[1] = (color >> 3) & 0xFC; // G
    data[2] = (color << 3) & 0xF8; // B
    TFT_sendData(data, 3);
}

void TFT_fillScreen(uint16_t color)
{
    TFT_setWindow(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);

    uint8_t data[3];
    data[0] = (color >> 8) & 0xF8;
    data[1] = (color >> 3) & 0xFC;
    data[2] = (color << 3) & 0xF8;

    gpio_set_level(TFT_DC, 1);
    spi_transaction_t t = {0};
    t.length = TFT_WIDTH * TFT_HEIGHT * 3 * 8;
    t.tx_buffer = data;
    t.flags = SPI_TRANS_USE_TXDATA;
    spi_device_polling_transmit(spi, &t);
}
