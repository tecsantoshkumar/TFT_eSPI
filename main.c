#include "TFT_eSPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    if (TFT_init() == ESP_OK) {
        TFT_fillScreen(0xF800); // Red
        vTaskDelay(pdMS_TO_TICKS(1000));

        TFT_drawPixel(100, 100, 0x07E0); // Green
        vTaskDelay(pdMS_TO_TICKS(1000));

        TFT_fillScreen(0x001F); // Blue
    }
}
