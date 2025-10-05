# ESP-IDF TFT_eSPI (C-Only ILI9488 Driver)

A lightweight and fast **TFT display driver for ESP32** using **ESP-IDF** written entirely in **C** — no Arduino layer, no C++.  
This project is designed for developers who prefer a clean, native ESP-IDF implementation of the ILI9488 SPI TFT driver.

---

## 📢 News

1. **Pure-C Driver:**  
   Full ESP-IDF compatible ILI9488 driver in pure C — ideal for embedded developers and production firmware.

2. **Modular Design:**  
   SPI and TFT logic are separated in `TFT_eSPI.c` and `TFT_eSPI.h` for easy integration with existing IDF projects.

3. **New Functions:**  
   Added basic drawing support — `fillScreen`, `drawPixel`, `setWindow`.  
   Rectangle, line, and gradient functions coming soon.

4. **Multi-Board Support:**  
   Tested with ESP32-WROOM and ESP32-S3 DevKit boards at up to 40 MHz SPI frequency.

---

## 🧩 Folder Structure

```
esp32-ili9488-tft/
├── main/
│ ├── TFT_eSPI.c
│ ├── TFT_eSPI.h
│ ├── main.c
│ ├── CMakeLists.txt
├── CMakeLists.txt
├── README.md
```

---

## ⚙️ Pin Configuration

| Signal | GPIO | Description |
|--------|------|-------------|
| MOSI | 23 | SPI MOSI |
| MISO | 19 | SPI MISO *(optional)* |
| CLK  | 18 | SPI Clock |
| CS   | 15 | Chip Select |
| DC   | 2  | Data/Command |
| RST  | 4  | Reset |

> Backlight (LED/BLK) must be connected to **3.3V** or controlled via a transistor if required.

---

## 🧠 Supported Controllers

| Controller | Interface | Status | Color Depth |
|-------------|------------|----------|---------------|
| ILI9488 | SPI | ✅ Working | 18-bit RGB |
| ILI9341 | SPI | 🧪 Testing | 16-bit RGB |
| ST7796 | SPI | 🧩 Planned | 16-bit RGB |

---

## ⚙️ Build Configuration

### TFT_eSPI.h

```
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   15
#define PIN_NUM_DC   2
#define PIN_NUM_RST  4
#define SPI_FREQUENCY 40000000
#define LCD_HOST SPI2_HOST


#include "TFT_eSPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    TFT_init();

    TFT_fillScreen(0xF800); // Red
    vTaskDelay(pdMS_TO_TICKS(1000));

    TFT_drawPixel(100, 100, 0x07E0); // Green pixel
    vTaskDelay(pdMS_TO_TICKS(1000));

    TFT_fillScreen(0x001F); // Blue
}
```

## Build & Flash
```
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

## API Reference
```
void TFT_init(void);
void TFT_reset(void);
void TFT_sendCommand(uint8_t cmd);
void TFT_sendData(const uint8_t *data, int len);
void TFT_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void TFT_drawPixel(uint16_t x, uint16_t y, uint16_t color);
void TFT_fillScreen(uint16_t color);
```

## Author

Santosh Kumar
Embedded Systems Developer | ESP-IDF | IoT | Firmware
