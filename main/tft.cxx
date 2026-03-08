/**
 * Copyright (c) 2025-2026 Sjofn LLC
 * Based on Limor Fried/Ladyada's driver for Adafruit Industries.
 * https://github.com/adafruit/Adafruit-ST7735-Library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tft.h"

#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

static const char* TAG = "tft";

// SPI3_HOST on ESP32-S3 — driven by the IDF spi_master driver.
// 40 MHz works reliably through the GPIO matrix with short PCB traces.
static constexpr spi_host_device_t TFT_SPI_HOST = SPI3_HOST;

// Maximum SPI clock for the ST7735S.  40 MHz works reliably through the
// GPIO matrix on ESP32-S3 with short PCB traces.
static constexpr int TFT_SPI_FREQ_HZ = 40 * 1000 * 1000;

// ── DC-pin pre-transfer callback ──────────────────────────────────────────
// Called by spi_device_polling_transmit() before every transaction.
// t->user encodes the DC level: 0 = command (DC LOW), 1 = data (DC HIGH).
// Stored file-scope because the IDF callback has no user-data pointer.
static gpio_num_t s_dc_pin = GPIO_NUM_NC;

static void IRAM_ATTR spi_pre_transfer_cb(spi_transaction_t* t)
{
    gpio_set_level(s_dc_pin, static_cast<int>(reinterpret_cast<intptr_t>(t->user)));
}

// ── Helpers ───────────────────────────────────────────────────────────────

TFT::TFT(int8_t cs_pin, int8_t rest_pin, int8_t dc_pin, int8_t sclk_pin,
    int8_t mosi_pin, int8_t led_k_pin, int8_t vtft_ctrl_pin)
:   _cs_pin(cs_pin)
,   _rest_pin(rest_pin)
,   _dc_pin(dc_pin)
,   _sclk_pin(sclk_pin)
,   _mosi_pin(mosi_pin)
,   _led_k_pin(led_k_pin)
,   _vtft_ctrl_pin(vtft_ctrl_pin)
,   _width(ST7735_WIDTH)
,   _height(ST7735_HEIGHT)
,   _x_start(ST7735_XSTART)
,   _y_start(ST7735_YSTART)
{ }

void TFT::reset(void)
{
    gpio_set_level(static_cast<gpio_num_t>(_rest_pin), 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(static_cast<gpio_num_t>(_rest_pin), 1);
}

// Send one command byte (DC = LOW).
void TFT::writeCommand(uint8_t cmd)
{
    spi_transaction_t t = {};
    t.flags     = SPI_TRANS_USE_TXDATA;
    t.length    = 8;
    t.tx_data[0] = cmd;
    t.user      = reinterpret_cast<void*>(0);   // DC LOW
    spi_device_polling_transmit(_spi, &t);
}

// Send one data byte (DC = HIGH).
void TFT::writeData(uint8_t data)
{
    spi_transaction_t t = {};
    t.flags     = SPI_TRANS_USE_TXDATA;
    t.length    = 8;
    t.tx_data[0] = data;
    t.user      = reinterpret_cast<void*>(1);   // DC HIGH
    spi_device_polling_transmit(_spi, &t);
}

// Send a byte buffer as data (DC = HIGH).
// For ≤4 bytes uses inline tx_data (no DMA concern).
// Larger buffers must be in DRAM (stack/static on ESP32-S3 always qualifies).
void TFT::writeData(uint8_t* buff, size_t buff_size)
{
    if (buff_size == 0) return;
    spi_transaction_t t = {};
    t.length = buff_size * 8;
    t.user   = reinterpret_cast<void*>(1);      // DC HIGH
    if (buff_size <= 4) {
        t.flags = SPI_TRANS_USE_TXDATA;
        memcpy(t.tx_data, buff, buff_size);
    } else {
        t.tx_buffer = buff;                      // DRAM → DMA-capable on ESP32-S3
    }
    spi_device_polling_transmit(_spi, &t);
}

void TFT::setAddressWindow(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    // column address set
    writeCommand(CASET);
    uint8_t data[4] = { 0x00, static_cast<uint8_t>(x + _x_start), 0x00, static_cast<uint8_t>(w + _x_start) };
    writeData(data, sizeof(data));

    // row address set
    writeCommand(RASET);
    data[1] = y + _y_start;
    data[3] = h + _y_start;
    writeData(data, sizeof(data));

    // write to RAM
    writeCommand(RAMWR);
}

// ── init ──────────────────────────────────────────────────────────────────

void TFT::init(void)
{
    if ((_dc_pin < 0) || (_cs_pin < 0) || (_rest_pin < 0) || (_led_k_pin < 0)) {
        ESP_LOGW(TAG, "Pin error: dc=%d cs=%d rst=%d led=%d",
                 _dc_pin, _cs_pin, _rest_pin, _led_k_pin);
        return;
    }

    // ── DMA pixel buffer ──────────────────────────────────────────────────
    // Allocated before the SPI init sequence so writeData can use it for
    // gamma tables (> 4 bytes but guaranteed DRAM from heap).
    _dma_buf = static_cast<uint16_t*>(
        heap_caps_malloc(ST7735_WIDTH * sizeof(uint16_t),
                         MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
    if (!_dma_buf) {
        ESP_LOGE(TAG, "DMA buffer allocation failed");
        return;
    }

    // ── GPIO: all non-SPI output pins in one config call ─────────────────
    // DC is driven by spi_pre_transfer_cb; RST, LED_K, and optionally
    // VTFT_CTRL are plain outputs driven directly by gpio_set_level().
    s_dc_pin = static_cast<gpio_num_t>(_dc_pin);
    uint64_t out_mask = (1ULL << _dc_pin) |
                        (1ULL << _rest_pin) |
                        (1ULL << _led_k_pin);
    if (_vtft_ctrl_pin >= 0) out_mask |= (1ULL << _vtft_ctrl_pin);
    const gpio_config_t out_conf = {
        .pin_bit_mask  = out_mask,
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);
    if (_vtft_ctrl_pin >= 0)
        gpio_set_level(static_cast<gpio_num_t>(_vtft_ctrl_pin), 1);  // VEXT on
    gpio_set_level(static_cast<gpio_num_t>(_led_k_pin), 1);           // backlight on
    gpio_set_level(static_cast<gpio_num_t>(_rest_pin),  1);           // RST idle-high
    gpio_set_level(static_cast<gpio_num_t>(_dc_pin),    1);           // DC harmless default

    // ── IDF SPI bus + device ──────────────────────────────────────────────
    // CS is managed by the SPI driver; do NOT call pinMode for it here.
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num     = _mosi_pin;
    buscfg.miso_io_num     = -1;
    buscfg.sclk_io_num     = _sclk_pin;
    buscfg.quadwp_io_num   = -1;
    buscfg.quadhd_io_num   = -1;
    buscfg.max_transfer_sz = ST7735_WIDTH * static_cast<int>(sizeof(uint16_t));
    esp_err_t ret = spi_bus_initialize(TFT_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(ret));
        // Fall through — spi_bus_add_device may still succeed if bus was
        // already initialised with compatible settings.
    } else {
        ESP_LOGI(TAG, "SPI bus init OK (MOSI=GPIO%d SCLK=GPIO%d)",
                 _mosi_pin, _sclk_pin);
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.mode           = 0;                    // CPOL=0 CPHA=0
    devcfg.clock_speed_hz = TFT_SPI_FREQ_HZ;
    devcfg.spics_io_num   = _cs_pin;
    devcfg.queue_size     = 1;
    devcfg.pre_cb         = spi_pre_transfer_cb;
    ret = spi_bus_add_device(TFT_SPI_HOST, &devcfg, &_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ST7735 device added (CS=GPIO%d @ %d MHz)",
             _cs_pin, TFT_SPI_FREQ_HZ / 1000000);

    // ── ST7735 init sequence ──────────────────────────────────────────────
    reset();

    writeCommand(SWRESET); vTaskDelay(pdMS_TO_TICKS(150));
    writeCommand(SLPOUT);  vTaskDelay(pdMS_TO_TICKS(120));

    writeCommand(FRMCTR1);
    { uint8_t b[] = {0x01, 0x2C, 0x2D}; writeData(b, sizeof(b)); }
    writeCommand(FRMCTR2);
    { uint8_t b[] = {0x01, 0x2C, 0x2D}; writeData(b, sizeof(b)); }
    writeCommand(FRMCTR3);
    { uint8_t b[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}; writeData(b, sizeof(b)); }

    writeCommand(INVCTR);  writeData(0x07);

    writeCommand(PWCTR1);  { uint8_t b[] = {0xA2, 0x02, 0x84}; writeData(b, sizeof(b)); }
    writeCommand(PWCTR2);  writeData(0xC5);
    writeCommand(PWCTR3);  { uint8_t b[] = {0x0A, 0x00}; writeData(b, sizeof(b)); }
    writeCommand(PWCTR4);  { uint8_t b[] = {0x8A, 0x2A}; writeData(b, sizeof(b)); }
    writeCommand(PWCTR5);  { uint8_t b[] = {0x8A, 0xEE}; writeData(b, sizeof(b)); }
    writeCommand(VMCTR1);  writeData(0x0E);
    writeCommand(INVOFF);
    writeCommand(MADCTL);  writeData(ST7735_ROTATION);
    writeCommand(COLMOD);  writeData(0x05);

#if defined(ST7735_IS_160X80)
    writeCommand(CASET);   { uint8_t b[] = {0x00, 0x00, 0x00, 0x4F}; writeData(b, sizeof(b)); }
    writeCommand(RASET);   { uint8_t b[] = {0x00, 0x00, 0x00, 0x9F}; writeData(b, sizeof(b)); }
    writeCommand(INVON);
#elif defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128)
    writeCommand(CASET);   { uint8_t b[] = {0x00, 0x00, 0x00, 0x7F}; writeData(b, sizeof(b)); }
    writeCommand(RASET);   { uint8_t b[] = {0x00, 0x00, 0x00, 0x7F}; writeData(b, sizeof(b)); }
#endif

    writeCommand(GMCTRP1);
    { uint8_t b[] = {0x02,0x1c,0x07,0x12,0x37,0x32,0x29,0x2d,
                     0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10}; writeData(b, sizeof(b)); }
    writeCommand(GMCTRN1);
    { uint8_t b[] = {0x03,0x1d,0x07,0x06,0x2E,0x2C,0x29,0x2D,
                     0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10}; writeData(b, sizeof(b)); }

    writeCommand(NORON);  vTaskDelay(pdMS_TO_TICKS(10));
    writeCommand(DISPON); vTaskDelay(pdMS_TO_TICKS(100));
}

// ── Drawing primitives ────────────────────────────────────────────────────

void TFT::drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if ((x >= _width) || (y >= _height)) return;

    setAddressWindow(x, y, x, y);
    _dma_buf[0] = __builtin_bswap16(color);

    spi_transaction_t t = {};
    t.length    = 16;
    t.tx_buffer = _dma_buf;
    t.user      = reinterpret_cast<void*>(1);
    spi_device_polling_transmit(_spi, &t);
}

void TFT::drawChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
    if (ch < 32 || ch >= 127) return;

    setAddressWindow(x, y, x + font.width - 1, y + font.height - 1);

    // Pre-swap colors once; font data is in flash (.rodata) so read via CPU
    // and expand each row into _dma_buf before the DMA transaction.
    const uint16_t color_be   = __builtin_bswap16(color);
    const uint16_t bgcolor_be = __builtin_bswap16(bgcolor);

    spi_transaction_t t = {};
    t.length    = font.width * 16;
    t.tx_buffer = _dma_buf;
    t.user      = reinterpret_cast<void*>(1);

    for (uint32_t row = 0; row < font.height; row++) {
        uint32_t bits = font.data[(ch - 32) * font.height + row];
        for (uint32_t col = 0; col < font.width; col++) {
            _dma_buf[col] = ((bits << col) & 0x8000) ? color_be : bgcolor_be;
        }
        spi_device_polling_transmit(_spi, &t);
    }
}

void TFT::drawStr(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor)
{
    while (*str) {
        if (x + font.width >= _width) {
            x = 0;
            y += font.height;
            if (y + font.height >= _height) break;
            if (*str == ' ') { str++; continue; }
        }
        drawChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }
}

void TFT::fillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((x >= _width) || (y >= _height)) {
        ESP_LOGW(TAG, "fillRectangle clipped X=%u Y=%u", x, y);
        return;
    }
    if ((x + w - 1) >= _width)  w = _width  - x;
    if ((y + h - 1) >= _height) h = _height - y;

    setAddressWindow(x, y, x + w - 1, y + h - 1);

    // Pre-fill the DMA buffer with the byte-swapped colour once, then blast
    // each row in a single DMA transaction — avoids per-pixel SPI overhead.
    const uint16_t color_be = __builtin_bswap16(color);
    for (uint16_t i = 0; i < w; i++) {
        _dma_buf[i] = color_be;
    }

    spi_transaction_t t = {};
    t.length    = static_cast<size_t>(w) * 16;
    t.tx_buffer = _dma_buf;
    t.user      = reinterpret_cast<void*>(1);

    for (uint16_t row = 0; row < h; row++) {
        spi_device_polling_transmit(_spi, &t);
    }
}

void TFT::fillScreen(uint16_t color)
{
    fillRectangle(0, 0, _width, _height, color);
}

void TFT::drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{
    if (x >= _width || y >= _height ||
        (x + w - 1) >= _width || (y + h - 1) >= _height)
    {
        ESP_LOGW(TAG, "drawImage clipped X=%u Y=%u W=%u H=%u", x, y, w, h);
        return;
    }

    setAddressWindow(x, y, x + w - 1, y + h - 1);

    // data[] is in flash (.rodata) — not DMA-accessible.  Copy one row at a
    // time into _dma_buf (DRAM) with byte-swap, then DMA to display.
    spi_transaction_t t = {};
    t.length    = static_cast<size_t>(w) * 16;
    t.tx_buffer = _dma_buf;
    t.user      = reinterpret_cast<void*>(1);

    for (uint16_t row = 0; row < h; row++) {
        const uint16_t* src = data + static_cast<size_t>(row) * w;
        for (uint16_t col = 0; col < w; col++) {
            _dma_buf[col] = __builtin_bswap16(src[col]);
        }
        spi_device_polling_transmit(_spi, &t);
    }
}

void TFT::drawXbm(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                  const uint8_t* xbm, uint16_t color, uint16_t bgcolor)
{
    if (x >= _width || y >= _height ||
        (x + w - 1) >= _width || (y + h - 1) >= _height)
    {
        ESP_LOGW(TAG, "drawXbm clipped X=%u Y=%u W=%u H=%u", x, y, w, h);
        return;
    }

    setAddressWindow(x, y, x + w - 1, y + h - 1);

    const uint16_t color_be   = __builtin_bswap16(color);
    const uint16_t bgcolor_be = __builtin_bswap16(bgcolor);
    const uint16_t widthInXbm = (w + 7) >> 3;

    // XBM bit ordering: LSB of each byte = leftmost pixel.
    // Read byte-by-byte from flash via CPU cache, expand into _dma_buf, DMA per row.
    spi_transaction_t t = {};
    t.length    = static_cast<size_t>(w) * 16;
    t.tx_buffer = _dma_buf;
    t.user      = reinterpret_cast<void*>(1);

    for (uint16_t row = 0; row < h; row++) {
        uint8_t xbm_byte = 0;
        for (uint16_t col = 0; col < w; col++) {
            if (!(col & 7)) {
                xbm_byte = pgm_read_byte(xbm + (col >> 3) + row * widthInXbm);
            } else {
                xbm_byte >>= 1;
            }
            _dma_buf[col] = (xbm_byte & 0x01) ? color_be : bgcolor_be;
        }
        spi_device_polling_transmit(_spi, &t);
    }
}

void TFT::invertColors(bool invert)
{
    writeCommand(invert ? INVON : INVOFF);
}

void TFT::setGamma(GammaDef gamma)
{
    writeCommand(GAMSET);
    writeData(static_cast<uint8_t>(gamma));
}
