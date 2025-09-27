/**
 * Copyright (c) 2025 Sjofn LLC
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

#include "malloc.h"
#include "cstring"
#include <SPI.h>

#define ST7735_FREQ 40000000

SPIClass st7735_spi(HSPI);
static const char* TAG = "tft";

TFT::TFT(int8_t cs_pin, int8_t rest_pin, int8_t dc_pin, int8_t sclk_pin,
    int8_t mosi_pin, int8_t led_k_pin, int8_t vtft_ctrl_pin)
{
    _cs_pin = cs_pin;
    _rest_pin = rest_pin;
    _dc_pin = dc_pin;
    _sclk_pin = sclk_pin;
    _mosi_pin = mosi_pin;
    _led_k_pin = led_k_pin;
    _vtft_ctrl_pin = vtft_ctrl_pin;
    _width = ST7735_WIDTH;
    _height = ST7735_HEIGHT;
    _x_start = ST7735_XSTART;
    _y_start = ST7735_YSTART;
}

TFT::~TFT()
{
}

void TFT::select(void)
{
    digitalWrite(_cs_pin, LOW);
}

void TFT::unselect(void)
{
    digitalWrite(_cs_pin, HIGH);
}

void TFT::reset(void)
{
    digitalWrite(_rest_pin, LOW);
    delay(5);
    digitalWrite(_rest_pin, HIGH);
}

void TFT::writeCommand(uint8_t cmd)
{
    digitalWrite(_dc_pin, LOW);
    st7735_spi.transfer(cmd);
}

void TFT::writeData(uint8_t data) {
    digitalWrite(_dc_pin, HIGH);
    st7735_spi.transfer(data);
}

void TFT::writeData(uint8_t *buff, size_t buff_size)
{
    digitalWrite(_dc_pin, HIGH);
    st7735_spi.transfer(buff, buff_size);
}

void TFT::writeColor(uint16_t color)
{
    digitalWrite(_dc_pin, HIGH);
    st7735_spi.transfer16(color);
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

void TFT::init(void) {
    if ((_dc_pin < 0) || (_cs_pin < 0) || (_rest_pin < 0) || (_led_k_pin < 0)) {
        ESP_LOGW(TAG, "Pin error: %x %x %x %x", _dc_pin, _cs_pin, _rest_pin, _led_k_pin);
        return;
    }

    if (_vtft_ctrl_pin >= 0) {
        pinMode(_vtft_ctrl_pin, OUTPUT);
        digitalWrite(_vtft_ctrl_pin, HIGH);
    }

    pinMode(_dc_pin, OUTPUT);
    pinMode(_cs_pin, OUTPUT);
    pinMode(_rest_pin, OUTPUT);

    pinMode(_led_k_pin, OUTPUT);
    digitalWrite(_led_k_pin, HIGH);

    st7735_spi.begin(_sclk_pin, -1, _mosi_pin, _cs_pin);
    st7735_spi.beginTransaction(SPISettings(ST7735_FREQ, MSBFIRST, SPI_MODE0));
    reset();
    select();
    {
        writeCommand(SWRESET);
        delay(150);
    }
    {
        writeCommand(SLPOUT);
        delay(120);
    }
    {
        writeCommand(FRMCTR1);
        uint8_t buffer[3] = {0x01, 0x2C, 0x2D};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(FRMCTR2);
        uint8_t buffer[3] = {0x01, 0x2C, 0x2D};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(FRMCTR3);
        uint8_t buffer[6] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(INVCTR);
        writeData(0x07); // no inversion
    }
    {
        writeCommand(PWCTR1);
        uint8_t buffer[3] = {0xA2, 0x02, 0x84};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(PWCTR2);
        writeData(0xC5);
    }
    {
        writeCommand(PWCTR3);
        uint8_t buffer[2] = {0x0A, 0x00};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(PWCTR4);
        uint8_t buffer[2] = {0x8A, 0x2A};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(PWCTR5);
        uint8_t buffer[2] = {0x8A, 0xEE};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(VMCTR1);
        uint8_t buffer[1] = {0x0E};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(INVOFF);
    }
    {
        writeCommand(MADCTL);
        writeData(ST7735_ROTATION);
    }
    {
        writeCommand(COLMOD);
        writeData(0x05);
    }
#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
    {
        writeCommand(ST7735_CASET);
        uint8_t buffer[4] = {0x00, 0x00, 0x00, 0x7F};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(ST7735_RASET);
        uint8_t buffer[4] = {0x00, 0x00, 0x00, 0x7F};
        writeData(buffer, sizeof(buffer));
    }
#elif defined(ST7735_IS_160X80)
    {
        writeCommand(CASET);
        uint8_t buffer[4] = {0x00, 0x00, 0x00, 0x4F};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(RASET);
        uint8_t buffer[4] = {0x00, 0x00, 0x00, 0x9F};
        writeData(buffer, sizeof(buffer));
    }
#endif
    {
        writeCommand(INVON);
    }
    {
        writeCommand(GMCTRP1);
        uint8_t buffer[16] = {0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
                              0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(GMCTRN1);
        uint8_t buffer[16] = {0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                              0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};
        writeData(buffer, sizeof(buffer));
    }
    {
        writeCommand(NORON);
        delay(10);
    }
    {
        writeCommand(DISPON);
        delay(100);
    }
    unselect();
}

void TFT::drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if ((x >= _width) || (y >= _height)) {
        return;
    }

    select();

    setAddressWindow(x, y, x + 1, y + 1);
    writeColor(color);

    unselect();
}

void TFT::drawChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
    uint32_t i, b, j;

    select();
    setAddressWindow(x, y, x + font.width - 1, y + font.height - 1);

    for (i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for (j = 0; j < font.width; j++) {
            if ((b << j) & 0x8000) {
                writeColor(color);
            } else {
                writeColor(bgcolor);
            }
        }
    }
    unselect();
}

void TFT::drawStr(uint16_t x, uint16_t y, String const &str_data, FontDef font, uint16_t color, uint16_t bgcolor)
{
    const char *str = str_data.c_str();
    drawStr(x, y, str, font, color, bgcolor);
}

void TFT::drawStr(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
    select();

    while (*str) {
        if (x + font.width >= _width) {
            x = 0;
            y += font.height;
            if (y + font.height >= _height) {
                break;
            }
            if (*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }
        drawChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    unselect();
}

void TFT::fillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // clipping
    if ((x >= _width) || (y >= _height)) {
        ESP_LOGW(TAG, "fillRectangle clipped X=%i Y=%i", x, y);
        return;
    }
    if ((x + w - 1) >= _width) w = _width - x;
    if ((y + h - 1) >= _height) h = _height - y;
    select();
    setAddressWindow(x, y, x + w - 1, y + h - 1);

    for (y = h; y > 0; y--) {
        for (x = w; x > 0; x--) {
            writeColor(color);
        }
    }
    unselect();
}

void TFT::fillScreen(uint16_t color)
{
    fillRectangle(0, 0, _width, _height, color);
}

void TFT::drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
    if (x >= _width || y >= _height || (x + w - 1) >= _width || (y + h - 1) >= _height)
    {
        ESP_LOGW(TAG, "drawImage clipped X=%i Y=%i W=%i, H=%i", x, y, w, h);
        return;
    };

    select();
    setAddressWindow(x, y, x + w - 1, y + h - 1);
    writeData((uint8_t *) data, sizeof(uint16_t) * w * h);
    unselect();
}

void TFT::drawXbm(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *xbm, uint16_t color, uint16_t bgcolor)
{
    if (x >= _width || y >= _height || (x + w - 1) >= _width || (y + h - 1) >= _height)
    {
        ESP_LOGW(TAG, "drawImage clipped X=%i Y=%i W=%i, H=%i", x, y, w, h);
        return;
    };

    select();
    setAddressWindow(x, y, x + w - 1, y + h - 1);

    int16_t widthInXbm = (w + 7) >> 3;
    uint8_t data = 0;

    for(int16_t y = 0; y < h; y++) {
        for(int16_t x = 0; x < w; x++ ) {
            if (x & 7) {
                data >>= 1; // Move a bit
            } else {  // Read new data every 8 bit
                data = pgm_read_byte(xbm + (x >> 3) + y * widthInXbm);
            }
            // if there is a bit draw it
            if (data & 0x01) {
                writeColor(color);
            } else {
                writeColor(bgcolor);
            }
        }
    }

    unselect();
}


void TFT::invertColors(bool invert)
{
    select();
    writeCommand(invert ? INVON : INVOFF);
    unselect();
}

void TFT::setGamma(GammaDef gamma)
{
    uint8_t data[1] = { static_cast<uint8_t>(gamma) };
    select();
    writeCommand(GAMSET);
    writeData(data, sizeof(data));
    unselect();
}
