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

#ifndef TFT_H_
#define TFT_H_

#include <Arduino.h>
#include "fonts.h"

#define ST7735_IS_160X80 1
#define ST7735_XSTART 1
#define ST7735_YSTART 26
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 80
#define ST7735_ROTATION (TFT::MADCTL_ARG::MY | TFT::MADCTL_ARG::MV | TFT::MADCTL_ARG::BGR)

// call before initializing any SPI devices

class TFT
{
public:
	typedef enum : uint16_t {
		BLACK   = 0x0000,
		BLUE    = 0x001F,
		RED     = 0xF800,
		GREEN   = 0x07E0,
		CYAN    = 0x07FF,
		MAGENTA = 0xF81F,
		YELLOW  = 0xFFE0,
		WHITE   = 0xFFFF,
	} Color;

	static inline uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) {
		return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
	}

	typedef enum {
		GAMMA_10 = 0x01,
		GAMMA_25 = 0x02,
		GAMMA_22 = 0x04,
		GAMMA_18 = 0x08,
	} GammaDef;

	typedef enum {
		MY = 0x80,	// page address order (0 = top-to-bottom)
		MX = 0x40,	// column address order (0 = left-to-right)
		MV = 0x20,	// page/column order (0 = normal mode)
		ML = 0x10,	// line address order (0 = LCD refresh top-to-bottom)
		RGB = 0x00, // RGB order (0 = RGB)
		BGR = 0x08,	// BGR order (0 = BGR)
		MH = 0x04,	// display data latch order (0 = LCD refresh left-to-right)
	} MADCTL_ARG;

	TFT(int8_t cs_pin, int8_t rest_pin, int8_t dc_pin, int8_t sclk_pin, int8_t mosi_pin, int8_t led_k_pin, int8_t vtft_ctrl_pin);
	~TFT();
	void init(void);
	void drawPixel(uint16_t x, uint16_t y, uint16_t color);
	void drawChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
	void drawStr(uint16_t x, uint16_t y, String const& str_data, FontDef font=Font_11x18, uint16_t color=BLUE, uint16_t bgcolor=BLACK);
	void drawStr(uint16_t x, uint16_t y, const char *str, FontDef font=Font_11x18, uint16_t color=BLUE, uint16_t bgcolor=BLACK);
	void fillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
	void fillScreen(uint16_t color);
	void drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
	void drawXbm(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t* xbm, uint16_t color=BLUE, uint16_t bgcolor=BLACK);
	void invertColors(bool invert);
	void setGamma(GammaDef gamma);

	uint8_t width() const { return _width; }
	uint8_t height() const { return _height; }

private:
	typedef enum :uint8_t {
		NOP     = 0x00,
		SWRESET = 0x01,
		RDDID   = 0x04,
		RDDST   = 0x09,

		SLPIN   = 0x10,
		SLPOUT  = 0x11,
		PTLON   = 0x12,
		NORON   = 0x13,

		INVOFF  = 0x20,
		INVON   = 0x21,
		GAMSET  = 0x26,
		DISPOFF = 0x28,
		DISPON  = 0x29,
		CASET   = 0x2A,
		RASET   = 0x2B,
		RAMWR   = 0x2C,
		RAMRD   = 0x2E,

		PTLAR   = 0x30,
		COLMOD  = 0x3A,
		MADCTL  = 0x36,

		FRMCTR1 = 0xB1,
		FRMCTR2 = 0xB2,
		FRMCTR3 = 0xB3,
		INVCTR  = 0xB4,
		DISSET5 = 0xB6,

		PWCTR1  = 0xC0,
		PWCTR2  = 0xC1,
		PWCTR3  = 0xC2,
		PWCTR4  = 0xC3,
		PWCTR5  = 0xC4,
		VMCTR1  = 0xC5,

		RDID1   = 0xDA,
		RDID2   = 0xDB,
		RDID3   = 0xDC,
		RDID4   = 0xDD,

		PWCTR6  = 0xFC,

		GMCTRP1 = 0xE0,
		GMCTRN1 = 0xE1,
	} CMD;

	/* data */
	void inline select(void);
	void inline unselect(void);
	void inline reset(void);
	void writeCommand(uint8_t cmd);
	void writeData(uint8_t data);
	void writeData(uint8_t* buff, size_t buff_size);
	void writeColor(uint16_t color);
	void setAddressWindow(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
	int8_t 	  _cs_pin;
	int8_t    _rest_pin;     
	int8_t    _dc_pin;    
	int8_t    _sclk_pin;   
	int8_t    _mosi_pin;  
	int8_t    _led_k_pin; 
	int8_t    _vtft_ctrl_pin;
	uint16_t _width;
	uint16_t _height;
	uint16_t _x_start;
	uint16_t _y_start;

public:
	class ScopedSelect {
	public:
		ScopedSelect(int8_t pin);
		~ScopedSelect();
	private:
		int8_t _pin;
	};
};

#endif // TFT_H_
