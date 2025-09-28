/**
 * Copyright (c) 2024-2025 Sjofn LLC
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

#ifndef HELTEC_H_
#define HELTEC_H_

#include <Arduino.h>
#include "tft.h"

struct notification_def;

typedef enum
{
    BLE_DISCONNECTED = 0,
    BLE_PAIRING,
    BLE_SERVER_CONNECTED,
    BLE_CONNECTED
} conn_state_def;

typedef void (*PairButtonCallback)();

class Hardware
{
    friend class NotificationService;

public:
    static constexpr uint8_t ADC_CTRL = 2;
    static constexpr uint8_t VEXT_CTRL = 3;

    // TFT
    static constexpr uint8_t ST7735_CS = 38;
    static constexpr uint8_t ST7735_REST = 39;
    static constexpr uint8_t ST7735_RS = 40;
    static constexpr uint8_t ST7735_SCLK = 41;
    static constexpr uint8_t ST7735_MOSI = 42;
    static constexpr uint8_t ST7735_LED = 21;

    // Misc
    static constexpr uint8_t BUTTON = 0;
    static constexpr uint8_t FACTORY_LED = 18;
    static constexpr uint8_t VBAT_READ = 1;
    static constexpr uint8_t BUZZER = 45;

    // LoRa
    static constexpr uint8_t NSS = 8;
    static constexpr uint8_t CLK = 9;
    static constexpr uint8_t MOSI = 10;
    static constexpr uint8_t MISO = 11;
    static constexpr uint8_t RESET = 12;
    static constexpr uint8_t BUSY = 13;
    static constexpr uint8_t DIO_1 = 14;

    static constexpr uint16_t HEADER_COLOR = 0x3190;

    Hardware();
    virtual ~Hardware();
    void begin();

    void pairing(String const& passcode);
    void setBLEConnectionState(conn_state_def state);
    void showTime(String const& timestamp);
    void showGpsState(bool connected);
    void glow(bool on);

    uint8_t getBatteryLevel();

protected:
    TaskHandle_t mDrawTask = nullptr;

private:
    static void startDrawing(void* pvParameters);
    static void batteryTimerCallback(TimerHandle_t xTimer);
    void showNotification(notification_def const& notification);

    void blank();
    void drawIcon(uint16_t x, uint16_t y, uint8_t const* xbm);
    void showBLEState(conn_state_def state);
    void showBatteryLevel(uint8_t percent);
    void standby();

    TFT mDisplay;
    conn_state_def mBleState = BLE_DISCONNECTED;
    bool mGpsState = false;
    uint8_t mBatteryLevel = 0;
    String mMessage = "";
    TimerHandle_t mBatteryTimer = nullptr;
};

extern Hardware Heltec;

#endif // HELTEC_H_