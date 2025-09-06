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
#include <HT_SSD1306Wire.h>

struct notification_def;

typedef enum
{
    BLE_DISCONNECTED = 0,
    BLE_PAIRING,
    BLE_SERVER_CONNECTED,
    BLE_CONNECTED
} conn_state_def;

typedef void (*PairButtonCallback)();

class Heltec_ESP32
{
public:
    static constexpr uint8_t BUTTON = 0;
    static constexpr uint8_t LED = 35;
    static constexpr uint8_t Vext = 36;
    static constexpr uint8_t SDA_OLED = 17;
    static constexpr uint8_t SCL_OLED = 18;
    static constexpr uint8_t RST_OLED = 21;
    static constexpr uint8_t RST_LoRa = 12;
    static constexpr uint8_t DIO0 = 26;
    static constexpr uint8_t ADC_CTRL = 37;
    static constexpr uint8_t VBAT_READ = 1;

    Heltec_ESP32();
    virtual ~Heltec_ESP32();
    void begin();
    void loop();

    void setBLEConnectionState(conn_state_def state);
    void pairing(String const& passcode);
    float getBatteryVoltage();

private:
    void showNotification(notification_def const& notification);
    void drawHeader();
    void standby();

    SSD1306Wire mDisplay;
    conn_state_def mBleState;
    String mMessage;
};

extern Heltec_ESP32 Heltec;

#endif // HELTEC_H_