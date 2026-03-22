/**
 * Copyright (c) 2024-2026 Sjofn LLC
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

#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "battery_monitor.h"
#include "display.h"          // pulls in util.h → conn_state_def
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/timers.h>
#include <time.h>
#include <climits>

struct notification_def;

class Hardware
{
    friend class NotificationService;

public:
    static constexpr uint8_t ADC_CTRL   = 2;
    static constexpr uint8_t VEXT_CTRL  = 3;

    // TFT
    static constexpr uint8_t ST7735_CS   = 38;
    static constexpr uint8_t ST7735_REST = 39;
    static constexpr uint8_t ST7735_RS   = 40;
    static constexpr uint8_t ST7735_SCLK = 41;
    static constexpr uint8_t ST7735_MOSI = 42;
    static constexpr uint8_t ST7735_LED  = 21;

    // Misc
    static constexpr uint8_t BUTTON      = 0;
    static constexpr uint8_t FACTORY_LED = 18;
    static constexpr uint8_t VBAT_READ   = 1;
    static constexpr uint8_t BUZZER      = 45;

    // Task-notification event bits for the draw task
    static constexpr uint32_t DRAW_NOTIFY    = (1u << 0);
    static constexpr uint32_t DRAW_STATE     = (1u << 1);
    static constexpr uint32_t DRAW_BATTERY   = (1u << 2);
    static constexpr uint32_t DRAW_TIME      = (1u << 3);
    static constexpr uint32_t DRAW_GPS       = (1u << 4);
    static constexpr uint32_t DRAW_LORA      = (1u << 5);
    static constexpr uint32_t DRAW_LORA_POS  = (1u << 6);
    static constexpr uint32_t DRAW_LORA_NODE = (1u << 7);
    static constexpr uint32_t DRAW_LORA_MESH = (1u << 8);
    static constexpr uint32_t DRAW_CHARGING  = (1u << 9);

    Hardware();
    ~Hardware() = default;
    void begin();

    void pairing(const char* passcode);
    void setBLEConnectionState(conn_state_def state);
    void notifyDraw(uint32_t events);
    void showTime(const char* timestamp);
    void showCallState(bool active);
    void showGpsState(bool fixed);
    void showLoraState(bool connected);
    void glow(bool on);
    void showLoraMessage(struct MeshMessage const& msg);
    void showPositionMessage(struct MeshPosition const& pos);
    void showNodeInfoMessage(struct MeshUser const& user);
    /**
     * Called from the CTS TimeCallback after the system clock has been synced.
     * Stores the UTC offset, immediately updates the header clock, and starts
     * a 30-second periodic timer so the display stays current without GPS.
     */
    void onTimeSync(const struct tm *localTime, int32_t utcOffsetSec);

    /**
     * Trigger a fresh ADC read and return the updated battery percentage.
     * Blocks ~100 ms while the voltage-divider rail settles.
     */
    uint8_t getBatteryLevel()  { _battery.update(); return _battery.level();   }

    /**
     * Trigger a fresh ADC read and return the updated battery voltage in volts.
     * Blocks ~100 ms while the voltage-divider rail settles.
     */
    float   getBatteryVoltage() { _battery.update(); return _battery.voltage(); }

    /** Return the last cached battery percentage (updated every 30 s). */
    uint8_t cachedBatteryLevel()   const { return _battery.level();     }

    /** Return the last cached battery voltage in volts (updated every 30 s). */
    float   cachedBatteryVoltage() const { return _battery.voltage();   }

    /**
     * Return true when the TP4054 charger is believed to be active.
     * Detection: battery voltage > 4.15 V (TP4054 holds VBAT at 4.20 V during charging).
     * Updated every time BatteryMonitor::update() runs (every 30 s).
     */
    bool isCharging() const { return _battery.isCharging(); }

private:
    static void startDrawing(void* pvParameters);
    static void clockTimerCallback(TimerHandle_t xTimer);
    void showNotification(notification_def const& notification);

    TaskHandle_t  mDrawTask = nullptr;

    Display        _display;
    BatteryMonitor _battery;

    conn_state_def mBleState     = BLE_DISCONNECTED;
    bool           mGpsFixed     = false;
    bool           mLoraConnected = false;

    int32_t mUtcOffsetSeconds = INT32_MIN;  // INT32_MIN = not yet synced
    char    mMessage[32]      = {};         // pairing passcode / hint (BTC task writer)
    char    mTimestamp[8]     = {};         // "HH:MM\0" (clock timer writer)

    portMUX_TYPE  mHardwareLock;  // initialised in Hardware() via portMUX_INITIALIZE
    TimerHandle_t mClockTimer = nullptr;
};

extern Hardware Heltec;

#endif // HARDWARE_H_
