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

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <esp_adc/adc_oneshot.h>
#include <time.h>
#include <climits>
#include "tft.h"

struct notification_def;

enum conn_state_def
{
    BLE_DISCONNECTED = 0,
    BLE_PAIRING,
    BLE_SERVER_CONNECTED,
    BLE_CONNECTED
};

using PairButtonCallback = void (*)();

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
    static constexpr uint8_t BUTTON     = 0;
    static constexpr uint8_t FACTORY_LED = 18;
    static constexpr uint8_t VBAT_READ  = 1;
    static constexpr uint8_t BUZZER     = 45;
    /**
     * TP4054 CHRG pin — active-LOW, open-drain output.
     * LOW  = battery is actively charging.
     * HIGH = not charging (charge complete or no USB power).
     * The pin has an on-board pull-up, but we also enable the internal pull-up
     * as a safety net.
     */
    static constexpr uint8_t CHARGE     = 6;

    static constexpr uint16_t HEADER_COLOR = 0x3190;

    // Task-notification event bits for the draw task
    static constexpr uint32_t DRAW_NOTIFY  = (1u << 0); // new notification ready
    static constexpr uint32_t DRAW_STATE   = (1u << 1); // BLE state changed
    static constexpr uint32_t DRAW_BATTERY = (1u << 2); // battery level check
    static constexpr uint32_t DRAW_TIME    = (1u << 3); // CTS clock updated
    static constexpr uint32_t DRAW_GPS     = (1u << 4); // GPS fix state changed
    static constexpr uint32_t DRAW_LORA      = (1u << 5); // Meshtastic text message
    static constexpr uint32_t DRAW_LORA_POS  = (1u << 6); // Meshtastic position received
    static constexpr uint32_t DRAW_LORA_NODE = (1u << 7); // Meshtastic nodeinfo received
    static constexpr uint32_t DRAW_LORA_MESH = (1u << 8); // LoRa mesh connection state changed
    static constexpr uint32_t DRAW_CHARGING  = (1u << 9); // USB charge state changed

    Hardware();
    ~Hardware();
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

    uint8_t getBatteryLevel();

    /**
     * Read the current battery voltage in volts via the ADC voltage divider.
     * Reads ADC once (averaged), updates both level and voltage caches.
     * Thread-safe: may block ~100 ms while the rail settles.
     */
    float getBatteryVoltage();

    /** Return the last cached battery percentage (updated every 30 s). */
    uint8_t cachedBatteryLevel()  const { return mBatteryLevel;  }

    /** Return the last cached battery voltage in volts (updated every 30 s). */
    float   cachedBatteryVoltage() const { return mBatteryVoltage; }

    /**
     * Return true when the TP4054 CHRG pin (GPIO6) is LOW, meaning the battery
     * is being actively charged via USB.  Updated every time _updateBatteryCache()
     * runs (every 30 s, or on demand via getBatteryLevel/getBatteryVoltage).
     * Note: the battery-level percentage is unreliable while charging because the
     * charger holds the voltage above the battery's resting level.
     */
    bool isCharging() const { return mIsCharging; }

private:
    static void startDrawing(void* pvParameters);
    static void batteryTimerCallback(TimerHandle_t xTimer);
    static void clockTimerCallback(TimerHandle_t xTimer);
    void showNotification(notification_def const& notification);

    /** Read ADC once and update mBatteryLevel + mBatteryVoltage. */
    void _updateBatteryCache();

    void blank();
    void drawIcon(uint16_t x, uint16_t y, uint8_t const* xbm, uint16_t color = TFT::Color::WHITE);
    void showBLEState(conn_state_def state);
    void showBatteryLevel(uint8_t percent);
    void standby();

    TaskHandle_t mDrawTask = nullptr;

    TFT mDisplay;
    conn_state_def mBleState = BLE_DISCONNECTED;
    bool mCallState = false;
    bool mGpsFixed  = false;
    bool mLoraConnected = false;
    uint8_t mBatteryLevel   = 0;
    float   mBatteryVoltage = 0.0f;
    bool    mIsCharging     = false;
    int32_t mUtcOffsetSeconds = INT32_MIN;  // INT32_MIN = not yet synced
    // Fixed-size arrays instead of String to allow safe writes from non-draw tasks
    // (BTC task writes mMessage; GPS task writes mTimestamp).  Both are protected
    // by mHardwareLock when crossing task boundaries.
    char mMessage[32]   = {};
    char mTimestamp[8]  = {};
    portMUX_TYPE mHardwareLock; // initialized in Hardware() via portMUX_INITIALIZE
    TimerHandle_t mBatteryTimer = nullptr;
    TimerHandle_t mClockTimer   = nullptr;
    adc_oneshot_unit_handle_t mAdcHandle = nullptr;
};

extern Hardware Heltec;

#endif // HARDWARE_H_
