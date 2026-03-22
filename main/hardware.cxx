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

#include "hardware.h"

#include "bitmaps.h"
#include "bleservice.h"
#include "buzzer.h"
#include "lora.h"
#include "notificationservice.h"
#include <cinttypes>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/task.h>

static const char* TAG = "hardware";

// ── Constructor / Destructor ──────────────────────────────────────────────

Hardware::Hardware()
:   _display(ST7735_CS, ST7735_REST, ST7735_RS, ST7735_SCLK, ST7735_MOSI, ST7735_LED, VEXT_CTRL)
{
    portMUX_INITIALIZE(&mHardwareLock);
}

// ── begin ─────────────────────────────────────────────────────────────────

void Hardware::begin()
{
    _display.init();
    ESP_LOGI(TAG, "TFT initialized.");

    // ── FACTORY_LED — output, initially off ──────────────────────────────
    const gpio_config_t led_conf = {
        .pin_bit_mask  = 1ULL << FACTORY_LED,
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(static_cast<gpio_num_t>(FACTORY_LED), 0);

    // ── BatteryMonitor — configures ADC_CTRL (GPIO2) and ADC1_CH0 (GPIO1)
    // and performs an immediate read so the draw task has a valid level from
    // the first frame.
    _battery.init();

    Buzzer::init();

    // Spawn the draw task before starting the battery timer so the timer has
    // a valid task handle to notify.
    xTaskCreatePinnedToCore(&Hardware::startDrawing,
        "DrawTask", 10000, this, 3, &mDrawTask, 0);

    // Start the 30-second periodic battery timer now that mDrawTask is set.
    _battery.startTimer(mDrawTask, DRAW_BATTERY, DRAW_CHARGING);
}

// ── startDrawing ──────────────────────────────────────────────────────────

/* static */
void Hardware::startDrawing(void* pvParameters)
{
    ESP_LOGI(TAG, "Starting Draw task");
    Hardware *h = static_cast<Hardware*>(pvParameters);

    // Paint header bar then initial icons.
    h->_display.paintHeaderBackground();
    h->_display.showBLEState(h->mBleState);
    h->_display.showBatteryLevel(h->_battery.level(), h->_battery.isCharging());
    {
        char initMsg[sizeof(h->mMessage)];
        portENTER_CRITICAL(&h->mHardwareLock);
        memcpy(initMsg, h->mMessage, sizeof(initMsg));
        portEXIT_CRITICAL(&h->mHardwareLock);
        h->_display.standby(h->mBleState, initMsg);
    }

    while (true)
    {
        uint32_t bits = 0;
        xTaskNotifyWait(0u, 0xFFFFFFFFu, &bits, portMAX_DELAY);

        if (bits & DRAW_BATTERY)
        {
            // Blocking ADC read — safe here in the draw task (not timer task).
            h->_battery.update();
            const uint8_t level = h->_battery.level();
            h->_display.showBatteryLevel(level, h->_battery.isCharging());
            Ble.setBatteryLevel(level);
        }

        if (bits & DRAW_CHARGING)
        {
            // VBUS / charging state changed between timer ticks.
            h->_display.showBatteryLevel(h->_battery.level(), h->_battery.isCharging());
        }

        if (bits & DRAW_STATE)
        {
            h->_display.showBLEState(h->mBleState);
        }

        if (bits & DRAW_TIME)
        {
            char localTs[sizeof(h->mTimestamp)];
            portENTER_CRITICAL(&h->mHardwareLock);
            memcpy(localTs, h->mTimestamp, sizeof(localTs));
            portEXIT_CRITICAL(&h->mHardwareLock);
            h->_display.showTime(localTs);
        }

        if (bits & DRAW_GPS)
        {
            bool fixed;
            portENTER_CRITICAL(&h->mHardwareLock);
            fixed = h->mGpsFixed;
            portEXIT_CRITICAL(&h->mHardwareLock);
            h->_display.showGpsState(fixed);
        }

        if (bits & DRAW_LORA_MESH)
        {
            bool connected;
            portENTER_CRITICAL(&h->mHardwareLock);
            connected = h->mLoraConnected;
            portEXIT_CRITICAL(&h->mHardwareLock);
            h->_display.showLoraState(connected);
        }

        if (bits & (DRAW_NOTIFY | DRAW_STATE))
        {
            h->_display.showCallState(Notifications.isCallingNotification());

            notification_def callNotification;
            if (Notifications.takeCallingNotification(callNotification)) {
                h->showNotification(callNotification);
                h->glow(true);
                vTaskDelay(pdMS_TO_TICKS(15000));
                h->glow(false);
            }

            static constexpr size_t MAX_BATCH = 8;
            notification_def pending[MAX_BATCH];
            const size_t pendingCount =
                Notifications.takeAllPendingNotifications(pending, MAX_BATCH);

            for (size_t i = 0; i < pendingCount; i++) {
                h->showNotification(pending[i]);
                h->glow(true);
                vTaskDelay(pdMS_TO_TICKS(15000));
                h->glow(false);
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            char localMsg[sizeof(h->mMessage)];
            portENTER_CRITICAL(&h->mHardwareLock);
            memcpy(localMsg, h->mMessage, sizeof(localMsg));
            portEXIT_CRITICAL(&h->mHardwareLock);
            h->_display.standby(h->mBleState, localMsg);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (bits & DRAW_LORA)
        {
#if CONFIG_LORA_ENABLED
            MeshMessage msg = Lora.lastMessage();
            if (msg.valid)
            {
                Buzzer::play(msg.isAlert ? Buzzer::SoundType::ALERT
                                         : Buzzer::SoundType::LORA);
                h->_display.showLoraMessage(msg);
                h->glow(true);
                vTaskDelay(pdMS_TO_TICKS(msg.isAlert ? 15000 : 10000));
                h->glow(false);
                char localMsg[sizeof(h->mMessage)];
                portENTER_CRITICAL(&h->mHardwareLock);
                memcpy(localMsg, h->mMessage, sizeof(localMsg));
                portEXIT_CRITICAL(&h->mHardwareLock);
                h->_display.standby(h->mBleState, localMsg);
            }
#endif
        }

        if (bits & DRAW_LORA_POS) { /* silent — neighbour table updated in LoRa task */ }
        if (bits & DRAW_LORA_NODE) { /* silent — neighbour table updated in LoRa task */ }
    }

    ESP_LOGI(TAG, "Ending Draw task");
    h->mDrawTask = nullptr;
    vTaskDelete(nullptr);
}

// ── pairing ───────────────────────────────────────────────────────────────

void Hardware::pairing(const char* passcode)
{
    portENTER_CRITICAL(&mHardwareLock);
    strncpy(mMessage, passcode, sizeof(mMessage) - 1);
    mMessage[sizeof(mMessage) - 1] = '\0';
    portEXIT_CRITICAL(&mHardwareLock);
    setBLEConnectionState(BLE_PAIRING);
}

// ── setBLEConnectionState ─────────────────────────────────────────────────

void Hardware::setBLEConnectionState(conn_state_def state)
{
    // Only update state here — showBLEState() is called by the draw task to
    // avoid an SPI race between the BTC task and the draw task.
    mBleState = state;
    notifyDraw(DRAW_STATE);
}

// ── notifyDraw ────────────────────────────────────────────────────────────

void Hardware::notifyDraw(uint32_t events)
{
    if (mDrawTask != nullptr)
        xTaskNotify(mDrawTask, events, eSetBits);
}

// ── showNotification (private) ────────────────────────────────────────────

void Hardware::showNotification(notification_def const& notification)
{
    Buzzer::play(notification.isCall());
    _display.showNotification(notification);
}

// ── showTime ──────────────────────────────────────────────────────────────

void Hardware::showTime(const char* timestamp)
{
    portENTER_CRITICAL(&mHardwareLock);
    strncpy(mTimestamp, timestamp, sizeof(mTimestamp) - 1);
    mTimestamp[sizeof(mTimestamp) - 1] = '\0';
    portEXIT_CRITICAL(&mHardwareLock);
    notifyDraw(DRAW_TIME);
}

// ── showCallState ─────────────────────────────────────────────────────────

void Hardware::showCallState(bool active)
{
    _display.showCallState(active);
}

// ── showGpsState ──────────────────────────────────────────────────────────

void Hardware::showGpsState(bool fixed)
{
    portENTER_CRITICAL(&mHardwareLock);
    mGpsFixed = fixed;
    portEXIT_CRITICAL(&mHardwareLock);
    notifyDraw(DRAW_GPS);
}

// ── showLoraState ─────────────────────────────────────────────────────────

void Hardware::showLoraState(bool connected)
{
    portENTER_CRITICAL(&mHardwareLock);
    mLoraConnected = connected;
    portEXIT_CRITICAL(&mHardwareLock);
    notifyDraw(DRAW_LORA_MESH);
}

// ── showLoraMessage ───────────────────────────────────────────────────────

void Hardware::showLoraMessage(MeshMessage const& msg)
{
    _display.showLoraMessage(msg);
}

// ── showPositionMessage ───────────────────────────────────────────────────

void Hardware::showPositionMessage(MeshPosition const& pos)
{
    _display.showPositionMessage(pos);
}

// ── showNodeInfoMessage ───────────────────────────────────────────────────

void Hardware::showNodeInfoMessage(MeshUser const& user)
{
    _display.showNodeInfoMessage(user);
}

// ── glow ──────────────────────────────────────────────────────────────────

void Hardware::glow(bool on)
{
    gpio_set_level(static_cast<gpio_num_t>(FACTORY_LED), on ? 1 : 0);
}

// ── onTimeSync ────────────────────────────────────────────────────────────

void Hardware::onTimeSync(const struct tm *localTime, int32_t utcOffsetSec)
{
    portENTER_CRITICAL(&mHardwareLock);
    mUtcOffsetSeconds = utcOffsetSec;
    portEXIT_CRITICAL(&mHardwareLock);

    char ts[sizeof(mTimestamp)];
    snprintf(ts, sizeof(ts), "%2d:%02d", localTime->tm_hour, localTime->tm_min);
    showTime(ts);

    if (mClockTimer == nullptr)
    {
        mClockTimer = xTimerCreate("Clock", pdMS_TO_TICKS(30000), pdTRUE,
                                   this, clockTimerCallback);
        if (mClockTimer != nullptr)
            xTimerStart(mClockTimer, 0);
        else
            ESP_LOGW(TAG, "Failed to create clock timer");
    }
    else
    {
        xTimerReset(mClockTimer, 0);
    }
}

// ── clockTimerCallback ────────────────────────────────────────────────────

/* static */
void Hardware::clockTimerCallback(TimerHandle_t xTimer)
{
    Hardware *h = static_cast<Hardware *>(pvTimerGetTimerID(xTimer));

    int32_t offset;
    portENTER_CRITICAL(&h->mHardwareLock);
    offset = h->mUtcOffsetSeconds;
    portEXIT_CRITICAL(&h->mHardwareLock);

    time_t utc = time(nullptr);
    time_t localEpoch = (offset != INT32_MIN)
                      ? (utc + static_cast<time_t>(offset))
                      : utc;

    struct tm localTm{};
    gmtime_r(&localEpoch, &localTm);

    char ts[8];
    snprintf(ts, sizeof(ts), "%2d:%02d", localTm.tm_hour, localTm.tm_min);
    h->showTime(ts);
}

/* extern */
Hardware Heltec;
