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
#include "notificationservice.h"
#include <algorithm>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <freertos/task.h>

static const char* TAG = "hardware";

Hardware::Hardware()
:   mDisplay(ST7735_CS, ST7735_REST, ST7735_RS, ST7735_SCLK, ST7735_MOSI, ST7735_LED, VEXT_CTRL)
{
    portMUX_INITIALIZE(&mHardwareLock);
}

/* virtual */
Hardware::~Hardware() = default;

void Hardware::begin()
{
    mDisplay.init();
    blank();
    mDisplay.drawXbm(16, 10, 128,64, Bitmaps::Gnu_128x64, TFT::Color::BLUE);
    ESP_LOGI(TAG, "TFT initialized.");

    // ── FACTORY_LED — output, initially off ───────────────────────────────
    {
        const gpio_config_t led_conf = {
            .pin_bit_mask  = 1ULL << FACTORY_LED,
            .mode          = GPIO_MODE_OUTPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_DISABLE,
        };
        gpio_config(&led_conf);
        gpio_set_level(static_cast<gpio_num_t>(FACTORY_LED), 0);
    }

    // ── ADC_CTRL (GPIO2) — initially pull-down (divider disabled) ─────────
    {
        const gpio_config_t adc_ctrl_conf = {
            .pin_bit_mask  = 1ULL << ADC_CTRL,
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_ENABLE,
            .intr_type     = GPIO_INTR_DISABLE,
        };
        gpio_config(&adc_ctrl_conf);
    }

    // ── ADC1 / VBAT_READ (GPIO1 = ADC1_CH0) ──────────────────────────────
    // Create the oneshot ADC unit once here; reused by every getBatteryLevel() call.
    {
        const adc_oneshot_unit_init_cfg_t adc_cfg = {
            .unit_id  = ADC_UNIT_1,
            .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        esp_err_t err = adc_oneshot_new_unit(&adc_cfg, &mAdcHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_new_unit: %s", esp_err_to_name(err));
        } else {
            const adc_oneshot_chan_cfg_t chan_cfg = {
                .atten    = ADC_ATTEN_DB_12,   // 0–3.9 V input range
                .bitwidth = ADC_BITWIDTH_12,    // 0–4095 raw output
            };
            err = adc_oneshot_config_channel(mAdcHandle, ADC_CHANNEL_0, &chan_cfg);
            if (err != ESP_OK)
                ESP_LOGE(TAG, "adc_oneshot_config_channel: %s", esp_err_to_name(err));
        }
    }

    mBatteryTimer = xTimerCreate("Battery", pdMS_TO_TICKS(30000), pdTRUE, nullptr, batteryTimerCallback);
    if (mBatteryTimer == nullptr)
    {
        ESP_LOGW(TAG, "Failed to create BatteryTimer");
    }
    else if (xTimerStart(mBatteryTimer, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Failed to start BatteryTimer");
    }

    // Read battery level NOW, before spawning the draw task.  On the dual-core
    // ESP32-S3 the draw task can start on the other core immediately after
    // xTaskCreatePinnedToCore returns.  If we called showBatteryLevel() here
    // instead, both cores would be issuing SPI commands simultaneously.
    // The draw task's init block will call showBatteryLevel(mBatteryLevel) for us.
    mBatteryLevel = getBatteryLevel();

    xTaskCreatePinnedToCore(&Hardware::startDrawing,
        "DrawTask", 10000, this, 3, &mDrawTask, 0);
}

/* static */
void Hardware::startDrawing(void* pvParameters)
{
    ESP_LOGI(TAG, "Starting Draw task");
    Hardware *h = static_cast<Hardware*>(pvParameters);

    h->mDisplay.fillRectangle(0, 0, h->mDisplay.width(), 20, HEADER_COLOR);
    h->showBLEState(h->mBleState);
    h->showBatteryLevel(h->mBatteryLevel); // mBatteryLevel was set in begin() before this task started
    h->standby();

    while (true)
    {
        uint32_t bits = 0;
        // ulBitsToClearOnEntry = 0: do NOT clear pending bits before reading.
        // With 0xFFFFFFFFu here (the former value) any bits posted while the task
        // was busy — e.g. DRAW_GPS fired during the 15 s notification vTaskDelay —
        // were zeroed before xTaskNotifyWait could return them, silently dropping
        // every GPS display update that arrived during a notification.
        // ulBitsToClearOnExit = 0xFFFFFFFFu: clear all bits after reading so the
        // next call starts clean.
        xTaskNotifyWait(0u, 0xFFFFFFFFu, &bits, portMAX_DELAY);

        if (bits & DRAW_BATTERY)
        {
            // Safe to block here — we are in the draw task, not the timer service task.
            uint8_t level = h->getBatteryLevel();
            if (level != h->mBatteryLevel)
            {
                h->mBatteryLevel = level;
                h->showBatteryLevel(level);
                Ble.setBatteryLevel(level);
            }
        }

        // Update BLE icon in header — safe here because we're in the draw task.
        // setBLEConnectionState() no longer calls showBLEState() directly, eliminating
        // the SPI race where the BTC task (BLE callbacks) and draw task both wrote SPI.
        if (bits & DRAW_STATE)
        {
            h->showBLEState(h->mBleState);
        }

        // Update clock display in header — safe here because we're in the draw task.
        // Time is now sourced from the iOS Current Time Service (CTS) via onTimeSync(),
        // not GPS.  showTime() writes mTimestamp under the spinlock; we copy it here.
        if (bits & DRAW_TIME)
        {
            char localTs[sizeof(h->mTimestamp)];
            portENTER_CRITICAL(&h->mHardwareLock);
            memcpy(localTs, h->mTimestamp, sizeof(localTs));
            portEXIT_CRITICAL(&h->mHardwareLock);

            if (localTs[0] != '\0')
            {
                h->mDisplay.drawStr(0, 6, localTs, Font_7x10, TFT::Color::WHITE, HEADER_COLOR);
            }
        }

        if (bits & DRAW_GPS)
        {
            bool fixed;
            portENTER_CRITICAL(&h->mHardwareLock);
            fixed = h->mGpsFixed;
            portEXIT_CRITICAL(&h->mHardwareLock);

            if (fixed)
            {
                h->drawIcon(h->mDisplay.width()-50, 1, Bitmaps::GPS, TFT::Color::WHITE);
            }
            else
            {
                h->mDisplay.fillRectangle(h->mDisplay.width()-50, 1, 16, 16, HEADER_COLOR);
            }
        }

        if (bits & (DRAW_NOTIFY | DRAW_STATE))
        {
            // Update call icon in header bar first
            h->showCallState(Notifications.isCallingNotification());

            // Show caller ID once when the call first arrives.  takeCallingNotification
            // atomically copies and marks showed=true; subsequent Modified re-fetches
            // (iOS fires one per second) leave showed=true so this block is skipped.
            notification_def callNotification;
            if (Notifications.takeCallingNotification(callNotification)) {
                h->showNotification(callNotification);
                h->glow(true);
                vTaskDelay(pdMS_TO_TICKS(15000));
                h->glow(false);
            }

            size_t count = Notifications.getNotificationCount();
            notification_def notification;
            for (size_t i = 0; i < count; i++) {
                // takeNotificationByIndex atomically copies the notification and
                // marks it as showed under the mutex, so the slot is safe to reuse
                // immediately — no stale pointer is held across the 15 s delay.
                if (Notifications.takeNotificationByIndex(i, notification)) {
                    h->showNotification(notification);
                    h->glow(true);
                    vTaskDelay(pdMS_TO_TICKS(15000));
                    h->glow(false);
                }
            }
            // Always refresh body text to reflect current BLE state after processing
            h->standby();

            // Yield to IDLE0 after display work.  DrawTask (priority 3) never
            // preempts for lower-priority tasks, so IDLE0 only runs when DrawTask
            // blocks.  Without this delay, rapid DRAW_NOTIFY / DRAW_STATE events
            // (e.g. iOS firing EventIDNotificationModified every second for active
            // calls) keep the task-notification bits non-zero, causing
            // xTaskNotifyWait to return immediately on the next iteration and
            // preventing IDLE0 from ever running — which triggers the task WDT.
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    ESP_LOGI(TAG, "Ending Draw task");
    h->mDrawTask = nullptr;
    vTaskDelete(nullptr);
}

void Hardware::pairing(const char* passcode)
{
    portENTER_CRITICAL(&mHardwareLock);
    strncpy(mMessage, passcode, sizeof(mMessage) - 1);
    mMessage[sizeof(mMessage) - 1] = '\0';
    portEXIT_CRITICAL(&mHardwareLock);
    setBLEConnectionState(BLE_PAIRING);
}

void Hardware::setBLEConnectionState(conn_state_def state)
{
    // Only update the state variable here. Do NOT call showBLEState() directly —
    // this function is called from BTC task callbacks (ServerCallback::onDisconnect,
    // SecurityCallback::onAuthenticationComplete, etc.) which run on a different core
    // from the draw task.  Calling showBLEState() here would race the draw task's
    // SPI bus access on the dual-core ESP32-S3.  The DRAW_STATE bit tells the draw
    // task to call showBLEState() at its next wakeup.
    mBleState = state;
    notifyDraw(DRAW_STATE);
}

void Hardware::notifyDraw(uint32_t events)
{
    if (mDrawTask != nullptr)
    {
        xTaskNotify(mDrawTask, events, eSetBits);
    }
}

void Hardware::showNotification(notification_def const& notification)
{
    char timestamp[8];
    struct tm timeinfo;
    localtime_r(&notification.time, &timeinfo);
    strftime(timestamp, sizeof(timestamp), "%R", &timeinfo);
    blank();
    mDisplay.fillRectangle(0, 20, mDisplay.width(), mDisplay.height() - 20, TFT::Color::WHITE);
    mDisplay.drawStr(0, 21, AppList.getDisplayName(notification.bundleId),
        Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    mDisplay.drawStr(mDisplay.width() - 38, 21, timestamp,
        Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    mDisplay.drawStr(0, 44, notification.title, Font_11x18, TFT::Color::BLACK, TFT::Color::WHITE);
    mDisplay.drawStr(0, 62, notification.message, Font_11x18, TFT::Color::BLACK, TFT::Color::WHITE);
}

uint8_t Hardware::getBatteryLevel()
{
    // Enable battery voltage divider by driving ADC_CTRL high via internal pull-up.
    gpio_set_pull_mode(static_cast<gpio_num_t>(ADC_CTRL), GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(100));  // wait for rail to settle

    int raw = 0;
    adc_oneshot_read(mAdcHandle, ADC_CHANNEL_0, &raw);

    // Disable divider — pull ADC_CTRL low to stop current through the resistor network.
    gpio_set_pull_mode(static_cast<gpio_num_t>(ADC_CTRL), GPIO_PULLDOWN_ONLY);

    return static_cast<uint8_t>(
        std::clamp(((raw - 680.f) / 343.f) * 100.f, 0.f, 100.f));
}

void Hardware::standby()
{
    blank();
    switch (mBleState) {
        case BLE_SERVER_CONNECTED:
            mDisplay.drawStr(0, 62, "Connected", Font_11x18, TFT::Color::WHITE);
            break;
        case BLE_CONNECTED:
            mDisplay.drawStr(0, 62, "Standby", Font_11x18, TFT::Color::WHITE);
            break;
        case BLE_PAIRING: {
            // mMessage is written from the BTC task (pairing()), so take a local
            // copy under the spinlock before passing it to drawStr.
            char localMsg[sizeof(mMessage)];
            portENTER_CRITICAL(&mHardwareLock);
            memcpy(localMsg, mMessage, sizeof(localMsg));
            portEXIT_CRITICAL(&mHardwareLock);
            // "Forget on iOS" = bond-mismatch hint; numeric codes = passkey display.
            // Use a smaller font for the hint so it fits without clipping.
            if (strncmp(localMsg, "Forget", 6) == 0) {
                mDisplay.drawStr(0, 28, "Pairing failed:", Font_7x10, TFT::Color::WHITE);
                mDisplay.drawStr(0, 42, "iOS Settings >", Font_7x10, TFT::Color::WHITE);
                mDisplay.drawStr(0, 56, "Bluetooth >", Font_7x10, TFT::Color::WHITE);
                mDisplay.drawStr(0, 70, "Forget device", Font_7x10, TFT::Color::WHITE);
            } else {
                mDisplay.drawStr(0, 36, "Pairing", Font_11x18, TFT::Color::WHITE);
                mDisplay.drawStr(0, 60, localMsg, Font_11x18, TFT::Color::WHITE);
            }
            break;
        }
        case BLE_DISCONNECTED:
            mDisplay.drawStr(0, 62, "Disconnected", Font_11x18, TFT::Color::WHITE);
            break;
    }
}

void Hardware::blank()
{
    mDisplay.fillRectangle(0, 20, mDisplay.width(), mDisplay.height() - 20, TFT::Color::BLACK);
}

void Hardware::drawIcon(const uint16_t x, const uint16_t y, const uint8_t* xbm, uint16_t color)
{
    mDisplay.drawXbm(x, y, 16, 16, xbm, color, HEADER_COLOR);
}

void Hardware::showTime(const char* timestamp)
{
    portENTER_CRITICAL(&mHardwareLock);
    strncpy(mTimestamp, timestamp, sizeof(mTimestamp) - 1);
    mTimestamp[sizeof(mTimestamp) - 1] = '\0';
    portEXIT_CRITICAL(&mHardwareLock);
    notifyDraw(DRAW_TIME);
}

void Hardware::showBLEState(conn_state_def state)
{
    switch (state) {
        case BLE_CONNECTED:
            drawIcon(mDisplay.width()-33, 1, Bitmaps::BluetoothRound);
            break;
        case BLE_SERVER_CONNECTED:
        case BLE_PAIRING:
            drawIcon(mDisplay.width()-33, 1, Bitmaps::Mqtt);
            break;
        case BLE_DISCONNECTED:
            drawIcon(mDisplay.width()-33, 1, Bitmaps::Bluetooth);
            break;
    }
}

void Hardware::showBatteryLevel(uint8_t percent)
{
    if (percent > 75) {
        drawIcon(mDisplay.width()-16, 1, Bitmaps::Battery_100);
    } else if (percent > 50) {
        drawIcon(mDisplay.width()-16, 1, Bitmaps::Battery_66);
    } else if (percent > 25) {
        drawIcon(mDisplay.width()-16, 1, Bitmaps::Battery_33);
    } else {
        drawIcon(mDisplay.width()-16, 1, Bitmaps::Battery_0, TFT::Color::RED);
    }
}

void Hardware::onTimeSync(const struct tm *localTime, int32_t utcOffsetSec)
{
    // Store the UTC offset so the periodic timer can compute local time from
    // the system clock.  Write under the spinlock for cross-core safety.
    portENTER_CRITICAL(&mHardwareLock);
    mUtcOffsetSeconds = utcOffsetSec;
    portEXIT_CRITICAL(&mHardwareLock);

    // Show the initial time immediately using the value iOS handed us — no need
    // to read the system clock since it was just set by _applyCurrentTime.
    char ts[sizeof(mTimestamp)];
    snprintf(ts, sizeof(ts), "%2d:%02d", localTime->tm_hour, localTime->tm_min);
    showTime(ts);   // thread-safe; posts DRAW_TIME to the draw task

    // Start the 30-second periodic timer so the display stays current.
    // 30 s ensures the minute digit turns over within half a minute of the
    // actual change.  The timer is started only once; subsequent CTS syncs
    // reset it so the next tick is 30 s from the fresh sync.
    if (mClockTimer == nullptr)
    {
        mClockTimer = xTimerCreate("Clock", pdMS_TO_TICKS(30000), pdTRUE,
                                   this, clockTimerCallback);
        if (mClockTimer != nullptr)
        {
            xTimerStart(mClockTimer, 0);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to create clock timer");
        }
    }
    else
    {
        // Reset the timer so the next tick is 30 s from this fresh sync.
        xTimerReset(mClockTimer, 0);
    }
}

/* static */
void Hardware::clockTimerCallback(TimerHandle_t xTimer)
{
    Hardware *h = static_cast<Hardware *>(pvTimerGetTimerID(xTimer));

    int32_t offset;
    portENTER_CRITICAL(&h->mHardwareLock);
    offset = h->mUtcOffsetSeconds;
    portEXIT_CRITICAL(&h->mHardwareLock);

    // System clock holds UTC (set by _applyCurrentTime via settimeofday).
    // Add the stored offset to recover iOS local time.
    // Edge case: if offset is INT32_MIN, _applyCurrentTime fell back to treating
    // local time as UTC, so the system clock already holds local time — add 0.
    time_t utc = time(nullptr);
    time_t localEpoch = (offset != INT32_MIN)
                      ? (utc + static_cast<time_t>(offset))
                      : utc;

    struct tm localTm{};
    gmtime_r(&localEpoch, &localTm);

    char ts[8];
    snprintf(ts, sizeof(ts), "%2d:%02d", localTm.tm_hour, localTm.tm_min);
    h->showTime(ts);    // writes mTimestamp + posts DRAW_TIME
}

void Hardware::showCallState(bool active)
{
    if (active == mCallState) { return; }

    mCallState = active;
    if (active)
    {
        drawIcon(mDisplay.width()-67, 1, Bitmaps::PhoneCall, TFT::Color::GREEN);
    }
    else
    {
        mDisplay.fillRectangle(mDisplay.width()-67, 1, 16, 16, HEADER_COLOR);
    }
}

void Hardware::showGpsState(bool fixed)
{
    portENTER_CRITICAL(&mHardwareLock);
    mGpsFixed = fixed;
    portEXIT_CRITICAL(&mHardwareLock);
    notifyDraw(DRAW_GPS);
}

void Hardware::glow(bool on)
{
    gpio_set_level(static_cast<gpio_num_t>(FACTORY_LED), on ? 1 : 0);
}

/* static */
void Hardware::batteryTimerCallback(TimerHandle_t xTimer)
{
    // Just set the battery bit — the actual ADC read (which needs a 100 ms GPIO
    // stabilisation delay) happens in the draw task, not here in the timer service task.
    Heltec.notifyDraw(DRAW_BATTERY);
}

/* extern */
Hardware Heltec;
