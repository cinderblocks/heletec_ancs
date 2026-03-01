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

#include "hardware.h"

#include "bitmaps.h"
#include "bleservice.h"
#include "notificationservice.h"

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

    pinMode(VBAT_READ, INPUT);
    pinMode(FACTORY_LED, OUTPUT);

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
        xTaskNotifyWait(0xFFFFFFFFu, 0xFFFFFFFFu, &bits, portMAX_DELAY);

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

        // Update GPS time and icon in header — safe here because we're in the draw task.
        // showTime() and showGpsState() no longer write SPI directly, eliminating the
        // SPI race where the GPS task and draw task both wrote SPI simultaneously.
        if (bits & DRAW_GPS)
        {
            char localTs[sizeof(h->mTimestamp)];
            portENTER_CRITICAL(&h->mHardwareLock);
            memcpy(localTs, h->mTimestamp, sizeof(localTs));
            portEXIT_CRITICAL(&h->mHardwareLock);

            if (localTs[0] != '\0')
            {
                h->mDisplay.drawStr(0, 6, localTs, Font_7x10, TFT::Color::WHITE, HEADER_COLOR);
            }
            if (h->mGpsState)
            {
                h->drawIcon(h->mDisplay.width()-50, 1, Bitmaps::GPS);
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

            size_t count = Notifications.getNotificationCount();
            for (size_t i = 0; i < count; i++) {
                // takeNotificationByIndex atomically copies the notification and
                // marks it as showed under the mutex, so the slot is safe to reuse
                // immediately — no stale pointer is held across the 15 s delay.
                notification_def notification;
                if (Notifications.takeNotificationByIndex(i, notification)) {
                    h->showNotification(notification);
                    h->glow(true);
                    vTaskDelay(pdMS_TO_TICKS(15000));
                    h->glow(false);
                }
            }
            // Always refresh body text to reflect current BLE state after processing
            h->standby();
        }
    }
    ESP_LOGI(TAG, "Ending Draw task");
    h->mDrawTask = nullptr;
    vTaskDelete(nullptr);
}

void Hardware::pairing(String const& passcode)
{
    // mMessage is read by standby() in the draw task, so guard the write with the
    // hardware spinlock to prevent a data race on the dual-core ESP32-S3.
    portENTER_CRITICAL(&mHardwareLock);
    strncpy(mMessage, passcode.c_str(), sizeof(mMessage) - 1);
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
    mDisplay.drawStr(0, 21, AppList.getDisplayName(notification.type),
        Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    mDisplay.drawStr(mDisplay.width() - 38, 21, timestamp,
        Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    mDisplay.drawStr(0, 44, notification.title, Font_11x18, TFT::Color::BLACK, TFT::Color::WHITE);
    mDisplay.drawStr(0, 62, notification.message, Font_11x18, TFT::Color::BLACK, TFT::Color::WHITE);
}

uint8_t Hardware::getBatteryLevel()
{
    pinMode(ADC_CTRL, INPUT_PULLUP);
    delay(100); // wait for GPIO stabilization
    uint16_t analogValue = analogRead(VBAT_READ);
    pinMode(ADC_CTRL, INPUT_PULLDOWN);

    //const float voltage = analogValue * 0.0037f;
    return constrain(((analogValue - 680.f) / 343.f) * 100, 0, 100);
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
            mDisplay.drawStr(0, 36, "Pairing", Font_11x18, TFT::Color::WHITE);
            mDisplay.drawStr(0, 60, localMsg, Font_11x18, TFT::Color::WHITE);
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

void Hardware::showTime(String const& timestamp)
{
    // mTimestamp is read by the draw task under DRAW_GPS, so guard the write with
    // the hardware spinlock to prevent a data race on the dual-core ESP32-S3.
    portENTER_CRITICAL(&mHardwareLock);
    strncpy(mTimestamp, timestamp.c_str(), sizeof(mTimestamp) - 1);
    mTimestamp[sizeof(mTimestamp) - 1] = '\0';
    portEXIT_CRITICAL(&mHardwareLock);
    notifyDraw(DRAW_GPS);
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

void Hardware::showGpsState(bool connected)
{
    if (connected == mGpsState) { return; }
    // Only update state here; the draw task handles the actual icon update under
    // DRAW_GPS, eliminating the SPI race with the GPS task on the other core.
    mGpsState = connected;
    notifyDraw(DRAW_GPS);
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

void Hardware::glow(bool on)
{
    digitalWrite(FACTORY_LED, on ? HIGH : LOW);
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
