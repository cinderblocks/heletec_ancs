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
{ }

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

    h->standby();

    while (true)
    {
        BaseType_t result = xTaskNotifyWait(0x00, 0x00, nullptr, portMAX_DELAY);
        if (result == pdPASS)
        {
            for (auto it = Notifications.getNotificationList().begin(); it != Notifications.getNotificationList().end(); ++it) {
                if (Notifications.isCallingNotification()) { break; }
                if (!it->second.showed && it->second.isComplete) {
                    h->showNotification(it->second);
                    it->second.showed = true;
                    h->glow(true);
                    vTaskDelay(15000 / portTICK_PERIOD_MS);
                    h->glow(false);

                }
            }
            if (Notifications.isCallingNotification()) {
                notification_def notification = Notifications.getCallingNotification();
                if (notification.isComplete) {
                    h->showNotification(notification);
                    h->glow(true);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    h->glow(false);
                }
            }
            h->standby();
        }
    }
    ESP_LOGI(TAG, "Ending Draw task");
    h->mDrawTask = nullptr;
    vTaskDelete(nullptr);
}

void Hardware::pairing(String const& passcode)
{
    mMessage = passcode;
    setBLEConnectionState(BLE_PAIRING);
}

void Hardware::setBLEConnectionState(conn_state_def state)
{
    mBleState = state;
    showBLEState(state);
    ::xTaskNotifyGive(mDrawTask);
}

void Hardware::showNotification(notification_def const& notification)
{
    char timestamp[8];
    strftime(timestamp, sizeof(timestamp), "%R",std::localtime(&notification.time));
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
        case BLE_PAIRING:
            mDisplay.drawStr(0, 36, "Pairing", Font_11x18, TFT::Color::WHITE);
            mDisplay.drawStr(0, 60, mMessage, Font_11x18, TFT::Color::WHITE);
            break;
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
    mDisplay.drawStr(0, 6, timestamp, Font_7x10, TFT::Color::WHITE, HEADER_COLOR);
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
    if (connected == mGpsState) { return ; }

    mGpsState = connected;
    if (connected)
    {
        drawIcon(mDisplay.width()-50, 1, Bitmaps::GPS);
    }
    else
    {
        mDisplay.fillRectangle(mDisplay.width()-50, 1, 16, 16, HEADER_COLOR);
    }
}

void Hardware::glow(bool on)
{
    digitalWrite(FACTORY_LED, on ? HIGH : LOW);
}

/* static */
void Hardware::batteryTimerCallback(TimerHandle_t xTimer)
{
    uint8_t level = Heltec.getBatteryLevel();
    if (level != Heltec.mBatteryLevel)
    {
        Heltec.mBatteryLevel = level;
        Heltec.showBatteryLevel(level);
        Ble.setBatteryLevel(level);
    }
}

/* extern */
Hardware Heltec;
