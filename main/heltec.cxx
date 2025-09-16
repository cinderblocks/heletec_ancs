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

#include "heltec.h"

#include "bitmaps.h"
#include "notificationservice.h"

static const char* TAG = "heltec";

Heltec_ESP32::Heltec_ESP32()
:   mDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED)
,   mBleState(BLE_DISCONNECTED)
{ }

/* virtual */
Heltec_ESP32::~Heltec_ESP32() = default;

void Heltec_ESP32::begin()
{
    //pinMode(LED, OUTPUT);
    Serial.begin(115200);
    Serial.flush();
    delay(50);
    ESP_LOGI(TAG, "Serial initialized.");

    mDisplay.init();
    mDisplay.drawXbm(0, 0, 128, 64, Bitmaps::Gnu_128x64);
    mDisplay.display();
    ESP_LOGI(TAG, "OLED initialized.");

    pinMode(ADC_CTRL, OUTPUT);
    digitalWrite(ADC_CTRL, LOW);
    pinMode(VBAT_READ, INPUT);
}

void Heltec_ESP32::loop()
{
    for (auto it = Notifications.getNotificationList().begin(); it != Notifications.getNotificationList().end(); ++it) {
        if (Notifications.isCallingNotification()) { break; }
        if (!it->second.showed && it->second.isComplete) {
            showNotification(it->second);
            it->second.showed = true;
            delay(15000);
        }
    }
    if (Notifications.isCallingNotification()) {
        notification_def notification = Notifications.getCallingNotification();
        if (notification.isComplete) {
            showNotification(notification);
            delay(1000);
        }
    } else {
        standby();
    }

    delay(100);
}

void Heltec_ESP32::setBLEConnectionState(conn_state_def state)
{
    mBleState = state;
}

void Heltec_ESP32::pairing(String const& passcode) {
    mMessage = passcode;
    setBLEConnectionState(BLE_PAIRING);
}

void Heltec_ESP32::showNotification(notification_def const& notification)
{
    char timebuf[9];
    strftime(timebuf, sizeof(timebuf), "%R",std::localtime(&notification.time));
    mDisplay.clear();
    drawHeader();
    mDisplay.setFont(ArialMT_Plain_10);
    mDisplay.drawString(0, 0, AppList.getDisplayName(notification.type));
    mDisplay.drawString(0, 10, timebuf);
    mDisplay.setFont(ArialMT_Plain_16);
    mDisplay.drawString(0, 24, notification.title);
    mDisplay.drawString(0, 40, notification.message);
    mDisplay.display();
}

void Heltec_ESP32::standby()
{
    mDisplay.clear();
    drawHeader();
    mDisplay.setFont(ArialMT_Plain_16);
    switch (mBleState) {
        case BLE_SERVER_CONNECTED:
            mDisplay.drawString(0, 40, "Connected.");
            break;
        case BLE_CONNECTED:
            mDisplay.drawString(0, 40, "Standby.");
            break;
        case BLE_PAIRING:
            mDisplay.drawString(0, 20, "Pairing...");
            mDisplay.drawString(0, 36, "Passcode " + mMessage);
            break;
        case BLE_DISCONNECTED:
            mDisplay.drawString(0, 40, "Disconnected.");
            break;
    }

    mDisplay.display();
}

void Heltec_ESP32::drawHeader()
{
    mDisplay.drawIco16x16(112, 0, reinterpret_cast<const char*>(Bitmaps::Battery_100));
    switch (mBleState) {
        case BLE_CONNECTED:
            mDisplay.drawIco16x16(95,0, reinterpret_cast<const char *>(Bitmaps::BluetoothRound));
            break;
        case BLE_SERVER_CONNECTED:
        case BLE_PAIRING:
            mDisplay.drawIco16x16(95,0, reinterpret_cast<const char *>(Bitmaps::Mqtt));
            break;
        case BLE_DISCONNECTED:
            mDisplay.drawIco16x16(95,0, reinterpret_cast<const char *>(Bitmaps::Bluetooth));
            break;
    }
}

float Heltec_ESP32::getBatteryVoltage()
{
    const uint16_t analogValue = analogRead(VBAT_READ);
    const float voltage = analogValue * 0.00403532794741887;
    return voltage;
}

/* extern */
Heltec_ESP32 Heltec;
