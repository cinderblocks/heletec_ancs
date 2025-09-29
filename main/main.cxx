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

#include "sdkconfig.h"
#include "applist.h"
#include "hardware.h"
#include "bleservice.h"
#include "gps.h"
#include "notificationservice.h"
#include "task.h"

#include <esp_log.h>
#include <cinttypes>

static const char* TAG = "main";
RTC_DATA_ATTR static int boot_count = 0;

class MainServerCallback final : public ANCSServiceServerCallback {
public:
    explicit MainServerCallback(Hardware *ht) : _ht(ht) { }
    virtual ~MainServerCallback() = default;
    void onConnect() override {
        _ht->setBLEConnectionState(BLE_CONNECTED);
    }
    void onDisconnect() override {
        _ht->setBLEConnectionState(BLE_DISCONNECTED);
    }
private:
    Hardware* _ht;
};

class MainClientCallback final : public ANCSServiceClientCallback {
public:
    explicit MainClientCallback(Hardware  *ht) : _ht(ht) { }
    virtual ~MainClientCallback() = default;
    void onConnect() override {
        ESP_LOGI(TAG, "Client Connected");
        _ht->setBLEConnectionState(BLE_CONNECTED);
    }
    void onDisconnect() override {
        _ht->setBLEConnectionState(BLE_DISCONNECTED);
    }
private:
    Hardware* _ht;
};

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Boot count: %d", boot_count++);

    initArduino();
    Heltec.begin();
    Ble.startServer("SpitefulBlue");
    NotificationReceiver.start();
    Ble.setServerCallback(new MainServerCallback(&Heltec));
    gps.start();

    while(true)
    {
        delay(5000);
    }
}

