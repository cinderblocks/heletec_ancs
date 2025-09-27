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
#include "heltec.h"
#include "ancsservice.h"
#include "gps.h"
#include "notificationservice.h"
#include "task.h"

#include <esp_log.h>
#include <cinttypes>

class NotificationDescription;

static const char* TAG = "main";
RTC_DATA_ATTR static int boot_count = 0;

ANCSService* ancsService;
NotificationDescription* notificationReceiver;
GPS* gps;

class MainServerCallback final : public ANCSServiceServerCallback {
public:
    explicit MainServerCallback(Heltec_ESP32 *ht) : _ht(ht) { }
    ~MainServerCallback() = default;
    void onConnect() override {
        _ht->setBLEConnectionState(BLE_CONNECTED);
    }
    void onDisconnect() override {
        _ht->setBLEConnectionState(BLE_DISCONNECTED);
    }
private:
    Heltec_ESP32* _ht;
};

class MainClientCallback final : public ANCSServiceClientCallback {
public:
    explicit MainClientCallback(Heltec_ESP32  *ht) : _ht(ht) { }
    ~MainClientCallback() = default;
    void onConnect() override {
        ESP_LOGI(TAG, "Client Connected");
        _ht->setBLEConnectionState(BLE_CONNECTED);
    }
    void onDisconnect() override {
        _ht->setBLEConnectionState(BLE_DISCONNECTED);
    }
private:
    Heltec_ESP32* _ht;
};

class NotificationDescription final : public Task {
public:
    NotificationDescription(String const& name, uint16_t stack_size) : Task(name, stack_size) { }
private:
    void run(void *data) override {
        while(true) {
            uint32_t pendingNotificationId = Notifications.getNextPendingNotification();
            if (pendingNotificationId != 0)
            {
                ancsService->retrieveNotificationData(pendingNotificationId);
            }
            delay(500);
        }
    }
};

static uint8_t last_battery_level = 0;
static void BatteryTimerCallback(TimerHandle_t xTimer)
{
    uint8_t level = Heltec.getBatteryLevel();
    if (level != last_battery_level) {
        last_battery_level = level;
        Heltec.showBatteryLevel(level);
        ancsService->setBatteryLevel(level);
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Boot count: %d", boot_count++);

    initArduino();
    Heltec.begin();

    ancsService = new ANCSService(
        &NotificationService::NotificationSourceNotifyCallback,
        &NotificationService::DataSourceNotifyCallback);
    ancsService->startServer("SpitefulBlue");
    notificationReceiver = new NotificationDescription("NotificationReceiver", 50000);
    notificationReceiver->start();
    ancsService->setServerCallback(new MainServerCallback(&Heltec));
    gps = new GPS("GPS", 10000);
    gps->start();

    while(true)
    {
        BatteryTimerCallback(nullptr);
        delay(5000);
    }
}
