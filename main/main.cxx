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

#include "sdkconfig.h"
#include "applist.h"
#include "gps.h"
#include "hardware.h"
#include "bleservice.h"
#include "lora.h"
#include "meshnode.h"
#include "notificationservice.h"
#include "task.h"

#include <esp_log.h>
#include <cinttypes>

static const char* TAG = "main";
RTC_DATA_ATTR static int boot_count = 0;

class MainServerCallback final : public ANCSServiceServerCallback {
public:
    explicit MainServerCallback(Hardware *ht) : _ht(ht) { }
    void onConnect() override {
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

    Heltec.begin();
    Ble.startServer(CONFIG_BLE_DEVICE_NAME);
    NotificationReceiver.start();
    gps.start();
#if CONFIG_LORA_ENABLED
    Node.init();
    Lora.start();
#endif
    static MainServerCallback serverCallback(&Heltec);
    Ble.setServerCallback(&serverCallback);

    // Sync the ESP32 system clock and header display from the iOS Current Time
    // Service (CTS).  Called once immediately on connect (initial read) and again
    // automatically on any iOS time change (DST, manual adjustment, NTP sync).
    // onTimeSync() stores the UTC offset and starts a 30-second periodic timer
    // so the header clock stays current for the lifetime of the connection.
    Ble.setTimeCallback([](const struct tm *localTime, int32_t utcOffsetSec) {
        Heltec.onTimeSync(localTime, utcOffsetSec);
    });

    // All work is done by dedicated FreeRTOS tasks.  Delete this task to
    // reclaim its ~4 KB stack — it serves no purpose after initialization.
    vTaskDelete(nullptr);
}

