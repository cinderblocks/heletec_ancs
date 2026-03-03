/**
* Copyright (c) 2025 Sjofn LLC
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

#include "gps.h"

#include "hardware.h"
#include "util.h"

static const char* TAG = "gps";

GPS::GPS(String const& name, uint16_t stack_size)
:   Task(name, stack_size, 1)
{ }

void GPS::run(void *data)
{
    ESP_LOGI(TAG, "Starting GPS");
    _serial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);

    // 10 s timeout — generous enough to survive temporary signal loss without
    // flooding the log or flickering the GPS icon on brief interruptions.
    static constexpr uint32_t GPS_TIMEOUT_MS = 10000;

    TickType_t lastDisplayUpdate = xTaskGetTickCount();

    while (true)
    {
        // Drain ALL bytes currently in the UART buffer in one pass.
        // Do NOT call delay() or vTaskDelay() while bytes are available —
        // the ESP32 UART RX FIFO is only 128 bytes.  At 115200 baud that fills
        // in ~11 ms, far shorter than the former 5-second sleep which was
        // causing FIFO overflow and silent data loss.
        // Also do NOT use a "while (_serial.read() > 0)" drain after a sentence —
        // that discards the leading bytes of the very next sentence and then
        // guarantees the buffer is empty, forcing the 5-second sleep every cycle.
        while (_serial.available() > 0)
        {
            _gps.encode(_serial.read());
        }

        // Update the display at most once per second regardless of how many
        // sentences were parsed in this drain pass.
        TickType_t now = xTaskGetTickCount();
        if ((now - lastDisplayUpdate) >= pdMS_TO_TICKS(1000))
        {
            lastDisplayUpdate = now;

            if (_gps.time.isValid() && _gps.time.age() < GPS_TIMEOUT_MS)
            {
                char timestamp[8];
                snprintf(timestamp, sizeof(timestamp), "%2u:%02u",
                         convert_to_tz(_gps.time.hour(), -5), _gps.time.minute());
                Heltec.showGpsState(true);
                Heltec.showTime(timestamp);

                if (_gps.location.isValid())
                {
                    ESP_LOGI(TAG, "LAT: %0.5f LNG: %0.5f",
                             _gps.location.lat(), _gps.location.lng());
                }
                if (_gps.altitude.isValid())
                {
                    ESP_LOGI(TAG, "Alt %0.5f", _gps.altitude.miles());
                }
            }
            else
            {
                ESP_LOGI(TAG, "GPS timeout");
                Heltec.showGpsState(false);
            }
        }

        // Short yield: lets other tasks run while remaining responsive to new
        // UART bytes.  10 ms << FIFO fill time (~11 ms at 115200), so no data
        // is ever lost waiting for the next drain pass.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* extern */
GPS gps("GPS", 10000);
