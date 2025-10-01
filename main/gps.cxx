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

static const char* TAG = "gps";
static constexpr uint32_t INTERVAL = 5000;

GPS::GPS(String const& name, uint16_t stack_size)
:   Task(name, stack_size, 1)
{ }

void GPS::run(void *data)
{
    ESP_LOGI(TAG, "Starting GPS");
    _serial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
    while (true)
    {
        if (_serial.available() > 0)
        {
            if (_serial.peek() != '\n')
            {
                _gps.encode(_serial.read());
            }
            else
            {
                _serial.read();

                if (_gps.time.isValid() && _gps.time.age() < INTERVAL)
                {
                    Heltec.showGpsState(true);
                    char timestamp[8];
                    snprintf(timestamp, sizeof(timestamp), "%2u:%02u", _gps.time.hour(), _gps.time.minute());
                    Heltec.showTime(timestamp);
                }
                else
                {
                    ESP_LOGI(TAG, "GPS timeout");
                    Heltec.showGpsState(false);
                }
                if (_gps.location.isValid())
                {
                    ESP_LOGI(TAG, "LAT: %0.5f LNG: %0.5f", _gps.location.lat(), _gps.location.lng());
                }
                if (_gps.altitude.isValid())
                {
                    ESP_LOGI(TAG, "Alt %0.5f", _gps.altitude.miles());
                }
                while (_serial.read() > 0);
            }
        }
        else
        {
            delay(INTERVAL);
        }
    }
}

/* extern */
GPS gps("GPS", 10000);
