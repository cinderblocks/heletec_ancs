/**
* Copyright (c) 2025-2026 Sjofn LLC
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

#ifndef HELTEC_ANCS_GPS_H
#define HELTEC_ANCS_GPS_H

#include "task.h"
#include <TinyGPSPlus.h>

class GPS : public Task
{
public:
    GPS(String const& name, uint16_t stack_size);
private:
    void run(void *data) override;

    // UC6580 GPS module pins (matches Heltec factory example)
    // GPS_TX = ESP32 TX → GPS RX; GPS_RX = ESP32 RX ← GPS TX
    static constexpr uint8_t GPS_RX = 33;
    static constexpr uint8_t GPS_TX = 34;

    TinyGPSPlus _gps;
    // No HardwareSerial member — UART1 is driven via the IDF uart driver
    // directly in run() to avoid the Arduino wrapper's dual-instance conflict.
};

extern GPS gps;

#endif //HELTEC_ANCS_GPS_H
