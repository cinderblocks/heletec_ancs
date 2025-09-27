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

#ifndef HELTEC_ANCS_GPS_H
#define HELTEC_ANCS_GPS_H

#include "task.h"
#include <HT_TinyGPS++.h>

class GPS : public Task
{
public:
    GPS(String const& name, uint16_t stack_size);
private:
    void run(void *data) override;

    static constexpr uint8_t GPS_RX = 33;
    static constexpr uint8_t GPS_TX = 34;
    static constexpr uint8_t GPS_RST = 35;
    static constexpr uint8_t GPS_PPS = 36;

    TinyGPSPlus _gps;
    HardwareSerial _serial = HardwareSerial(2);
    TaskHandle_t _gpsTask = nullptr;
};

#endif //HELTEC_ANCS_GPS_H