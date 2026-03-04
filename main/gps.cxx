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

#include <sys/time.h>
#include <time.h>
#include <inttypes.h>

static const char* TAG = "gps";

// Written only from the PPS ISR; read (and cleared) from the GPS task.
static volatile bool s_ppsTriggered = false;

static void IRAM_ATTR ppsISR()
{
    s_ppsTriggered = true;
}

GPS::GPS(String const& name, uint16_t stack_size)
:   Task(name, stack_size, 1)
{ }

void GPS::run(void *data)
{
    ESP_LOGI(TAG, "Starting GPS");

    // Always negotiate baud rate on startup.  After a power cycle (or a previous
    // RST pulse) the module reverts to its factory default of 9600 baud.
    // Open at 9600 first, send PMTK251 to switch to 115200, then reopen at 115200.
    // If the module already retained 115200 in its config, the 9600-baud PMTK
    // sentence arrives as garbage and is ignored — we reopen at 115200 and it works.
    _serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    vTaskDelay(pdMS_TO_TICKS(200)); // let module settle
    _serial.println("$PMTK251,115200*1F");
    _serial.flush();
    vTaskDelay(pdMS_TO_TICKS(100)); // let module process + switch baud
    _serial.end();
    _serial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
    // Discard any bytes buffered during the 9600-baud window (TinyGPS++ would
    // reject them with bad checksums, but clearing them avoids log noise).
    while (_serial.available()) { _serial.read(); }
    ESP_LOGI(TAG, "GPS serial open at 115200");

    // PPS: fires at the leading edge of each UTC second; use it for
    // sub-millisecond settimeofday() precision.
    pinMode(GPS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(GPS_PPS), ppsISR, RISING);

    // 10 s timeout — generous enough to survive temporary signal loss without
    // flooding the log or flickering the GPS icon on brief interruptions.
    static constexpr uint32_t GPS_TIMEOUT_MS   = 10000;
    // If no new UART bytes arrive for this many consecutive seconds the module
    // is considered hung and is power-cycled via GPS_RST.
    static constexpr uint32_t GPS_WATCHDOG_SEC = 30;

    TickType_t lastDisplayUpdate   = xTaskGetTickCount();
    uint32_t   lastCharsProcessed  = _gps.charsProcessed(); // seed from current count
    uint32_t   staleCount          = 0;

    while (true)
    {
        // Drain ALL bytes currently in the UART buffer in one pass.
        // Do NOT call delay() or vTaskDelay() while bytes are available —
        // the ESP32 UART RX FIFO is only 128 bytes.  At 115200 baud that fills
        // in ~11 ms, far shorter than the former 5-second sleep which was
        // causing FIFO overflow and silent data loss.
        while (_serial.available() > 0)
        {
            _gps.encode(_serial.read());
        }

        // ── PPS precision sync ──────────────────────────────────────────────
        // PPS fires at the start of second N+1; the last parsed NMEA time is N.
        // mktime() is used with tm_sec+1 (newlib handles overflow correctly).
        // GPS provides UTC; ESP-IDF boots with TZ unset (treated as UTC by
        // newlib's mktime), so no explicit TZ manipulation is needed.
        if (s_ppsTriggered)
        {
            s_ppsTriggered = false;
            if (_gps.date.isValid() && _gps.time.isValid() &&
                _gps.time.age() < 2000)
            {
                struct tm t = {};
                t.tm_year = _gps.date.year() - 1900;
                t.tm_mon  = _gps.date.month() - 1;
                t.tm_mday = _gps.date.day();
                t.tm_hour = _gps.time.hour();
                t.tm_min  = _gps.time.minute();
                t.tm_sec  = _gps.time.second() + 1; // next second
                struct timeval tv = { .tv_sec = mktime(&t), .tv_usec = 0 };
                settimeofday(&tv, nullptr);
            }
        }

        // ── 1-second display / housekeeping update ──────────────────────────
        TickType_t now = xTaskGetTickCount();
        if ((now - lastDisplayUpdate) >= pdMS_TO_TICKS(1000))
        {
            lastDisplayUpdate = now;

            // Watchdog: if charsProcessed stopped growing the module is hung.
            uint32_t currentChars = _gps.charsProcessed();
            if (currentChars == lastCharsProcessed)
            {
                if (++staleCount >= GPS_WATCHDOG_SEC)
                {
                    // The module has 115200 stored in NVM — do NOT pull RST (that
                    // reverts it to 9600).  A plain UART-driver cycle is enough to
                    // recover a stalled ESP-IDF UART driver or a stuck FIFO.
                    ESP_LOGW(TAG, "GPS watchdog: no data for %us — cycling UART "
                             "(charsProcessed=%" PRIu32 ")", GPS_WATCHDOG_SEC, currentChars);
                    _serial.end();
                    vTaskDelay(pdMS_TO_TICKS(500));
                    _serial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
                    while (_serial.available()) { _serial.read(); } // flush stale bytes
                    lastCharsProcessed = _gps.charsProcessed();
                    staleCount = 0;
                }
            }
            else
            {
                if (staleCount > 0)
                {
                    ESP_LOGI(TAG, "GPS data resumed (charsProcessed=%" PRIu32 ")", currentChars);
                }
                lastCharsProcessed = currentChars;
                staleCount = 0;
            }

            if (_gps.time.isValid() && _gps.time.age() < GPS_TIMEOUT_MS)
            {
                // Fallback system-clock sync (less precise than PPS but covers
                // the case where the PPS pin is not connected or not yet fired).
                if (_gps.date.isValid())
                {
                    struct tm t = {};
                    t.tm_year = _gps.date.year() - 1900;
                    t.tm_mon  = _gps.date.month() - 1;
                    t.tm_mday = _gps.date.day();
                    t.tm_hour = _gps.time.hour();
                    t.tm_min  = _gps.time.minute();
                    t.tm_sec  = _gps.time.second();
                    struct timeval tv = { .tv_sec = mktime(&t), .tv_usec = 0 };
                    settimeofday(&tv, nullptr);
                }

                char timestamp[8];
                snprintf(timestamp, sizeof(timestamp), "%2u:%02u",
                         convert_to_tz(_gps.time.hour(), CONFIG_GPS_TZ_OFFSET),
                         _gps.time.minute());
                Heltec.showGpsState(true);
                Heltec.showTime(timestamp);

                // Gate location logging on a reasonable HDOP (< 5.0 == value < 500).
                if (_gps.location.isValid() &&
                    _gps.hdop.isValid() && _gps.hdop.value() < 500)
                {
                    ESP_LOGI(TAG, "LAT: %0.5f LNG: %0.5f (HDOP: %.1f)",
                             _gps.location.lat(), _gps.location.lng(),
                             _gps.hdop.value() / 100.0);
                }
                if (_gps.altitude.isValid())
                {
                    ESP_LOGI(TAG, "Alt %.1f m", _gps.altitude.meters());
                }
            }
            else
            {
                //ESP_LOGI(TAG, "GPS timeout");
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