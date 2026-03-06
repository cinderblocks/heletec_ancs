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

#include "gps.h"
#include "hardware.h"
#include <esp_log.h>
#include <inttypes.h>

static const char* TAG = "gps";

GPS::GPS(String const& name, uint16_t stack_size)
:   Task(name, stack_size, 1)
{ }

void GPS::run(void* /*data*/)
{
    ESP_LOGI(TAG, "Starting GPS (RX=%d TX=%d)", GPS_RX, GPS_TX);

    // Power the VGNSS rail — same GPIO as VEXT used by the TFT.
    // TFT::init() has already driven it HIGH, but guard here in case
    // the draw task hasn't started yet on this boot.
    pinMode(Hardware::VEXT_CTRL, OUTPUT);
    digitalWrite(Hardware::VEXT_CTRL, HIGH);

    // Brief settling time for the UC6580 after power-on.
    vTaskDelay(pdMS_TO_TICKS(100));

    // Open UART1 at 115200 — matches the Heltec factory example exactly.
    _serial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);

    bool     prevFixed     = false;
    uint32_t prevSats      = UINT32_MAX; // sentinel: "not yet logged"
    TickType_t lastTick    = xTaskGetTickCount();

    // How stale a location fix is allowed to be before we consider it lost.
    static constexpr uint32_t FIX_MAX_AGE_MS = 3000;
    // Interval for the 1-second housekeeping tick.
    static constexpr uint32_t TICK_INTERVAL_MS = 1000;

    while (true)
    {
        // ── Drain UART buffer ─────────────────────────────────────────────
        // Must not vTaskDelay() while bytes are available: at 115200 baud
        // the 128-byte ESP32 UART FIFO fills in ~11 ms.
        while (_serial.available() > 0)
        {
            _gps.encode(_serial.read());
        }

        // ── 1-second housekeeping tick ────────────────────────────────────
        TickType_t now = xTaskGetTickCount();
        if ((now - lastTick) < pdMS_TO_TICKS(TICK_INTERVAL_MS))
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // short yield, stay responsive
            continue;
        }
        lastTick = now;

        // A fix is considered valid when location is fresh (< FIX_MAX_AGE_MS).
        const bool fixed = _gps.location.isValid() &&
                           _gps.location.age() < FIX_MAX_AGE_MS;

        // ── Update toolbar icon on state change ───────────────────────────
        if (fixed != prevFixed)
        {
            prevFixed = fixed;
            Heltec.showGpsState(fixed);
            ESP_LOGI(TAG, "GPS fix %s (chars processed: %" PRIu32 ")",
                     fixed ? "acquired" : "lost", _gps.charsProcessed());
        }

        // ── Log position info while fixed ─────────────────────────────────
        if (fixed)
        {
            const uint32_t sats = _gps.satellites.isValid()
                                ? _gps.satellites.value() : 0u;
            const double   hdop = _gps.hdop.isValid()
                                ? _gps.hdop.hdop() : 99.9;

            // Log satellite count only when it changes to avoid log spam.
            if (sats != prevSats)
            {
                prevSats = sats;
                ESP_LOGI(TAG, "Satellites: %" PRIu32 "  HDOP: %.1f", sats, hdop);
            }

            // Log position every tick (1 Hz) — useful during bring-up.
            // Gate on a reasonable HDOP (< 5.0) to suppress noise.
            if (hdop < 5.0)
            {
                ESP_LOGI(TAG, "Lat: %.5f  Lng: %.5f  Alt: %.1f m",
                         _gps.location.lat(),
                         _gps.location.lng(),
                         _gps.altitude.isValid() ? _gps.altitude.meters() : 0.0);
            }
        }
    }
}

/* extern */
GPS gps("GPS", 4096);
