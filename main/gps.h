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
#include <freertos/queue.h>
#include <driver/uart.h>

class GPS : public Task
{
public:
    GPS(const char* name, uint16_t stack_size);

    // ── Diagnostic accessors ──────────────────────────────────────────────
    // Safe to call from any task.  _gps is owned by the GPS task; these reads
    // carry no lock — for telemetry purposes a stale or slightly torn read is
    // acceptable, and the data is inherently stale by nature.
    bool     isFixed()        const;  ///< true when location age < FIX_MAX_AGE_MS
    uint32_t satellites();            ///< satellite count (0 if invalid); clears updated flag
    float    hdop();                  ///< HDOP (99.9 if invalid); clears updated flag
    uint32_t passedChecksum() const;  ///< cumulative passed-checksum count
    uint32_t failedChecksum() const;  ///< cumulative failed-checksum count

private:
    void run(void *data) override;

    /// (Re)install the IDF UART driver at the requested baud rate.
    /// Tears down any existing driver first.  Fills _uartQueue on success.
    esp_err_t _installUart(int baud);

    // UC6580 GPS module pins (matches Heltec factory schematic)
    // GPS_TX = ESP32 TX → GPS RX;  GPS_RX = ESP32 RX ← GPS TX
    static constexpr uint8_t GPS_RX = 33;
    static constexpr uint8_t GPS_TX = 34;

    TinyGPSPlus  _gps;
    QueueHandle_t _uartQueue = nullptr;  // populated by _installUart()
};

extern GPS gps;

#endif // HELTEC_ANCS_GPS_H