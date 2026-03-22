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

#ifndef HELTEC_ANCS_UTIL_H
#define HELTEC_ANCS_UTIL_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ── BLE connection state ───────────────────────────────────────────────────
// Defined here (not in hardware.h) so that Display and BleService can use
// it without a circular dependency on Hardware.
enum conn_state_def
{
    BLE_DISCONNECTED = 0,
    BLE_PAIRING,
    BLE_SERVER_CONNECTED,
    BLE_CONNECTED
};

// ── ScopedLock ────────────────────────────────────────────────────────────
// RAII wrapper for a FreeRTOS mutex semaphore.  Takes the mutex on
// construction and releases it on destruction, ensuring it is always
// released even on early return.
//
// Do NOT hold a ScopedLock across a task delay or any blocking call —
// use a plain mutex (xSemaphoreTake/Give) in those cases.
struct ScopedLock {
    explicit ScopedLock(SemaphoreHandle_t m) : _m(m)
    { if (_m) xSemaphoreTake(_m, portMAX_DELAY); }
    ~ScopedLock() { if (_m) xSemaphoreGive(_m); }
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
private:
    SemaphoreHandle_t _m;
};

#endif // HELTEC_ANCS_UTIL_H