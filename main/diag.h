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

#ifndef DIAG_H_
#define DIAG_H_

#include <NimBLEDevice.h>
#include <freertos/timers.h>
#include <cstddef>

/**
 * BLE Diagnostic Service
 *
 * Exposes a single READ + NOTIFY GATT characteristic that returns a compact
 * JSON snapshot of runtime state: heap, GPS, BLE, battery, notification queue.
 *
 * Service UUID:        BA5EBA11-0000-D1A6-0000-000000000001
 * Characteristic UUID: BA5EBA11-0000-D1A6-0000-000000000002
 *
 * Example report (≈ 250 bytes):
 *   {"up":3600,"heap":142080,"heap_min":98304,"ble":1,"bat":85,
 *    "gps":{"fix":1,"sats":8,"hdop":1.2,"ok":142,"fail":0},
 *    "lora":{"state":"listening","rx":12,"crc_err":0,"hdr_err":0,
 *            "decrypt":10,"text":3,"rssi":-87,"snr":7.5},
 *    "notif":2,"bonds":1}
 *
 * Usage:
 *   // In BleService::startServer(), after createService() calls:
 *   Diag::registerService(pServer);   // before startServices()
 *
 *   // In BleService disconnect handler:
 *   Diag::stopNotifications();
 */
class Diag
{
public:
    /// Register the diagnostic GATT service on the given server.
    /// Must be called before NimBLEServer::startServices().
    static void registerService(NimBLEServer* pServer);

    /// Stop the periodic NOTIFY timer.  Call on BLE disconnect so the timer
    /// does not fire into an unconnected stack.
    static void stopNotifications();

    /// Build a compact JSON diagnostic report into buf (null-terminated).
    /// Returns the number of bytes written (excluding NUL).
    static size_t buildReport(char* buf, size_t bufSize);

private:
    static NimBLECharacteristic* _pChar;
    static TimerHandle_t         _notifyTimer;

    static void _notifyTimerCb(TimerHandle_t xTimer);

    /// GATT characteristic callbacks — onRead builds a fresh report;
    /// onSubscribe starts/stops the periodic notify timer.
    class CharCallbacks final : public NimBLECharacteristicCallbacks
    {
        void onRead(NimBLECharacteristic* pChar,
                    NimBLEConnInfo& connInfo) override;
        void onSubscribe(NimBLECharacteristic* pChar,
                         NimBLEConnInfo& connInfo,
                         uint16_t subValue) override;
    };
    static CharCallbacks _charCbs;
};

#endif // DIAG_H_
