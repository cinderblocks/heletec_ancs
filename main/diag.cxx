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

#include "diag.h"
#include "bleservice.h"
#include "gps.h"
#include "hardware.h"
#include "lora.h"
#include "notificationservice.h"
#include "sdkconfig.h"

#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <cstdio>
#include <cinttypes>

static const char* TAG = "diag";

// ── UUIDs ─────────────────────────────────────────────────────────────────
// Custom 128-bit UUIDs — not assigned by the Bluetooth SIG.
static const NimBLEUUID DIAG_SVC_UUID ("BA5EBA11-0000-D1A6-0000-000000000001");
static const NimBLEUUID DIAG_CHAR_UUID("BA5EBA11-0000-D1A6-0000-000000000002");

// ── Static member definitions ──────────────────────────────────────────────
NimBLECharacteristic* Diag::_pChar       = nullptr;
TimerHandle_t         Diag::_notifyTimer = nullptr;
Diag::CharCallbacks   Diag::_charCbs;

// ── buildReport ───────────────────────────────────────────────────────────
size_t Diag::buildReport(char* buf, size_t bufSize)
{
    const uint32_t upSec    = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    const uint32_t heap     = (uint32_t)esp_get_free_heap_size();
    const uint32_t heapMin  = (uint32_t)esp_get_minimum_free_heap_size();
    const unsigned bat      = (unsigned)Heltec.getBatteryLevel();
    const int      bleConn  = Ble.isConnected() ? 1 : 0;
    const int      bonds    = (int)NimBLEDevice::getNumBonds();
    const unsigned notif    = (unsigned)Notifications.getNotificationCount();
    const int      gpsFix   = gps.isFixed() ? 1 : 0;
    const uint32_t gpsSats  = gps.satellites();
    const float    gpsHdop  = gps.hdop();
    const uint32_t gpsOk    = gps.passedChecksum();
    const uint32_t gpsFail  = gps.failedChecksum();

#if CONFIG_LORA_ENABLED
    const LoRaStats ls = Lora.stats();
    const char* loraState =
        ls.state == LoRaStats::State::Listening  ? "listening"   :
        ls.state == LoRaStats::State::InitFailed ? "init_failed" : "disabled";
#endif

    int n = snprintf(buf, bufSize,
        "{"
        "\"up\":%" PRIu32 ","
        "\"heap\":%" PRIu32 ","
        "\"heap_min\":%" PRIu32 ","
        "\"ble\":%d,"
        "\"bat\":%u,"
        "\"gps\":{\"fix\":%d,\"sats\":%" PRIu32 ",\"hdop\":%.1f,"
                 "\"ok\":%" PRIu32 ",\"fail\":%" PRIu32 "},"
#if CONFIG_LORA_ENABLED
        "\"lora\":{\"state\":\"%s\",\"rx\":%" PRIu32 ",\"crc_err\":%" PRIu32 ","
                  "\"hdr_err\":%" PRIu32 ",\"decrypt\":%" PRIu32 ","
                  "\"text\":%" PRIu32 ",\"tx\":%" PRIu32 ","
                  "\"tx_err\":%" PRIu32 ",\"tx_timeout\":%" PRIu32 ","
                  "\"rssi\":%d,\"snr\":%.1f},"
#endif
        "\"notif\":%u,"
        "\"bonds\":%d"
        "}",
        upSec, heap, heapMin,
        bleConn, bat,
        gpsFix, gpsSats, (double)gpsHdop, gpsOk, gpsFail,
#if CONFIG_LORA_ENABLED
        loraState, ls.rxPackets, ls.crcErrors,
        ls.headerErrors, ls.decryptOk,
        ls.textMessages, ls.txPackets,
        ls.txErrors, ls.txTimeouts,
        (int)ls.lastRssi, (double)ls.lastSnr,
#endif
        notif, bonds);

    if (n < 0 || (size_t)n >= bufSize) {
        buf[bufSize - 1] = '\0';
        return bufSize - 1;
    }
    return (size_t)n;
}

// ── CharCallbacks::onRead ─────────────────────────────────────────────────
void Diag::CharCallbacks::onRead(NimBLECharacteristic* pChar,
                                 NimBLEConnInfo& /*connInfo*/)
{
    char buf[384];
    const size_t len = Diag::buildReport(buf, sizeof(buf));
    pChar->setValue(reinterpret_cast<const uint8_t*>(buf), len);
    ESP_LOGD(TAG, "Read (%zu B): %s", len, buf);
}

// ── CharCallbacks::onSubscribe ────────────────────────────────────────────
void Diag::CharCallbacks::onSubscribe(NimBLECharacteristic* /*pChar*/,
                                      NimBLEConnInfo& /*connInfo*/,
                                      uint16_t subValue)
{
    // subValue: 0 = unsubscribed, 1 = NOTIFY, 2 = INDICATE
    if (subValue != 0) {
        ESP_LOGI(TAG, "Client subscribed — starting periodic NOTIFY "
                 "(interval: %us)", (unsigned)CONFIG_DIAG_NOTIFY_INTERVAL_SEC);
        if (_notifyTimer) xTimerStart(_notifyTimer, 0);
    } else {
        ESP_LOGI(TAG, "Client unsubscribed — stopping periodic NOTIFY");
        if (_notifyTimer) xTimerStop(_notifyTimer, 0);
    }
}

// ── _notifyTimerCb ────────────────────────────────────────────────────────
void Diag::_notifyTimerCb(TimerHandle_t /*xTimer*/)
{
    if (!_pChar || !Ble.isConnected()) { return; }

    char buf[384];
    const size_t len = buildReport(buf, sizeof(buf));
    _pChar->setValue(reinterpret_cast<const uint8_t*>(buf), len);
    _pChar->notify();
    ESP_LOGD(TAG, "Notify (%zu B): %s", len, buf);
}

// ── stopNotifications ─────────────────────────────────────────────────────
void Diag::stopNotifications()
{
    if (_notifyTimer) xTimerStop(_notifyTimer, 0);
}

// ── registerService ───────────────────────────────────────────────────────
void Diag::registerService(NimBLEServer* pServer)
{
    NimBLEService* pSvc = pServer->createService(DIAG_SVC_UUID);

    _pChar = pSvc->createCharacteristic(
        DIAG_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    _pChar->setCallbacks(&_charCbs);

    // Prime the characteristic with an initial value so the very first READ
    // never returns an empty payload.
    char buf[384];
    const size_t len = buildReport(buf, sizeof(buf));
    _pChar->setValue(reinterpret_cast<const uint8_t*>(buf), len);

    pSvc->start();

    // Create the periodic NOTIFY timer (auto-reload, initially stopped).
    // Started by onSubscribe(); stopped on unsubscribe or BLE disconnect.
    if (CONFIG_DIAG_NOTIFY_INTERVAL_SEC > 0) {
        _notifyTimer = xTimerCreate(
            "diag_notify",
            pdMS_TO_TICKS((uint32_t)CONFIG_DIAG_NOTIFY_INTERVAL_SEC * 1000U),
            pdTRUE,     // auto-reload
            nullptr,
            _notifyTimerCb);
        if (!_notifyTimer) {
            ESP_LOGW(TAG, "Failed to create notify timer");
        }
    }

    ESP_LOGI(TAG, "Diagnostic service registered  UUID: %s  notify_interval: %us",
             DIAG_SVC_UUID.toString().c_str(),
             (unsigned)CONFIG_DIAG_NOTIFY_INTERVAL_SEC);
}
