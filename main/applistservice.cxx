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

#include "applistservice.h"
#include "applist.h"
#include "bleservice.h"

#include <esp_log.h>
#include <cstring>
#include <cstdio>

static const char* TAG = "applist_svc";

// ── UUIDs ─────────────────────────────────────────────────────────────────
static const NimBLEUUID APPLIST_SVC_UUID ("BA5EBA11-0000-D1A6-0000-000000000003");
static const NimBLEUUID APPLIST_LIST_UUID("BA5EBA11-0000-D1A6-0000-000000000004");
static const NimBLEUUID APPLIST_CMD_UUID ("BA5EBA11-0000-D1A6-0000-000000000005");

// ── Static member definitions ─────────────────────────────────────────────
NimBLECharacteristic*      AppListService::_pListChar = nullptr;
NimBLECharacteristic*      AppListService::_pCmdChar  = nullptr;
AppListService::ListCallbacks AppListService::_listCbs;
AppListService::CmdCallbacks  AppListService::_cmdCbs;

// ── _buildListValue ───────────────────────────────────────────────────────
// Format: one custom entry per line, fields separated by a tab:
//   "com.example.foo\tFoo App\n"
//
// The value is intentionally custom-entries-only.  Built-in entries are
// static and documented; the LIST characteristic is the mutable layer.
size_t AppListService::_buildListValue(char* buf, size_t bufSize)
{
    size_t pos = 0;
    const size_t count = AppList.getCustomCount();

    for (size_t i = 0; i < count; i++) {
        ApplicationList::CustomEntry entry;
        if (!AppList.getCustomEntry(i, entry)) continue;

        // Each line: "bundleId\tdisplayName\n"
        int n = snprintf(buf + pos, bufSize - pos,
                         "%s\t%s\n", entry.bundleId, entry.displayName);
        if (n <= 0 || pos + (size_t)n >= bufSize) break;  // truncate rather than overflow
        pos += (size_t)n;
    }

    if (pos == 0 && bufSize > 0) {
        // No custom entries — send a clear indicator rather than an empty payload.
        int n = snprintf(buf, bufSize, "(none)\n");
        pos = (n > 0) ? (size_t)n : 0;
    }

    if (pos < bufSize) buf[pos] = '\0';
    return pos;
}

// ── notifyListUpdated ─────────────────────────────────────────────────────
void AppListService::notifyListUpdated()
{
    if (!_pListChar || !Ble.isConnected()) return;

    char buf[512];
    const size_t len = _buildListValue(buf, sizeof(buf));
    _pListChar->setValue(reinterpret_cast<const uint8_t*>(buf), len);
    _pListChar->notify();
    ESP_LOGD(TAG, "Notified list (%zu B)", len);
}

// ── ListCallbacks::onRead ─────────────────────────────────────────────────
void AppListService::ListCallbacks::onRead(NimBLECharacteristic* pChar,
                                           NimBLEConnInfo& /*connInfo*/)
{
    char buf[512];
    const size_t len = AppListService::_buildListValue(buf, sizeof(buf));
    pChar->setValue(reinterpret_cast<const uint8_t*>(buf), len);
    ESP_LOGD(TAG, "Read list (%zu B):\n%s", len, buf);
}

// ── CmdCallbacks::onWrite ─────────────────────────────────────────────────
// Accepted commands (UTF-8 string written to the characteristic):
//
//   "+"  + bundleId + "\t" + displayName   — add custom entry
//   "-"  + bundleId                        — remove custom entry
//   "!"                                    — clear all custom entries
//
void AppListService::CmdCallbacks::onWrite(NimBLECharacteristic* pChar,
                                           NimBLEConnInfo& /*connInfo*/)
{
    const std::string val = pChar->getValue();
    if (val.empty()) return;

    const char cmd = val[0];
    // Payload is everything after the first byte, null-terminated.
    // Make a mutable copy so we can tokenise it safely.
    char payload[ApplicationList::BUNDLE_ID_MAX + ApplicationList::DISPLAY_NAME_MAX + 4] = {};
    const size_t payloadLen = val.size() - 1;
    if (payloadLen > 0) {
        memcpy(payload, val.c_str() + 1, std::min(payloadLen, sizeof(payload) - 1));
    }

    switch (cmd)
    {
        case '+':
        {
            // Payload: "bundleId\tdisplayName" (tab-separated)
            char* tab = strchr(payload, '\t');
            if (!tab) {
                ESP_LOGW(TAG, "CMD '+': missing tab separator in payload");
                return;
            }
            *tab = '\0';                       // split into two C-strings
            const char* bundleId    = payload;
            const char* displayName = tab + 1;
            // Strip any trailing newline from display name.
            char* nl = strchr(displayName, '\n');
            if (nl) *nl = '\0';

            if (AppList.addEntry(bundleId, displayName)) {
                ESP_LOGI(TAG, "CMD '+': added '%s' (\"%s\")", bundleId, displayName);
                AppListService::notifyListUpdated();
            }
            break;
        }

        case '-':
        {
            // Payload: "bundleId" (strip trailing newline if present)
            char* nl = strchr(payload, '\n');
            if (nl) *nl = '\0';

            if (AppList.removeEntry(payload)) {
                ESP_LOGI(TAG, "CMD '-': removed '%s'", payload);
                AppListService::notifyListUpdated();
            }
            break;
        }

        case '!':
            AppList.resetToDefaults();
            ESP_LOGI(TAG, "CMD '!': reset to built-in defaults");
            AppListService::notifyListUpdated();
            break;

        default:
            ESP_LOGW(TAG, "CMD: unknown command byte 0x%02X", (unsigned char)cmd);
            break;
    }
}

// ── registerService ───────────────────────────────────────────────────────
void AppListService::registerService(NimBLEServer* pServer)
{
    NimBLEService* pSvc = pServer->createService(APPLIST_SVC_UUID);

    // LIST — readable and notifiable; primed with the current custom list.
    _pListChar = pSvc->createCharacteristic(
        APPLIST_LIST_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    _pListChar->setCallbacks(&_listCbs);

    char buf[512];
    const size_t len = _buildListValue(buf, sizeof(buf));
    _pListChar->setValue(reinterpret_cast<const uint8_t*>(buf), len);

    // CMD — write-only (no response needed; result is visible via LIST NOTIFY).
    _pCmdChar = pSvc->createCharacteristic(
        APPLIST_CMD_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    _pCmdChar->setCallbacks(&_cmdCbs);

    pSvc->start();

    ESP_LOGI(TAG, "AppList service registered  custom entries: %zu",
             AppList.getCustomCount());
}
