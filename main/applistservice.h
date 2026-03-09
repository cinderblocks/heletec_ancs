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

#ifndef APPLIST_SERVICE_H_
#define APPLIST_SERVICE_H_

#include <NimBLEDevice.h>

/**
 * AppList BLE Management Service
 *
 * Lets a connected client add/remove custom app-filter entries at runtime.
 * Changes are persisted to NVS immediately and survive reboots.
 *
 * Service UUID:           BA5EBA11-0000-D1A6-0000-000000000003
 *
 * LIST characteristic:    BA5EBA11-0000-D1A6-0000-000000000004
 *   Properties: READ + NOTIFY
 *   Value: tab-separated lines, one custom entry per line:
 *     "<bundleId>\t<displayName>\n"
 *   A NOTIFY is sent automatically after every successful add/remove/reset.
 *   Built-in entries are not included (they are static and documented in code).
 *
 * CMD characteristic:     BA5EBA11-0000-D1A6-0000-000000000005
 *   Properties: WRITE (no response)
 *   Commands:
 *     "+"  + bundleId + "\t" + displayName   — add a custom entry
 *     "-"  + bundleId                        — remove a custom entry
 *     "!"                                    — reset to defaults (clear all custom)
 *
 * Example (nRF Connect — write as UTF-8 string):
 *   Add:    +com.example.myapp\tMy App
 *   Remove: -com.example.myapp
 *   Reset:  !
 */
class AppListService
{
public:
    /// Register the service on the given server.
    /// Must be called before NimBLEServer::startServices().
    static void registerService(NimBLEServer* pServer);

    /// Push the current custom list to all subscribed LIST clients.
    /// Called automatically by the CMD callbacks; exposed publicly so
    /// other code can trigger a refresh if needed.
    static void notifyListUpdated();

private:
    static NimBLECharacteristic* _pListChar;
    static NimBLECharacteristic* _pCmdChar;

    /// Fill buf with the tab-separated custom-entry list (null-terminated).
    /// Returns the number of bytes written (excluding NUL).
    static size_t _buildListValue(char* buf, size_t bufSize);

    class ListCallbacks final : public NimBLECharacteristicCallbacks {
        void onRead(NimBLECharacteristic* pChar,
                    NimBLEConnInfo& connInfo) override;
    };

    class CmdCallbacks final : public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic* pChar,
                     NimBLEConnInfo& connInfo) override;
    };

    static ListCallbacks _listCbs;
    static CmdCallbacks  _cmdCbs;
};

#endif // APPLIST_SERVICE_H_
