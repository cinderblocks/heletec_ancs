/**
 * Copyright (c) 2024-2025 Sjofn LLC
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

#include "security.h"

#include "hardware.h"
#include <BLEDevice.h>
#include <esp_gap_ble_api.h>
#include <esp_log.h>

static const char* TAG = "security";

uint32_t SecurityCallback::onPassKeyRequest()
{
    ESP_LOGI(TAG, "PassKeyRequest");
    return BLE_SM_DEFAULT_PASSKEY;
}

void SecurityCallback::onPassKeyNotify(uint32_t passkey)
{
    ESP_LOGI(TAG, "On passkey Notify number:%d", passkey);
    Heltec.pairing(String(passkey));
}

bool SecurityCallback::onSecurityRequest()
{
    ESP_LOGI(TAG, "On Security Request");
    return true;
}

bool SecurityCallback::onConfirmPIN(uint32_t pin)
{
    ESP_LOGI(TAG, "On Confirmed PIN Request");
    Heltec.pairing(String(pin));
    return true;
}

#if defined(CONFIG_BLUEDROID_ENABLED)
void SecurityCallback::onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl)
#elif defined(CONFIG_NIMBLE_ENABLED)
void SecurityCallback::onAuthenticationComplete(ble_gap_conn_desc* cmpl)
#endif
{
#if defined(CONFIG_BLUEDROID_ENABLED)
    if (!cmpl.success)
    {
        if (cmpl.fail_reason == 0x66)
        {
            // The peer rejected our LTK — stale bond on our side (re-flash, NVS wipe, etc.).
            //
            // IMPORTANT: cmpl.bd_addr is the RPA (Resolvable Private Address) that iOS
            // is using for this connection attempt.  iOS rotates its RPA on every
            // reconnect, so cmpl.bd_addr is never the identity address that Bluedroid
            // stored in NVS when we originally paired.  Calling
            // esp_ble_remove_bond_device(cmpl.bd_addr) therefore always hits the
            // "Device not found" error path and leaves the stale bond intact, causing
            // the 0x66 rejection loop to repeat indefinitely.
            //
            // Fix: enumerate all stored bonds and wipe them all.  We cannot resolve
            // the RPA to an identity address without the peer's IRK (which we only
            // hold from a now-stale bond), so a full wipe is the only reliable option.
            ESP_LOGW(TAG, "Stale bond rejected by peer (0x66) at RPA %02x:%02x:%02x:%02x:%02x:%02x — wiping all local bonds",
                cmpl.bd_addr[0], cmpl.bd_addr[1], cmpl.bd_addr[2],
                cmpl.bd_addr[3], cmpl.bd_addr[4], cmpl.bd_addr[5]);

            int bondCount = esp_ble_get_bond_device_num();
            if (bondCount > 0)
            {
                static constexpr int MAX_BONDS = 8; // Bluedroid NVS maximum
                esp_ble_bond_dev_t devList[MAX_BONDS];
                int count = (bondCount <= MAX_BONDS) ? bondCount : MAX_BONDS;
                esp_ble_get_bond_device_list(&count, devList);
                for (int i = 0; i < count; i++)
                {
                    esp_ble_remove_bond_device(devList[i].bd_addr);
                }
                ESP_LOGW(TAG, "Cleared %d stored bond(s) — next connection will require fresh pairing", count);
            }
            else
            {
                ESP_LOGW(TAG, "No local bonds found to clear (already wiped by BT stack)");
            }
        }
        else
        {
            ESP_LOGI(TAG, "Authentication failed, reason=0x%02x", cmpl.fail_reason);
        }
        // ServerCallback::onDisconnect will have already (or will shortly) call
        // setBLEConnectionState(BLE_DISCONNECTED), but set it here too so the
        // display updates correctly for auth failures that don't cause an immediate
        // link-layer disconnect (e.g. wrong PIN on some platforms).
        Heltec.setBLEConnectionState(BLE_DISCONNECTED);
        return;
    }
#elif defined(CONFIG_NIMBLE_ENABLED)
    if (!cmpl->sec_state.encrypted)
    {
        BLEDevice::getServer()->disconnect(cmpl->conn_handle);
        ESP_LOGI(TAG, "Encryption failed - Disconnecting client.");
        Heltec.setBLEConnectionState(BLE_DISCONNECTED);
        return;
    }
#endif
    ESP_LOGI(TAG, "Authentication successful");
    // Clear the pairing display - device is now connected and ready
    Heltec.setBLEConnectionState(BLE_CONNECTED);
}

/* extern */
SecurityCallback Security;
