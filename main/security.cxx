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
            // The phone rejected our bond credentials — stale bond (re-flash, or phone
            // forgot us).  Clear the local NVS entry so the next connection starts a
            // fresh numeric-comparison pairing rather than looping on the stale LTK.
            ESP_LOGW(TAG, "Stale bond rejected by peer (0x66) — clearing local bond for %02x:%02x:%02x:%02x:%02x:%02x",
                cmpl.bd_addr[0], cmpl.bd_addr[1], cmpl.bd_addr[2],
                cmpl.bd_addr[3], cmpl.bd_addr[4], cmpl.bd_addr[5]);
            esp_ble_remove_bond_device(cmpl.bd_addr);
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
