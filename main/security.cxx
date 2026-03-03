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

#include "bleservice.h"
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
            ESP_LOGW(TAG, "Stale bond rejected by peer (0x66) at RPA %02x:%02x:%02x:%02x:%02x:%02x — wiping all local bonds",
                cmpl.bd_addr[0], cmpl.bd_addr[1], cmpl.bd_addr[2],
                cmpl.bd_addr[3], cmpl.bd_addr[4], cmpl.bd_addr[5]);

            int bondCount = esp_ble_get_bond_device_num();
            if (bondCount > 0)
            {
                static constexpr int MAX_BONDS = 8;
                esp_ble_bond_dev_t devList[MAX_BONDS];
                int count = (bondCount <= MAX_BONDS) ? bondCount : MAX_BONDS;
                esp_ble_get_bond_device_list(&count, devList);
                for (int i = 0; i < count; i++)
                {
                    esp_ble_remove_bond_device(devList[i].bd_addr);
                }
                ESP_LOGW(TAG, "Cleared %d stored bond(s)", count);
            }
            else
            {
                ESP_LOGW(TAG, "No local bonds found (already wiped by BT stack)");
            }

            // noteAuthFail() tracks consecutive failures.  On the first failure
            // iOS had a stale LTK; it clears it and retries.  On the second+
            // failure iOS has no bond but sends MITM=0 (background service can't
            // show a dialog) which our MITM=1 requirement rejects — the loop
            // cannot self-resolve.  Once the threshold is reached, the display
            // shows "iOS Settings" so the user knows to open Settings > Bluetooth
            // and tap the device name; with the BT settings page foregrounded,
            // iOS will send MITM=1 and the passkey dialog will appear.
            if (!Ble.noteAuthFail())
            {
                // Below threshold — still in the stale-LTK-clearing phase,
                // just show Disconnected and keep advertising normally.
                Heltec.setBLEConnectionState(BLE_DISCONNECTED);
            }
            // If noteAuthFail() returned true the display already shows the
            // pairing instructions (state = BLE_PAIRING); don't overwrite it.
        }
        else
        {
            ESP_LOGI(TAG, "Authentication failed, reason=0x%02x", cmpl.fail_reason);
            Heltec.setBLEConnectionState(BLE_DISCONNECTED);
        }
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
    Ble.resetAuthStreak();
    // Clear the pairing display - device is now connected and ready
    Heltec.setBLEConnectionState(BLE_CONNECTED);
}

/* extern */
SecurityCallback Security;
