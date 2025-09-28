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
        ESP_LOGI(TAG, "Authentication failed = 0x%x");
        return;
    }
#elif defined(CONFIG_NIMBLE_ENABLED)
    if (!cmpl->sec_state.encrypted)
    {
        BLEDevice::getServer()->disconnect(cmpl->conn_handle);
        ESP_LOGI(TAG, "Encryption failed - Disconnecting client.");
        return;
    }
#endif
    ESP_LOGI(TAG, "Authentication successful");
}

/* extern */
SecurityCallback Security;
