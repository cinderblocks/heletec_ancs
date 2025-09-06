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

#ifndef SECURITY_CALLBACK_H_
#define SECURITY_CALLBACK_H_

#include <BLESecurity.h>

class SecurityCallback final : public BLESecurityCallbacks
{
public:
    uint32_t onPassKeyRequest() override;
    void onPassKeyNotify(uint32_t passkey) override;
    bool onSecurityRequest() override;
    bool onConfirmPIN(uint32_t pin) override;
#if defined(CONFIG_BLUEDROID_ENABLED)
    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override;
#elif defined(CONFIG_NIMBLE_ENABLED)
    void onAuthenticationComplete(ble_gap_conn_desc* cmpl) override;
#endif
};

extern SecurityCallback Security;

#endif /* SECURITY_CALLBACK_H_ */
