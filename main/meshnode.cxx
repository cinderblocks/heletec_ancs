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

#include "meshnode.h"
#include "sdkconfig.h"

#include <esp_mac.h>
#include <esp_log.h>
#include <esp_random.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <cstdio>
#include <cstring>
#include <cinttypes>

static const char* TAG      = "meshnode";
static const char* NVS_NS   = "mesh";      // NVS namespace
static const char* NVS_KEY_SHORT = "short";
static const char* NVS_KEY_LONG  = "long";

// ── init ──────────────────────────────────────────────────────────────────
void MeshNode::init()
{
    // ── Derive node ID from BT MAC (lower 4 bytes) ────────────────────────
    // esp_read_mac(ESP_MAC_BT) returns the Bluetooth MAC directly, matching
    // exactly the node ID a stock Meshtastic build would assign to this chip.
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_BT);

    _nodeId = (static_cast<uint32_t>(mac[2]) << 24)
            | (static_cast<uint32_t>(mac[3]) << 16)
            | (static_cast<uint32_t>(mac[4]) <<  8)
            |  static_cast<uint32_t>(mac[5]);

    snprintf(_nodeIdStr, sizeof(_nodeIdStr), "!%08" PRIx32, _nodeId);

    // ── Load names from NVS; fall back to Kconfig defaults ───────────────
    // nvs_flash_init() is idempotent — safe to call even if already done
    // elsewhere (bleservice.cxx calls it indirectly through NimBLE).
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated or has a newer version — erase and retry.
        ESP_LOGW(TAG, "NVS partition needs erase: %s", esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
    }

    // Populate in-memory cache with Kconfig defaults first, then try NVS.
    strncpy(_shortName, CONFIG_MESH_NODE_SHORT_NAME, sizeof(_shortName) - 1);
    _shortName[sizeof(_shortName) - 1] = '\0';
    strncpy(_longName,  CONFIG_MESH_NODE_LONG_NAME,  sizeof(_longName)  - 1);
    _longName[sizeof(_longName) - 1] = '\0';

    if (err == ESP_OK)
    {
        nvs_handle_t h;
        err = nvs_open(NVS_NS, NVS_READONLY, &h);
        if (err == ESP_OK)
        {
            // Short name — read into a temporary to avoid partial overwrites
            char tmp[sizeof(_shortName)] = {};
            size_t len = sizeof(tmp);
            if (nvs_get_str(h, NVS_KEY_SHORT, tmp, &len) == ESP_OK && len > 1)
            {
                // len includes the NUL terminator; > 1 means at least one char
                memcpy(_shortName, tmp, sizeof(_shortName));
            }

            // Long name
            char tmpL[sizeof(_longName)] = {};
            len = sizeof(tmpL);
            if (nvs_get_str(h, NVS_KEY_LONG, tmpL, &len) == ESP_OK && len > 1)
            {
                memcpy(_longName, tmpL, sizeof(_longName));
            }

            nvs_close(h);
        }
        // ESP_ERR_NVS_NOT_FOUND is expected on first boot — not an error.
    }
    else
    {
        ESP_LOGW(TAG, "NVS init failed (%s) — using Kconfig defaults",
                 esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Node ID: %s  short: \"%s\"  long: \"%s\"",
             _nodeIdStr, _shortName, _longName);
}

// ── setShortName ──────────────────────────────────────────────────────────
void MeshNode::setShortName(const char* name)
{
    if (name == nullptr || name[0] == '\0') return;

    strncpy(_shortName, name, sizeof(_shortName) - 1);
    _shortName[sizeof(_shortName) - 1] = '\0';

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK)
    {
        const esp_err_t err = nvs_set_str(h, NVS_KEY_SHORT, _shortName);
        if (err == ESP_OK) nvs_commit(h);
        else ESP_LOGW(TAG, "nvs_set_str(short) failed: %s", esp_err_to_name(err));
        nvs_close(h);
    }

    ESP_LOGI(TAG, "Short name updated: \"%s\"", _shortName);
}

// ── setLongName ───────────────────────────────────────────────────────────
void MeshNode::setLongName(const char* name)
{
    if (name == nullptr || name[0] == '\0') return;

    strncpy(_longName, name, sizeof(_longName) - 1);
    _longName[sizeof(_longName) - 1] = '\0';

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK)
    {
        const esp_err_t err = nvs_set_str(h, NVS_KEY_LONG, _longName);
        if (err == ESP_OK) nvs_commit(h);
        else ESP_LOGW(TAG, "nvs_set_str(long) failed: %s", esp_err_to_name(err));
        nvs_close(h);
    }

    ESP_LOGI(TAG, "Long name updated: \"%s\"", _longName);
}

// ── nextPacketId ──────────────────────────────────────────────────────────
uint32_t MeshNode::nextPacketId() const
{
    // Use the hardware RNG.  Meshtastic requires packet IDs to be random
    // (they form the nonce for AES-CTR; reuse would allow ciphertext XOR).
    // Reserve 0 as an invalid sentinel — extremely unlikely but worth guarding.
    uint32_t id;
    do { id = esp_random(); } while (id == 0);
    return id;
}

/* extern */
MeshNode Node;
