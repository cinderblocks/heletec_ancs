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
#include "mesh_crypto.h"
#include "sdkconfig.h"

#include <esp_mac.h>
#include <esp_log.h>
#include <esp_random.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <cstdio>
#include <cstring>
#include <cinttypes>

static const char* TAG          = "meshnode";
static const char* NVS_NS       = "mesh";
static const char* NVS_KEY_SHORT   = "short";
static const char* NVS_KEY_LONG    = "long";
static const char* NVS_KEY_PRIVKEY = "privkey";

#ifndef CONFIG_LORA_IS_LICENSED
#  define CONFIG_LORA_IS_LICENSED 0
#endif

// ── init ──────────────────────────────────────────────────────────────────
void MeshNode::init()
{
    // ── Derive node ID from WiFi STA MAC (lower 4 bytes) ──────────────────
    // Meshtastic firmware derives nodeNum from ESP_MAC_WIFI_STA (the base
    // eFuse MAC), NOT ESP_MAC_BT.  On ESP32-S3 the two MACs differ by +1 in
    // the last byte, so using ESP_MAC_BT would generate a node ID that no
    // stock Meshtastic node would ever assign to this chip — causing the User
    // proto "id" field ("!xxxxxxxx") to disagree with the OTA "from" field
    // that a receiving node sees, and some firmware builds discard the NODEINFO.
    //
    // The macaddr field (User proto field 4) must also come from WIFI_STA to
    // match what Meshtastic derives when it computes a node's BLE address
    // from the factory MAC for display purposes.
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    memcpy(_mac, mac, 6);

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
        ESP_LOGW(TAG, "NVS partition needs erase: %s", esp_err_to_name(err));
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    const bool nvsReady = (err == ESP_OK);

    // Populate in-memory cache with Kconfig defaults first, then try NVS.
    strncpy(_shortName, CONFIG_MESH_NODE_SHORT_NAME, sizeof(_shortName) - 1);
    _shortName[sizeof(_shortName) - 1] = '\0';
    strncpy(_longName,  CONFIG_MESH_NODE_LONG_NAME,  sizeof(_longName)  - 1);
    _longName[sizeof(_longName) - 1] = '\0';

    if (nvsReady)
    {
        nvs_handle_t h;
        if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK)
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
        ESP_LOGW(TAG, "NVS init failed — using Kconfig defaults");
    }

    // ── PKC keypair (encrypted mode) or all-zeros (licensed mode) ──────────
#if CONFIG_LORA_IS_LICENSED
    // IsLicensed mode — no PKC (public-key direct messages).
    // Channel AES-128-CTR encryption is still used on TX for interop.
    // Public key is all-zeros; no private key is generated or stored.
    memset(_publicKey, 0, 32);
    memset(_privateKey, 0, 32);
    _hasPkcKeys = false;

    ESP_LOGI(TAG, "Node ID: %s  short: \"%s\"  long: \"%s\"  (IsLicensed, no PKC)",
             _nodeIdStr, _shortName, _longName);
#else
    // Encrypted mode — generate or load a persistent X25519 keypair.
    // The private key is stored in NVS as a 32-byte blob.  If absent
    // (first boot), generate a new one using the hardware RNG and persist it.
    bool keyLoaded = false;
    if (nvsReady)
    {
        nvs_handle_t h;
        if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK)
        {
            size_t klen = 32;
            if (nvs_get_blob(h, NVS_KEY_PRIVKEY, _privateKey, &klen) == ESP_OK
                && klen == 32)
            {
                keyLoaded = true;
                ESP_LOGI(TAG, "Loaded X25519 private key from NVS");
            }
            nvs_close(h);
        }
    }

    if (!keyLoaded)
    {
        // Generate a fresh 32-byte random private key
        esp_fill_random(_privateKey, 32);
        ESP_LOGI(TAG, "Generated new X25519 private key");

        // Persist to NVS
        if (nvsReady)
        {
            nvs_handle_t h;
            if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK)
            {
                if (nvs_set_blob(h, NVS_KEY_PRIVKEY, _privateKey, 32) == ESP_OK)
                    nvs_commit(h);
                else
                    ESP_LOGW(TAG, "Failed to persist private key to NVS");
                nvs_close(h);
            }
        }
    }

    // ── Derive public key from private key via platform-free mc_x25519PublicKey ──
    // Keys are little-endian (RFC 7748 / Meshtastic wire format).
    // mc_x25519PublicKey delegates to mbedtls ECP Curve25519.
    if (mc_x25519PublicKey(_privateKey, _publicKey))
    {
        _hasPkcKeys = true;
        ESP_LOGI(TAG, "X25519 public key: %02x%02x%02x%02x...%02x%02x%02x%02x",
                 _publicKey[0], _publicKey[1], _publicKey[2], _publicKey[3],
                 _publicKey[28], _publicKey[29], _publicKey[30], _publicKey[31]);
    }
    else
    {
        ESP_LOGE(TAG, "X25519 key derivation failed — PKC disabled");
        memset(_publicKey, 0, 32);
        _hasPkcKeys = false;
    }

    ESP_LOGI(TAG, "Node ID: %s  short: \"%s\"  long: \"%s\"  (encrypted, PKC %s)",
             _nodeIdStr, _shortName, _longName,
             _hasPkcKeys ? "enabled" : "DISABLED");
#endif
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
    uint32_t id;
    do { id = esp_random(); } while (id == 0);
    return id;
}

// ── computeSharedSecret ───────────────────────────────────────────────────
// Delegates to platform-free mc_x25519SharedSecret() in mesh_crypto.cxx.
// See mesh_crypto.h for the full endianness and wire-format documentation.
bool MeshNode::computeSharedSecret(const uint8_t* remotePubKey,
                                    uint8_t* sharedOut) const
{
    if (!_hasPkcKeys || remotePubKey == nullptr || sharedOut == nullptr)
        return false;
    return mc_x25519SharedSecret(_privateKey, remotePubKey, sharedOut);
}

/* extern */
MeshNode Node;
