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
#include <mbedtls/ecp.h>
#include <mbedtls/bignum.h>
#include <cstdio>
#include <cstring>
#include <cinttypes>

// mbedTLS 3.x (ESP-IDF 5.x) requires a non-NULL f_rng for mbedtls_ecp_mul.
// Passing NULL returns MBEDTLS_ERR_ECP_BAD_INPUT_DATA (-0x4F80) immediately.
// Wrap the ESP32 hardware RNG so the Montgomery ladder can apply blinding.
static int _ecp_rng(void* /*ctx*/, unsigned char* buf, size_t len)
{
    esp_fill_random(buf, len);
    return 0;
}

static const char* TAG          = "meshnode";
static const char* NVS_NS       = "mesh";
static const char* NVS_KEY_SHORT   = "short";
static const char* NVS_KEY_LONG    = "long";
static const char* NVS_KEY_PUBKEY  = "pubkey";
static const char* NVS_KEY_PRIVKEY = "privkey";

// ── init ──────────────────────────────────────────────────────────────────
void MeshNode::init()
{
    // ── Derive node ID from BT MAC (lower 4 bytes) ────────────────────────
    // esp_read_mac(ESP_MAC_BT) returns the Bluetooth MAC directly, matching
    // exactly the node ID a stock Meshtastic build would assign to this chip.
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_BT);
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

    // ── Load or generate persistent X25519 key pair ──────────────────────
    // Meshtastic 2.5+ uses PKC (ch=0x00 packets) for direct messages once
    // a node's public_key is known.  We need a proper X25519 key pair so we
    // can decrypt those packets.
    //
    // "privkey" (NVS blob, 32 B): clamped X25519 private key
    // "pubkey"  (NVS blob, 32 B): X25519 public key = X25519(priv, G)
    //
    // Nodes with only the old random "pubkey" (pre-PKC) will regenerate
    // because "privkey" will be absent.
    bool haveKey = false;
    if (nvsReady)
    {
        nvs_handle_t hk;
        if (nvs_open(NVS_NS, NVS_READWRITE, &hk) == ESP_OK)
        {
            size_t kLen = 32;
            bool havePriv = (nvs_get_blob(hk, NVS_KEY_PRIVKEY, _privateKey, &kLen) == ESP_OK
                             && kLen == 32);
            kLen = 32;
            bool havePub  = (nvs_get_blob(hk, NVS_KEY_PUBKEY,  _publicKey,  &kLen) == ESP_OK
                             && kLen == 32);

            if (havePriv && havePub)
            {
                // Verify the stored public key matches what we derive from the
                // private key.  If the key pair was generated by a buggy older
                // version of sharedSecret() (wrong byte order), the stored
                // public key won't match and we must regenerate.
                uint8_t base[32] = {};
                base[0] = 9; // Curve25519 base point
                uint8_t derivedPub[32] = {};
                if (sharedSecret(base, derivedPub) &&
                    memcmp(derivedPub, _publicKey, 32) == 0)
                {
                    haveKey = true;
                    ESP_LOGI(TAG, "Loaded X25519 key pair from NVS (pub=%02x%02x%02x%02x...)",
                             _publicKey[0], _publicKey[1], _publicKey[2], _publicKey[3]);
                }
                else
                {
                    ESP_LOGW(TAG, "Stored public key doesn't match private key"
                             " — X25519 method changed, regenerating key pair");
                    havePriv = false;
                    havePub  = false;
                }
            }
            if (!havePriv || !havePub)
            {
                // Generate new clamped X25519 private key
                esp_fill_random(_privateKey, 32);
                _privateKey[0]  &= 248; // clear low 3 bits (cofactor)
                _privateKey[31] &= 127; // clear high bit
                _privateKey[31] |=  64; // set second-high bit

                // Derive public key = X25519(priv, base_point=9)
                uint8_t base[32] = {};
                base[0] = 9; // Curve25519 base point u-coordinate
                if (sharedSecret(base, _publicKey))
                {
                    if (nvs_set_blob(hk, NVS_KEY_PRIVKEY, _privateKey, 32) == ESP_OK &&
                        nvs_set_blob(hk, NVS_KEY_PUBKEY,  _publicKey,  32) == ESP_OK)
                    {
                        nvs_commit(hk);
                        haveKey = true;
                        // Log the public key so it can be verified against what other nodes see
                        char pubhex[65] = {};
                        for (int i = 0; i < 32; i++)
                            snprintf(pubhex + i*2, 3, "%02x", _publicKey[i]);
                        ESP_LOGI(TAG, "Generated new X25519 key pair — pub=%s", pubhex);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Failed to store X25519 key pair in NVS");
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "X25519 key derivation failed");
                }
            }
            nvs_close(hk);
        }
    }
    if (!haveKey)
    {
        // Fallback: deterministic from MAC when NVS unavailable.
        // Not a valid X25519 key pair — PKC decryption will fail, but at least
        // the node advertises something in the public_key field.
        memset(_privateKey, 0, 32);
        memcpy(_privateKey, _mac, 6);
        _privateKey[0]  &= 248;
        _privateKey[31] |=  64;
        memset(_publicKey, 0, 32);
        memcpy(_publicKey, _mac, 6);
        _publicKey[6] = 0x42;
        ESP_LOGW(TAG, "Using fallback key (NVS unavailable)");
    }

    ESP_LOGI(TAG, "Node ID: %s  short: \"%s\"  long: \"%s\"",
             _nodeIdStr, _shortName, _longName);

    // Log full public key so the user can compare with what other nodes
    // see for this device (e.g. Meshtastic app → node details → public key).
    {
        char pubhex[65] = {};
        for (int i = 0; i < 32; i++)
            snprintf(pubhex + i*2, 3, "%02x", _publicKey[i]);
        ESP_LOGI(TAG, "Public key: %s", pubhex);
    }
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

// ── sharedSecret ──────────────────────────────────────────────────────────
// Computes X25519(our_private_key, theirPub32) → out32 (32-byte shared secret).
//
// Implementation mirrors Meshtastic's ESP32CryptoEngine::encryptCurve25519()
// EXACTLY — byte-for-byte identical code path using MBEDTLS_PRIVATE() which
// is the correct mbedTLS 3.x way to access struct members without defining
// MBEDTLS_ALLOW_PRIVATE_ACCESS.
//
// Previous attempt using ecp_point_read_binary / ecp_point_write_binary
// produced a wrong public key: ecp_point_write_binary may emit a format-byte
// prefix or use a different byte order in certain ESP-IDF builds, so the
// public key we advertised in NodeInfo did not match our private key from the
// peer's perspective — they encrypted with a different shared secret than the
// one we computed, causing PKC decryption to produce garbage.
//
// This version is guaranteed to match Meshtastic because it uses:
//   mpi_read_binary_le  (Q.X ← peer pub, LE)   — same as Meshtastic
//   mpi_lset            (Q.Z ← 1)               — same as Meshtastic
//   ecp_mul             (R = d * Q, with RNG)    — same as Meshtastic
//   mpi_write_binary_le (out ← R.X, LE)          — same as Meshtastic
bool MeshNode::sharedSecret(const uint8_t* theirPub32, uint8_t* out32) const
{
    mbedtls_ecp_group grp;
    mbedtls_mpi       d;
    mbedtls_ecp_point Q, R;

    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);
    mbedtls_ecp_point_init(&R);

    bool ok = false;
    int  rc = 0;
    do {
        rc = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519);
        if (rc != 0) { ESP_LOGE(TAG, "ecp_group_load rc=%d", rc); break; }

        // Private scalar — 32 bytes LE (clamped at generation time)
        rc = mbedtls_mpi_read_binary_le(&d, _privateKey, 32);
        if (rc != 0) { ESP_LOGE(TAG, "mpi_read_binary_le(d) rc=%d", rc); break; }

        // Peer's public u-coordinate — 32 bytes LE.
        // Use MBEDTLS_PRIVATE() + mpi_read_binary_le exactly as Meshtastic does,
        // NOT ecp_point_read_binary (which may apply transformations).
        rc = mbedtls_mpi_read_binary_le(&Q.MBEDTLS_PRIVATE(X), theirPub32, 32);
        if (rc != 0) { ESP_LOGE(TAG, "mpi_read_binary_le(Q.X) rc=%d", rc); break; }
        rc = mbedtls_mpi_lset(&Q.MBEDTLS_PRIVATE(Z), 1);
        if (rc != 0) { ESP_LOGE(TAG, "mpi_lset(Q.Z) rc=%d", rc); break; }

        // Montgomery ladder — f_rng mandatory in mbedTLS 3.x (NULL → -0x4F80)
        rc = mbedtls_ecp_mul(&grp, &R, &d, &Q, _ecp_rng, nullptr);
        if (rc != 0) { ESP_LOGE(TAG, "ecp_mul rc=%d (-0x%04x)", rc, (unsigned)-rc); break; }

        // Export result X coordinate — 32 bytes LE, exactly as Meshtastic.
        // Use mpi_write_binary_le directly, NOT ecp_point_write_binary.
        rc = mbedtls_mpi_write_binary_le(&R.MBEDTLS_PRIVATE(X), out32, 32);
        if (rc != 0) { ESP_LOGE(TAG, "mpi_write_binary_le(R.X) rc=%d", rc); break; }

        ok = true;
    } while (false);

    mbedtls_ecp_group_free(&grp);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);
    mbedtls_ecp_point_free(&R);
    return ok;
}

/* extern */
MeshNode Node;
