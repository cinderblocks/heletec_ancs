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
#include <mbedtls/ecdh.h>
#include <mbedtls/ecp.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/platform_util.h>
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

// ── byte-reverse helper for mbedtls ↔ wire format ────────────────────────
// Meshtastic stores X25519 scalars/points on the wire as little-endian
// 32-byte strings (RFC 7748).  mbedtls MPI is big-endian, so every key
// must be byte-reversed on the way in and out of the MPI API.
static void reverseBytes32(const uint8_t* src, uint8_t* dst)
{
    for (int i = 0; i < 32; i++)
        dst[i] = src[31 - i];
}

// ── Hardware RNG entropy source for mbedtls ──────────────────────────────
static int hw_entropy_source(void* /*data*/, unsigned char* output,
                              size_t len, size_t* olen)
{
    esp_fill_random(output, len);
    *olen = len;
    return 0;
}

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

    // ── PKC keypair (encrypted mode) or all-zeros (licensed mode) ──────────
#if CONFIG_LORA_IS_LICENSED
    // IsLicensed mode — no encryption.
    // Public key is all-zeros; no private key is generated or stored.
    memset(_publicKey, 0, 32);
    memset(_privateKey, 0, 32);
    _hasPkcKeys = false;

    ESP_LOGI(TAG, "Node ID: %s  short: \"%s\"  long: \"%s\"  (IsLicensed, no encryption)",
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

    // ── Derive public key from private key using mbedtls X25519 ──────────
    // Meshtastic wire keys are little-endian (RFC 7748).  mbedtls MPI is
    // big-endian, so we reverse the private key on import and the public key
    // on export.  Curve25519 clamping is done internally by mbedtls when
    // using mbedtls_ecp_mul on MBEDTLS_ECP_DP_CURVE25519.
    {
        mbedtls_ecp_group grp;
        mbedtls_ecp_point Q;
        mbedtls_mpi d;

        mbedtls_ecp_group_init(&grp);
        mbedtls_ecp_point_init(&Q);
        mbedtls_mpi_init(&d);

        bool ok = false;
        do {
            if (mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519) != 0)
                break;

            // Import private key (LE wire → BE for mbedtls MPI)
            uint8_t privBE[32];
            reverseBytes32(_privateKey, privBE);
            if (mbedtls_mpi_read_binary(&d, privBE, 32) != 0)
                break;

            // Set up CSPRNG for blinding
            mbedtls_entropy_context entropy;
            mbedtls_ctr_drbg_context ctr_drbg;
            mbedtls_entropy_init(&entropy);
            mbedtls_ctr_drbg_init(&ctr_drbg);
            mbedtls_entropy_add_source(&entropy, hw_entropy_source,
                                       nullptr, 32,
                                       MBEDTLS_ENTROPY_SOURCE_STRONG);
            if (mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func,
                                       &entropy, nullptr, 0) != 0)
            {
                mbedtls_ctr_drbg_free(&ctr_drbg);
                mbedtls_entropy_free(&entropy);
                break;
            }

            // Q = d * G (base point multiplication)
            if (mbedtls_ecp_mul(&grp, &Q, &d, &grp.G,
                                mbedtls_ctr_drbg_random, &ctr_drbg) != 0)
            {
                mbedtls_ctr_drbg_free(&ctr_drbg);
                mbedtls_entropy_free(&entropy);
                break;
            }

            // Export public key (X coordinate only for Curve25519)
            // mbedtls gives BE; reverse back to LE for wire/storage.
            uint8_t pubBE[32];
            if (mbedtls_mpi_write_binary(&Q.MBEDTLS_PRIVATE(X), pubBE, 32) != 0)
            {
                mbedtls_ctr_drbg_free(&ctr_drbg);
                mbedtls_entropy_free(&entropy);
                break;
            }
            reverseBytes32(pubBE, _publicKey);
            ok = true;

            mbedtls_ctr_drbg_free(&ctr_drbg);
            mbedtls_entropy_free(&entropy);
        } while (false);

        mbedtls_mpi_free(&d);
        mbedtls_ecp_point_free(&Q);
        mbedtls_ecp_group_free(&grp);

        if (ok)
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
// X25519 ECDH: shared = ECDH(our_private, remote_public).
//
// Input keys are little-endian (Meshtastic/RFC 7748 wire format).  They are
// reversed to big-endian for the mbedtls MPI API.
//
// Output: the 32-byte shared secret is written to sharedOut in LITTLE-ENDIAN
// order, matching the output of Curve25519::eval() (Arduino Crypto library)
// that Meshtastic ESP32 firmware uses directly as the AES-256 key.
bool MeshNode::computeSharedSecret(const uint8_t* remotePubKey,
                                    uint8_t* sharedOut) const
{
    if (!_hasPkcKeys || remotePubKey == nullptr || sharedOut == nullptr)
        return false;

    mbedtls_ecp_group grp;
    mbedtls_ecp_point Qr;   // remote public key point
    mbedtls_mpi d;           // our private key scalar
    mbedtls_mpi z;           // shared secret (X coordinate)

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&Qr);
    mbedtls_mpi_init(&d);
    mbedtls_mpi_init(&z);

    bool ok = false;
    do {
        if (mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519) != 0)
            break;

        // Import our private key (LE wire → BE for mbedtls MPI)
        uint8_t privBE[32];
        reverseBytes32(_privateKey, privBE);
        if (mbedtls_mpi_read_binary(&d, privBE, 32) != 0)
            break;

        // Import remote public key as the X coordinate of the point
        // (Curve25519 uses X-only representation; LE wire → BE for MPI)
        uint8_t remoteBE[32];
        reverseBytes32(remotePubKey, remoteBE);
        if (mbedtls_mpi_read_binary(&Qr.MBEDTLS_PRIVATE(X), remoteBE, 32) != 0)
            break;
        if (mbedtls_mpi_lset(&Qr.MBEDTLS_PRIVATE(Z), 1) != 0)
            break;

        // Set up CSPRNG for blinding
        mbedtls_entropy_context entropy;
        mbedtls_ctr_drbg_context ctr_drbg;
        mbedtls_entropy_init(&entropy);
        mbedtls_ctr_drbg_init(&ctr_drbg);
        mbedtls_entropy_add_source(&entropy, hw_entropy_source,
                                   nullptr, 32,
                                   MBEDTLS_ENTROPY_SOURCE_STRONG);
        if (mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func,
                                   &entropy, nullptr, 0) != 0)
        {
            mbedtls_ctr_drbg_free(&ctr_drbg);
            mbedtls_entropy_free(&entropy);
            break;
        }

        // ECDH: z = d * Qr (scalar multiplication of remote point by our scalar)
        if (mbedtls_ecdh_compute_shared(&grp, &z, &Qr, &d,
                                         mbedtls_ctr_drbg_random,
                                         &ctr_drbg) != 0)
        {
            mbedtls_ctr_drbg_free(&ctr_drbg);
            mbedtls_entropy_free(&entropy);
            break;
        }

        // Export shared secret.
        // Meshtastic ESP32 firmware uses Curve25519::eval() (Arduino Crypto
        // library, rweather/arduinolibs), which outputs a little-endian
        // 32-byte shared secret used directly as the AES-256 key.
        // mbedtls_mpi_write_binary produces big-endian, so reverse to LE.
        uint8_t zBE[32] = {};
        if (mbedtls_mpi_write_binary(&z, zBE, 32) != 0)
        {
            mbedtls_ctr_drbg_free(&ctr_drbg);
            mbedtls_entropy_free(&entropy);
            break;
        }
        reverseBytes32(zBE, sharedOut);  // BE → LE to match Meshtastic AES key
        ok = true;

        mbedtls_ctr_drbg_free(&ctr_drbg);
        mbedtls_entropy_free(&entropy);
    } while (false);

    // Zeroize sensitive material
    mbedtls_mpi_free(&z);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Qr);
    mbedtls_ecp_group_free(&grp);

    if (!ok)
        ESP_LOGW(TAG, "ECDH shared secret computation failed");
    return ok;
}

/* extern */
MeshNode Node;
