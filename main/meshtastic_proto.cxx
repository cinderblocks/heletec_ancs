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

/**
 * meshtastic_proto.cxx — Meshtastic protocol implementation.
 *
 * Covers:
 *   - AES-128-CTR channel decryption and encryption (_decrypt)
 *   - AES-256-CTR X25519 PKC direct message decryption (_decryptPkc)
 *   - PKC packet buffering and retry (_bufferPkcPacket, _retryPkcBuffer)
 *   - Protobuf decoder for Data, Position, and User messages
 *   - Protobuf encoder helpers and all OTA packet builders
 *   - TX senders (sendPosition, sendNodeInfo, sendTelemetry, sendMapReport)
 *   - Neighbour table management (_upsertNeighbor, neighborCount, …)
 *   - Packet dispatch (_processPacket)
 *   - Thread-safe accessors (lastMessage, stats)
 *
 * SX1262 hardware layer lives in sx1262.cxx.
 * FreeRTOS task entry point lives in lora.cxx.
 */

#include "lora.h"
#include "lora_internal.h"
#include "mesh_codec.h"
#include "gps.h"
#include "hardware.h"
#include "meshnode.h"

#include <mbedtls/aes.h>
#include <mbedtls/platform_util.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cinttypes>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <algorithm>

static const char* TAG = "lora";

// ── Meshtastic default channel PSK ────────────────────────────────────────
// AES-128 key for the default LongFast channel (factory default on every
// Meshtastic device).  Used for both RX decryption and TX encryption so
// this node interoperates with the existing encrypted mesh.
// CONFIG_LORA_IS_LICENSED controls whether OUR node encrypts TX packets:
//   - false (default): encrypt TX with AES-128-CTR, decrypt RX, support PKC DMs
//   - true (ham mode):  TX plaintext, still attempt AES-128-CTR RX decryption
//     since other nodes in the mesh still encrypt with the channel PSK.
/* static */ const uint8_t LoRa::DEFAULT_PSK[16] = {
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};

// ── _decrypt ──────────────────────────────────────────────────────────────
// Meshtastic AES-128-CTR cipher (decrypt == encrypt for CTR mode).
// Used for BOTH decrypting incoming packets and encrypting outgoing ones.
//
// Nonce layout (CryptoEngine::initCounter() in Meshtastic firmware):
//   bytes  [0:3]  = packetId LE
//   bytes  [4:7]  = 0x00 × 4
//   bytes  [8:11] = fromNode LE
//   bytes [12:15] = 0x00 × 4
bool LoRa::_decrypt(const uint8_t* in, size_t len,
                    uint32_t packetId, uint32_t fromNode,
                    uint8_t* out)
{
    if (len == 0) return false;

    uint8_t nonce[16] = {};
    nonce[0]  = static_cast<uint8_t>(packetId);
    nonce[1]  = static_cast<uint8_t>(packetId >>  8);
    nonce[2]  = static_cast<uint8_t>(packetId >> 16);
    nonce[3]  = static_cast<uint8_t>(packetId >> 24);
    // [4:7]  = 0
    nonce[8]  = static_cast<uint8_t>(fromNode);
    nonce[9]  = static_cast<uint8_t>(fromNode >>  8);
    nonce[10] = static_cast<uint8_t>(fromNode >> 16);
    nonce[11] = static_cast<uint8_t>(fromNode >> 24);
    // [12:15] = 0

    uint8_t streamBlock[16] = {};
    size_t  nc_off = 0;

    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    bool ok = false;
    if (mbedtls_aes_setkey_enc(&ctx, DEFAULT_PSK, 128) == 0)
        ok = (mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce, streamBlock, in, out) == 0);
    mbedtls_aes_free(&ctx);
    return ok;
}

// ── _decryptPkc ───────────────────────────────────────────────────────────
// PKC (public-key cryptography) direct message decryption.
// Meshtastic uses chanHash=0x00 to mark PKC-encrypted packets:
//   1. Look up the sender's X25519 public key in our neighbour table.
//   2. Compute the ECDH shared secret (our private key × their public key).
//   3. Use the 32-byte shared secret as an AES-256-CTR key.
//   4. Nonce layout — SAME as channel encryption:
//      [packetId LE (4B) | 0 (4B) | fromNode LE (4B) | 0 (4B)]
bool LoRa::_decryptPkc(const uint8_t* in, size_t len,
                        uint32_t packetId, uint32_t fromNode,
                        uint8_t* out)
{
#if CONFIG_LORA_IS_LICENSED
    (void)in; (void)len; (void)packetId; (void)fromNode; (void)out;
    return false;  // PKC disabled in licensed mode
#else
    if (len == 0) return false;
    if (!Node.hasPkcKeys()) return false;

    // ── Find the sender's public key in the neighbour table ──────────────
    uint8_t remotePub[32] = {};
    bool found = false;
    {
        portENTER_CRITICAL(&_neighborLock);
        for (size_t i = 0; i < NEIGHBOR_MAX; i++)
        {
            if (_neighbors[i].occupied &&
                _neighbors[i].user.fromNode == fromNode &&
                _neighbors[i].user.hasPublicKey)
            {
                memcpy(remotePub, _neighbors[i].user.publicKey, 32);
                found = true;
                break;
            }
        }
        portEXIT_CRITICAL(&_neighborLock);
    }

    if (!found)
    {
        ESP_LOGI(TAG, "PKC: no public key for node 0x%08" PRIx32, fromNode);
        return false;
    }

    // ── Compute ECDH shared secret ──────────────────────────────────────
    uint8_t sharedSecret[32] = {};
    if (!Node.computeSharedSecret(remotePub, sharedSecret))
    {
        ESP_LOGW(TAG, "PKC: ECDH failed for node 0x%08" PRIx32, fromNode);
        return false;
    }

    // ── AES-256-CTR with the shared secret as key ───────────────────────
    uint8_t nonce[16] = {};
    nonce[0]  = static_cast<uint8_t>(packetId);
    nonce[1]  = static_cast<uint8_t>(packetId >>  8);
    nonce[2]  = static_cast<uint8_t>(packetId >> 16);
    nonce[3]  = static_cast<uint8_t>(packetId >> 24);
    // [4:7]  = 0
    nonce[8]  = static_cast<uint8_t>(fromNode);
    nonce[9]  = static_cast<uint8_t>(fromNode >>  8);
    nonce[10] = static_cast<uint8_t>(fromNode >> 16);
    nonce[11] = static_cast<uint8_t>(fromNode >> 24);
    // [12:15] = 0

    uint8_t streamBlock[16] = {};
    size_t  nc_off = 0;

    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    bool ok = false;
    if (mbedtls_aes_setkey_enc(&ctx, sharedSecret, 256) == 0)
        ok = (mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce, streamBlock, in, out) == 0);
    mbedtls_aes_free(&ctx);

    // Zeroize the shared secret from stack
    mbedtls_platform_zeroize(sharedSecret, sizeof(sharedSecret));

    return ok;
#endif
}

// ── _bufferPkcPacket ──────────────────────────────────────────────────────
// Store a raw undecryptable PKC packet for later retry once we learn the
// sender's public key.  Oldest / expired entries are evicted.
void LoRa::_bufferPkcPacket(const uint8_t* buf, uint8_t len,
                             int16_t rssi, float snr)
{
    if (len < MESH_HDR + 1) return;

    const uint32_t pktId = buf[8]  | (uint32_t)buf[9]  <<  8
                         | (uint32_t)buf[10] << 16 | (uint32_t)buf[11] << 24;
    const uint32_t from  = buf[4]  | (uint32_t)buf[5]  <<  8
                         | (uint32_t)buf[6]  << 16 | (uint32_t)buf[7]  << 24;
    const TickType_t now = xTaskGetTickCount();

    // Dedup within buffer — don't store the same packet twice
    for (size_t i = 0; i < PKC_PENDING_MAX; i++)
        if (_pkcPending[i].occupied && _pkcPending[i].pktId == pktId)
            return;

    // Find an empty or expired slot; fall back to evicting oldest
    int slot = -1;
    TickType_t oldestAge = 0;
    int oldestSlot = 0;

    for (int i = 0; i < (int)PKC_PENDING_MAX; i++)
    {
        if (_pkcPending[i].occupied &&
            (now - _pkcPending[i].tick) > pdMS_TO_TICKS(PKC_PENDING_EXPIRE_MS))
            _pkcPending[i].occupied = false;  // expire

        if (!_pkcPending[i].occupied) { slot = i; break; }

        const TickType_t age = now - _pkcPending[i].tick;
        if (age > oldestAge) { oldestAge = age; oldestSlot = i; }
    }
    if (slot == -1) slot = oldestSlot;

    memcpy(_pkcPending[slot].data, buf, len);
    _pkcPending[slot].len      = len;
    _pkcPending[slot].rssi     = rssi;
    _pkcPending[slot].snr      = snr;
    _pkcPending[slot].fromNode = from;
    _pkcPending[slot].pktId    = pktId;
    _pkcPending[slot].tick     = now;
    _pkcPending[slot].occupied = true;

    ESP_LOGI(TAG, "PKC: buffered pkt 0x%08" PRIx32 " from 0x%08" PRIx32
             " (%u bytes) for later decryption",
             pktId, from, (unsigned)len);
}

// ── _retryPkcBuffer ───────────────────────────────────────────────────────
// Called after we learn a node's public key (from their NODEINFO).
// Tries to decrypt any buffered PKC packets from that node and dispatches
// successfully decrypted messages (TEXT is displayed, others are logged).
void LoRa::_retryPkcBuffer(uint32_t fromNode)
{
    for (size_t i = 0; i < PKC_PENDING_MAX; i++)
    {
        if (!_pkcPending[i].occupied || _pkcPending[i].fromNode != fromNode)
            continue;

        const uint8_t* buf    = _pkcPending[i].data;
        const uint8_t  pktLen = _pkcPending[i].len;
        const int16_t  rssi   = _pkcPending[i].rssi;
        const float    snr    = _pkcPending[i].snr;

        // Parse OTA header
        const uint32_t to    = buf[0]  | (uint32_t)buf[1]  <<  8
                             | (uint32_t)buf[2]  << 16 | (uint32_t)buf[3]  << 24;
        const uint32_t from  = buf[4]  | (uint32_t)buf[5]  <<  8
                             | (uint32_t)buf[6]  << 16 | (uint32_t)buf[7]  << 24;
        const uint32_t pktId = buf[8]  | (uint32_t)buf[9]  <<  8
                             | (uint32_t)buf[10] << 16 | (uint32_t)buf[11] << 24;

        ESP_LOGI(TAG, "PKC: retrying buffered pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32, pktId, from);

        const uint8_t* ciphertext = buf + MESH_HDR;
        const size_t   cipherLen  = pktLen - MESH_HDR;
        uint8_t plain[256] = {};

        if (!_decryptPkc(ciphertext, cipherLen, pktId, from, plain))
        {
            ESP_LOGW(TAG, "PKC: retry decrypt still failed for 0x%08" PRIx32, pktId);
            _pkcPending[i].occupied = false;
            continue;
        }

        ESP_LOGI(TAG, "PKC: retry decrypt SUCCESS for pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32, pktId, from);

        portENTER_CRITICAL(&_statsLock);
        _stats.decryptOk++;
        portEXIT_CRITICAL(&_statsLock);

        // Parse Data proto
        uint32_t       portnum    = 0;
        const uint8_t* payload    = nullptr;
        size_t         payloadLen = 0;
        bool           wantResp   = false;

        if (!_parseData(plain, cipherLen, portnum, payload, payloadLen, wantResp))
        {
            ESP_LOGW(TAG, "PKC: retry parse failed for 0x%08" PRIx32, pktId);
            _pkcPending[i].occupied = false;
            continue;
        }

        ESP_LOGI(TAG, "PKC: retry decoded portnum=%u payloadLen=%u from 0x%08" PRIx32,
                 portnum, (unsigned)payloadLen, from);

        // ── Dispatch — TEXT_MESSAGE_APP is the primary PKC DM use case ───
        if (portnum == PORT_TEXT && payload != nullptr && payloadLen > 0)
        {
            const bool isDirect    = (to == Node.nodeId());
            const bool isBroadcast = (to == 0xFFFFFFFFu);
            if (isDirect || isBroadcast)
            {
                MeshMessage msg;
                msg.fromNode = from;
                msg.rssi     = rssi;
                msg.snr      = snr;
                msg.valid    = true;
                const size_t copyLen = std::min(payloadLen, sizeof(msg.text) - 1);
                memcpy(msg.text, payload, copyLen);
                msg.text[copyLen] = '\0';

                ESP_LOGI(TAG, "PKC DM (buffered) from 0x%08" PRIx32
                         " (rssi=%d snr=%.1f): %s",
                         from, rssi, (double)snr, msg.text);

                portENTER_CRITICAL(&_statsLock);
                _stats.textMessages++;
                _stats.lastRssi = rssi;
                _stats.lastSnr  = snr;
                portEXIT_CRITICAL(&_statsLock);

                portENTER_CRITICAL(&_msgLock);
                _lastMsg = msg;
                portEXIT_CRITICAL(&_msgLock);

                Heltec.notifyDraw(Hardware::DRAW_LORA);
            }
        }
        else
        {
            ESP_LOGI(TAG, "PKC: retry portnum=%u from 0x%08" PRIx32
                     " — processed (non-text)", portnum, from);
        }

        _pkcPending[i].occupied = false;
    }
}

// ── _parseData ────────────────────────────────────────────────────────────
bool LoRa::_parseData(const uint8_t* data, size_t len,
                      uint32_t& portnum,
                      const uint8_t*& payload, size_t& payloadLen,
                      bool& wantResponse)
{
    return mc_parseData(data, len, portnum, payload, payloadLen, wantResponse);
}

// ── _encodePosition ───────────────────────────────────────────────────────
/* static */ size_t LoRa::_encodePosition(uint8_t* buf, size_t cap,
                                           int32_t lat_i, int32_t lon_i,
                                           int32_t alt_m,
                                           uint32_t /*pdop_x100*/, uint32_t sats,
                                           uint32_t unixTime,
                                           uint32_t speed_cm_s,
                                           uint32_t track_x100)
{
    return mc_encodePosition(buf, cap, lat_i, lon_i, alt_m, sats, unixTime,
                              speed_cm_s, track_x100);
}

// ── _encodeUser ───────────────────────────────────────────────────────────
// Encode a Meshtastic User proto (NODEINFO_APP payload).
//
// meshtastic/mesh.proto User message fields sent:
//   Field 1 (id,           string):  "!xxxxxxxx"
//   Field 2 (long_name,    string):  up to 32 chars
//   Field 3 (short_name,   string):  up to 4 chars
//   Field 4 (macaddr,      bytes):   6-byte BT MAC
//   Field 5 (hw_model,     enum):    HardwareModel varint (tag 0x28)
//   Field 7 (role,         enum):    DeviceRole varint (tag 0x38)
//   Field 8 (public_key,   bytes):   32-byte X25519 public key (tag 0x42)
//   Field 9 (is_unmessageable, bool):false (tag 0x48), required by Meshtastic 2.5+
//
// buf must be at least 120 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeUser(uint8_t* buf, size_t cap,
                                       uint32_t nodeId,
                                       const char* longName,
                                       const char* shortName)
{
    const uint8_t* pubKey = nullptr;
#if !CONFIG_LORA_IS_LICENSED
    if (Node.hasPkcKeys()) pubKey = Node.publicKey();
#endif
    return mc_encodeUser(buf, cap, nodeId, longName, shortName,
                         Node.macaddr(), HW_MODEL,
                         static_cast<uint8_t>(CONFIG_MESH_NODE_ROLE),
                         pubKey,
                         static_cast<bool>(CONFIG_LORA_IS_LICENSED));
}

// ── _encodeData ───────────────────────────────────────────────────────────
// Wrap an encoded application payload in a Meshtastic Data proto.
//
// meshtastic/mesh.proto Data message — verified against live hex dumps:
//   Field 1 (portnum,       PortNum):  varint      — tag = 0x08
//   Field 2 (payload,       bytes):    LEN         — tag = 0x12
//   Field 3 (want_response, bool):     varint      — tag = 0x18
//   Field 4 (dest,          fixed32):  4-byte LE   — tag = 0x25
//   Field 6 (request_id,    fixed32):  4-byte LE   — tag = 0x35
//   Field 9 (ok_to_mqtt,    bool):     varint      — tag = 0x48
//
// *** BUG HISTORY (field 6 vs 7): Previously used tag 0x3D (field 7 = reply_id)
// but Meshtastic's Router.cpp correlates by request_id (field 6, tag 0x35).
// Confirmed via live hex dump: "35 70 bc 3d 7d" = tag 0x35, field 6 request_id. ***
//
// buf must be at least payloadLen + 32 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeData(uint8_t* buf, size_t cap,
                                       uint32_t portnum,
                                       const uint8_t* payload, size_t payloadLen,
                                       bool want_response,
                                       uint32_t dest,
                                       uint32_t /*source*/,
                                       uint32_t requestId)
{
    return mc_encodeData(buf, cap, portnum, payload, payloadLen,
                         want_response, dest, requestId);
}

// ── _buildTxPacket ────────────────────────────────────────────────────────
// Build a complete Meshtastic OTA packet ready for transmit().
// In encrypted mode: Data proto is AES-128-CTR encrypted with channel PSK.
// In IsLicensed mode: Data proto is sent as plaintext (no encryption).
//
// Layout of out[]:
//   [0..15]  16-byte OTA header (plaintext)
//   [16..]   AES-128-CTR encrypted (or plaintext) Data proto
//
// out must be at least 256 bytes.  Returns false only on encoding error.
bool LoRa::_buildTxPacket(uint8_t* out, uint8_t& outLen,
                           uint32_t portnum,
                           const uint8_t* payload, size_t payloadLen,
                           bool want_response,
                           uint32_t to,
                           uint32_t requestId)
{
    if (payload == nullptr || payloadLen == 0 || payloadLen > 220) return false;

    const uint32_t packetId = Node.nextPacketId();
    const uint32_t fromNode = Node.nodeId();
    const bool     unicast  = (to != 0xFFFFFFFF);

    // 1. Encode Data proto.
    uint8_t dataBuf[244] = {};
    const size_t dataLen = _encodeData(dataBuf, sizeof(dataBuf),
                                        portnum, payload, payloadLen,
                                        want_response,
                                        unicast ? to : 0,
                                        0,
                                        unicast ? requestId : 0);
    if (dataLen == 0 || dataLen > 239) return false;

    // Hex dump of plaintext Data proto for wire-level debugging.
    {
        char hex[120 * 3 + 1] = {};
        for (size_t i = 0; i < dataLen && i < 120; i++)
            snprintf(hex + i * 3, 4, "%02x ", dataBuf[i]);
        ESP_LOGI(TAG, "TX Data proto (%u bytes): %s", (unsigned)dataLen, hex);
    }

    // 2. Build 16-byte OTA header
    out[0] = static_cast<uint8_t>(to);
    out[1] = static_cast<uint8_t>(to >>  8);
    out[2] = static_cast<uint8_t>(to >> 16);
    out[3] = static_cast<uint8_t>(to >> 24);
    out[4] = static_cast<uint8_t>(fromNode);
    out[5] = static_cast<uint8_t>(fromNode >>  8);
    out[6] = static_cast<uint8_t>(fromNode >> 16);
    out[7] = static_cast<uint8_t>(fromNode >> 24);
    out[8]  = static_cast<uint8_t>(packetId);
    out[9]  = static_cast<uint8_t>(packetId >>  8);
    out[10] = static_cast<uint8_t>(packetId >> 16);
    out[11] = static_cast<uint8_t>(packetId >> 24);
    out[12] = unicast ? FLAGS_UNICAST : FLAGS_BROADCAST;
    out[13] = DEFAULT_CHAN_HASH;
    out[14] = 0x00;
    out[15] = 0x00;

    // 3. Encrypt Data proto or send as plaintext
#if CONFIG_LORA_IS_LICENSED
    memcpy(out + MESH_HDR, dataBuf, dataLen);
#else
    if (!_decrypt(dataBuf, dataLen, packetId, fromNode, out + MESH_HDR))
    {
        ESP_LOGW(TAG, "_buildTxPacket: AES-CTR encrypt failed");
        return false;
    }
#endif
    outLen = static_cast<uint8_t>(MESH_HDR + dataLen);

    ESP_LOGI(TAG,
        "OTA hdr: %02x%02x%02x%02x %02x%02x%02x%02x "
        "%02x%02x%02x%02x flags=%02x hash=%02x  port=%u  pktlen=%u",
        out[0],  out[1],  out[2],  out[3],
        out[4],  out[5],  out[6],  out[7],
        out[8],  out[9],  out[10], out[11],
        out[12], out[13],
        portnum, (unsigned)outLen);

    return true;
}

// ── sendPosition ──────────────────────────────────────────────────────────
bool LoRa::sendPosition(double lat, double lng, float altM,
                         uint32_t pdop_x100, uint32_t sats, uint32_t unixTime)
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    if (unixTime == 0)
        unixTime = static_cast<uint32_t>(time(nullptr));

    const int32_t lat_i = static_cast<int32_t>(lat * 1e7);
    const int32_t lon_i = static_cast<int32_t>(lng * 1e7);
    const int32_t alt_m = static_cast<int32_t>(altM);

    const float  speedKmh   = gps.speed();
    const float  courseDeg  = gps.course();
    const uint32_t speed_cm_s = static_cast<uint32_t>(speedKmh * 100.0f / 3.6f);
    const uint32_t track_x100 = (speedKmh >= 2.0f)
                               ? static_cast<uint32_t>(courseDeg * 100.0f)
                               : 0;

    uint8_t posBuf[100] = {};
    const size_t posLen = _encodePosition(posBuf, sizeof(posBuf),
                                           lat_i, lon_i, alt_m,
                                           pdop_x100, sats, unixTime,
                                           speed_cm_s, track_x100);
    if (posLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_POSITION, posBuf, posLen,
                        /*want_response=*/false)) return false;

    ESP_LOGI(TAG, "TX POSITION lat=%.6f lon=%.6f alt=%dm sats=%" PRIu32
             " spd=%.1fkm/h hdg=%.1f ts=%" PRIu32,
             lat, lng, (int)altM, sats, (double)speedKmh, (double)courseDeg, unixTime);

    return transmit(pkt, pktLen);
}

// ── sendNodeInfo ──────────────────────────────────────────────────────────
bool LoRa::sendNodeInfo(uint32_t to, bool wantResponse, uint32_t requestId)
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    uint8_t userBuf[128] = {};
    const size_t userLen = _encodeUser(userBuf, sizeof(userBuf),
                                        Node.nodeId(),
                                        Node.longName(),
                                        Node.shortName());
    if (userLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_NODEINFO, userBuf, userLen,
                        wantResponse, to, requestId)) return false;

    ESP_LOGI(TAG, "TX NODEINFO id=%s long=\"%s\" short=\"%s\" hw=%u to=0x%08" PRIx32
             " req_id=0x%08" PRIx32,
             Node.nodeIdStr(), Node.longName(), Node.shortName(),
             (unsigned)HW_MODEL, to, requestId);

    return transmit(pkt, pktLen);
}

// ── _encodeTelemetry ──────────────────────────────────────────────────────
// Encode a Meshtastic Telemetry proto with Device Metrics.
//
// meshtastic/telemetry.proto:
//   Field 1 (time,           fixed32):  UTC Unix seconds — tag = 0x0D
//   Field 2 (device_metrics, message):  oneof variant — tag = 0x12
//     Inner DeviceMetrics:
//       Field 1 (battery_level, uint32): 0-100 %   — tag = 0x08
//       Field 2 (voltage,       float):  volts     — tag = 0x15 (fixed32)
//       Field 5 (uptime_seconds,uint32): varint    — tag = 0x28
//
// buf must be at least 32 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeTelemetry(uint8_t* buf, size_t cap,
                                            uint32_t unixTime, uint32_t uptimeSec,
                                            uint8_t batteryLevel, float batteryVoltage)
{
    return mc_encodeTelemetry(buf, cap, unixTime, uptimeSec, batteryLevel, batteryVoltage);
}

// ── sendTelemetry ─────────────────────────────────────────────────────────
bool LoRa::sendTelemetry()
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    const uint32_t now    = static_cast<uint32_t>(time(nullptr));
    const uint32_t uptime = static_cast<uint32_t>(
        xTaskGetTickCount() / (TickType_t)configTICK_RATE_HZ);

    const uint8_t batLevel   = Heltec.cachedBatteryLevel();
    const float   batVoltage = Heltec.cachedBatteryVoltage();

    uint8_t telBuf[40] = {};
    const size_t telLen = _encodeTelemetry(telBuf, sizeof(telBuf), now, uptime,
                                            batLevel, batVoltage);
    if (telLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_TELEMETRY, telBuf, telLen,
                        /*want_response=*/false)) return false;

    ESP_LOGI(TAG, "TX TELEMETRY uptime=%us bat=%u%% %.2fV time=%" PRIu32,
             (unsigned)uptime, batLevel, (double)batVoltage, now);
    return transmit(pkt, pktLen);
}

// ── _encodeMapReport ──────────────────────────────────────────────────────
// Encode a Meshtastic MapReport proto (MAP_REPORT_APP payload, port 73).
//
// MapReport announces this node to MQTT bridges so it appears on the public
// Meshtastic map (meshtastic.network/map).  Fields verified against
// Meshtastic firmware mesh.proto 2.5+:
//   Field  1 (long_name,          string):  tag 0x0A
//   Field  2 (short_name,         string):  tag 0x12
//   Field  3 (hw_model,           varint):  tag 0x18
//   Field  5 (region,             varint):  tag 0x28  RegionCode enum
//   Field  6 (modem_preset,       varint):  tag 0x30  ModemPreset enum
//   Field  7 (has_default_channel,bool):    tag 0x38  true when using factory PSK
//   Field  8 (latitude_i,         fixed32): tag 0x45
//   Field  9 (longitude_i,        fixed32): tag 0x4D
//   Field 10 (altitude,           varint):  tag 0x50
//   Field 11 (position_precision, varint):  tag 0x58  32 = full GPS precision
//   Field 12 (num_online_local_nodes,varint):tag 0x60 neighbour count
//
// buf must be at least 120 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeMapReport(uint8_t* buf, size_t cap,
                                            const char* longName,
                                            const char* shortName,
                                            int32_t lat_i, int32_t lon_i,
                                            int32_t alt_m,
                                            uint32_t numNeighbors)
{
#if   defined(CONFIG_LORA_PRESET_LONG_SLOW)
    static constexpr uint8_t MODEM_PRESET = 1;
#elif defined(CONFIG_LORA_PRESET_MEDIUM_SLOW)
    static constexpr uint8_t MODEM_PRESET = 3;
#elif defined(CONFIG_LORA_PRESET_SHORT_FAST)
    static constexpr uint8_t MODEM_PRESET = 6;
#else
    static constexpr uint8_t MODEM_PRESET = 0; // default LongFast
#endif
    return mc_encodeMapReport(buf, cap, longName, shortName,
                               lat_i, lon_i, alt_m, numNeighbors,
                               HW_MODEL,
                               static_cast<uint8_t>(CONFIG_LORA_REGION_CODE),
                               MODEM_PRESET);
}

// ── sendMapReport ─────────────────────────────────────────────────────────
bool LoRa::sendMapReport()
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif
#if CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC == 0
    return false; // disabled via Kconfig
#endif

    if (!gps.isFixed())
    {
        ESP_LOGD(TAG, "MAP_REPORT skipped — no GPS fix");
        return false;
    }

    const int32_t lat_i = static_cast<int32_t>(gps.lat() * 1e7);
    const int32_t lon_i = static_cast<int32_t>(gps.lng() * 1e7);
    const int32_t alt_m = static_cast<int32_t>(gps.altitude());

    uint8_t mrBuf[160] = {};
    const size_t mrLen = _encodeMapReport(mrBuf, sizeof(mrBuf),
                                           Node.longName(), Node.shortName(),
                                           lat_i, lon_i, alt_m,
                                           neighborCount());
    if (mrLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_MAP_REPORT, mrBuf, mrLen,
                        /*want_response=*/false)) return false;

    ESP_LOGI(TAG, "TX MAP_REPORT lat=%.6f lon=%.6f alt=%dm neighbors=%u role=%u",
             (double)lat_i / 1e7, (double)lon_i / 1e7, (int)alt_m,
             (unsigned)neighborCount(), (unsigned)CONFIG_MESH_NODE_ROLE);
    return transmit(pkt, pktLen);
}

// ─────────────────────────────────────────────────────────────────────────
// Neighbour table & RX decoder helpers
// ─────────────────────────────────────────────────────────────────────────

// ── _parsePosition / _parseUser — delegate to platform-free mc_ functions ─
/* static */ bool LoRa::_parsePosition(const uint8_t* data, size_t len, MeshPosition& pos)
{ return mc_parsePosition(data, len, pos); }

/* static */ bool LoRa::_parseUser(const uint8_t* data, size_t len, MeshUser& user)
{ return mc_parseUser(data, len, user); }

// ── _upsertNeighbor ───────────────────────────────────────────────────────
// Insert or update the neighbour table entry for fromNode.
// pos/user may be nullptr to leave that field unchanged on an existing entry.
void LoRa::_upsertNeighbor(uint32_t fromNode,
                             const MeshPosition* pos,
                             const MeshUser*     user)
{
    portENTER_CRITICAL(&_neighborLock);

    int slot = -1;
    TickType_t oldest = portMAX_DELAY;
    int oldestSlot = 0;

    for (int i = 0; i < (int)NEIGHBOR_MAX; i++)
    {
        if (_neighbors[i].occupied &&
            (_neighbors[i].pos.fromNode == fromNode ||
             _neighbors[i].user.fromNode == fromNode))
        {
            slot = i;
            break;
        }
        if (!_neighbors[i].occupied)
        {
            slot = i;
            break;
        }
        const TickType_t age = _neighbors[i].pos.lastSeen;
        if (age < oldest) { oldest = age; oldestSlot = i; }
    }
    if (slot == -1) slot = oldestSlot;

    // Clear stale data from a different node so their public key doesn't
    // leak into this entry (e.g. on eviction of an old neighbour).
    const bool isExisting = _neighbors[slot].occupied &&
        (_neighbors[slot].pos.fromNode == fromNode ||
         _neighbors[slot].user.fromNode == fromNode);
    if (!isExisting)
        _neighbors[slot] = {};

    _neighbors[slot].occupied = true;
    if (pos)
    {
        _neighbors[slot].pos           = *pos;
        _neighbors[slot].pos.fromNode  = fromNode;
        _neighbors[slot].pos.lastSeen  = xTaskGetTickCount();
        _neighbors[slot].pos.valid     = true;
    }
    else
    {
        _neighbors[slot].pos.fromNode = fromNode;
    }
    if (user)
    {
        _neighbors[slot].user          = *user;
        _neighbors[slot].user.fromNode = fromNode;
        _neighbors[slot].user.valid    = true;

        portEXIT_CRITICAL(&_neighborLock);
    }
    else
    {
        _neighbors[slot].user.fromNode = fromNode;
        portEXIT_CRITICAL(&_neighborLock);
    }
}

// ── neighborCount ─────────────────────────────────────────────────────────
size_t LoRa::neighborCount() const
{
    portENTER_CRITICAL(&_neighborLock);
    size_t n = 0;
    for (size_t i = 0; i < NEIGHBOR_MAX; i++)
        if (_neighbors[i].occupied) n++;
    portEXIT_CRITICAL(&_neighborLock);
    return n;
}

// ── neighborPosition ──────────────────────────────────────────────────────
MeshPosition LoRa::neighborPosition(size_t idx) const
{
    portENTER_CRITICAL(&_neighborLock);
    size_t count = 0;
    MeshPosition result = {};
    for (size_t i = 0; i < NEIGHBOR_MAX; i++)
    {
        if (_neighbors[i].occupied)
        {
            if (count == idx) { result = _neighbors[i].pos; break; }
            count++;
        }
    }
    portEXIT_CRITICAL(&_neighborLock);
    return result;
}

// ── neighborUser ──────────────────────────────────────────────────────────
MeshUser LoRa::neighborUser(size_t idx) const
{
    portENTER_CRITICAL(&_neighborLock);
    size_t count = 0;
    MeshUser result = {};
    for (size_t i = 0; i < NEIGHBOR_MAX; i++)
    {
        if (_neighbors[i].occupied)
        {
            if (count == idx) { result = _neighbors[i].user; break; }
            count++;
        }
    }
    portEXIT_CRITICAL(&_neighborLock);
    return result;
}

// ── _processPacket ────────────────────────────────────────────────────────
// Parse the raw LoRa payload, decrypt (AES-128-CTR channel, AES-256-CTR PKC,
// or plaintext in IsLicensed mode), decode the Data protobuf, and dispatch
// by portnum.
void LoRa::_processPacket(const uint8_t* buf, uint8_t pktLen,
                           int16_t rssi, float snr)
{
    if (pktLen < MESH_HDR + 1)
    {
        ESP_LOGD(TAG, "Packet too short (%u B), ignoring", pktLen);
        return;
    }

    const uint32_t to      = buf[0]  | (uint32_t)buf[1] <<  8
                           | (uint32_t)buf[2] << 16 | (uint32_t)buf[3] << 24;
    const uint32_t from    = buf[4]  | (uint32_t)buf[5] <<  8
                           | (uint32_t)buf[6] << 16 | (uint32_t)buf[7] << 24;
    const uint32_t pktId   = buf[8]  | (uint32_t)buf[9] <<  8
                           | (uint32_t)buf[10] << 16 | (uint32_t)buf[11] << 24;
    const uint8_t  hopLim  = buf[12] & 0x07;
    const uint8_t  chanHash= buf[13];

    // ── Deduplication ─────────────────────────────────────────────────────
    // Skip packets we have already processed (same packet ID).
    for (size_t i = 0; i < SEEN_IDS_MAX; i++)
    {
        if (_seenIds[i] == pktId && pktId != 0)
        {
            ESP_LOGD(TAG, "Duplicate pkt 0x%08" PRIx32 " — skipped", pktId);
            return;
        }
    }
    _seenIds[_seenCursor] = pktId;
    _seenCursor = (_seenCursor + 1) % SEEN_IDS_MAX;

    ESP_LOGI(TAG,
        "Rx pkt: to=0x%08" PRIx32 " from=0x%08" PRIx32 " id=0x%08" PRIx32
        " hop=%u ch=0x%02x len=%u rssi=%d snr=%.1f",
        to, from, pktId, hopLim, chanHash, pktLen, rssi, (double)snr);

    // ── Filter out our own transmissions ─────────────────────────────────
    if (from == Node.nodeId())
    {
        ESP_LOGI(TAG, "Ignoring own echo (hop=%u rssi=%d) — nearby node rebroadcast",
                 hopLim, rssi);
        return;
    }

    // ── Decrypt payload ───────────────────────────────────────────────────
    const uint8_t* ciphertext = buf + MESH_HDR;
    const size_t   cipherLen  = pktLen - MESH_HDR;
    uint8_t plain[256] = {};

#if CONFIG_LORA_IS_LICENSED
    bool decrypted = _decrypt(ciphertext, cipherLen, pktId, from, plain);
    if (!decrypted)
    {
        memcpy(plain, ciphertext, cipherLen);
        ESP_LOGI(TAG, "AES-CTR failed, treating as plaintext");
    }
#else
    bool decrypted = false;
    if (chanHash == 0x00)
    {
        // PKC (public-key cryptography) direct message — AES-256-CTR
        if (to != Node.nodeId())
        {
            ESP_LOGD(TAG, "PKC pkt not addressed to us (to=0x%08" PRIx32 ")", to);
            return;
        }

        bool hasKey = false;
        {
            portENTER_CRITICAL(&_neighborLock);
            for (size_t i = 0; i < NEIGHBOR_MAX; i++)
            {
                if (_neighbors[i].occupied &&
                    _neighbors[i].user.fromNode == from &&
                    _neighbors[i].user.hasPublicKey)
                {
                    hasKey = true;
                    break;
                }
            }
            portEXIT_CRITICAL(&_neighborLock);
        }

        if (!hasKey)
        {
            _bufferPkcPacket(buf, pktLen, rssi, snr);

            ESP_LOGI(TAG, "PKC from 0x%08" PRIx32
                     ": no public key — buffered, requesting NODEINFO", from);

#if CONFIG_LORA_TX_ENABLED
            const TickType_t now = xTaskGetTickCount();
            bool alreadyRequested = false;
            int oldestSlot = 0;
            TickType_t oldestAge = 0;

            for (size_t i = 0; i < PKC_REQ_RING_MAX; i++)
            {
                if (_pkcKeyReqs[i].nodeId == from &&
                    (now - _pkcKeyReqs[i].tick) < pdMS_TO_TICKS(PKC_REQ_COOLDOWN_MS))
                {
                    alreadyRequested = true;
                    break;
                }
                const TickType_t age = now - _pkcKeyReqs[i].tick;
                if (age > oldestAge) { oldestAge = age; oldestSlot = (int)i; }
            }

            if (!alreadyRequested)
            {
                _pkcKeyReqs[oldestSlot].nodeId = from;
                _pkcKeyReqs[oldestSlot].tick   = now;
                ESP_LOGI(TAG, "TX NODEINFO request to 0x%08" PRIx32
                         " (need PKC public key)", from);
                sendNodeInfo(from, /*wantResponse=*/true, /*requestId=*/0);
            }
            else
            {
                ESP_LOGD(TAG, "PKC key request for 0x%08" PRIx32
                         " already sent recently — waiting", from);
            }
#endif
            return;
        }

        decrypted = _decryptPkc(ciphertext, cipherLen, pktId, from, plain);
        if (!decrypted)
        {
            ESP_LOGW(TAG, "PKC decrypt failed for pkt 0x%08" PRIx32
                     " from 0x%08" PRIx32 " — ECDH or AES error",
                     pktId, from);
            return;
        }
        ESP_LOGI(TAG, "PKC DM decrypted from 0x%08" PRIx32, from);
    }
    else
    {
        decrypted = _decrypt(ciphertext, cipherLen, pktId, from, plain);
        if (!decrypted)
        {
            ESP_LOGW(TAG, "AES-CTR failed for pkt 0x%08" PRIx32, pktId);
            return;
        }
    }
#endif

    portENTER_CRITICAL(&_statsLock);
    _stats.decryptOk++;
    portEXIT_CRITICAL(&_statsLock);

    // Hex dump of decrypted Data proto for wire-level debugging.
    {
        char hex[120 * 3 + 1] = {};
        for (size_t i = 0; i < cipherLen && i < 120; i++)
            snprintf(hex + i * 3, 4, "%02x ", plain[i]);
        ESP_LOGI(TAG, "RX Data proto (%u bytes): %s", (unsigned)cipherLen, hex);
    }

    const size_t plainLen = cipherLen;
    uint32_t       portnum    = 0;
    const uint8_t* payload    = nullptr;
    size_t         payloadLen = 0;
    bool           wantResp   = false;

    if (!_parseData(plain, plainLen, portnum, payload, payloadLen, wantResp))
    {
        ESP_LOGI(TAG, "Parse failed for pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32 " ch=0x%02x",
                 pktId, from, chanHash);
        return;
    }
    if ((payload == nullptr || payloadLen == 0) && portnum != PORT_TRACEROUTE)
    {
        ESP_LOGI(TAG, "No payload in Data proto for pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32 " portnum=%u",
                 pktId, from, portnum);
        return;
    }

    ESP_LOGI(TAG, "Decoded pkt from 0x%08" PRIx32 " portnum=%u payloadLen=%u want_resp=%d",
             from, portnum, (unsigned)payloadLen, (int)wantResp);

    // ── Dispatch by portnum ───────────────────────────────────────────────

    if (portnum == PORT_TEXT)
    {
        const bool isDirect   = (to == Node.nodeId());
        const bool isBroadcast = (to == 0xFFFFFFFFu);
        if (!isDirect && !isBroadcast)
        {
            ESP_LOGI(TAG, "Text from 0x%08" PRIx32 " to 0x%08" PRIx32
                     " — not addressed to us, suppressing notification",
                     from, to);
            return;
        }

        MeshMessage msg;
        msg.fromNode = from;
        msg.rssi     = rssi;
        msg.snr      = snr;
        msg.valid    = true;
        const size_t copyLen = std::min(payloadLen, sizeof(msg.text) - 1);
        memcpy(msg.text, payload, copyLen);
        msg.text[copyLen] = '\0';

        ESP_LOGI(TAG, "Text from 0x%08" PRIx32 " (rssi=%d snr=%.1f) [%s]: %s",
                 from, rssi, (double)snr, isDirect ? "DM" : "CH", msg.text);

        portENTER_CRITICAL(&_statsLock);
        _stats.textMessages++;
        _stats.lastRssi = rssi;
        _stats.lastSnr  = snr;
        portEXIT_CRITICAL(&_statsLock);

        portENTER_CRITICAL(&_msgLock);
        _lastMsg = msg;
        portEXIT_CRITICAL(&_msgLock);

        Heltec.notifyDraw(Hardware::DRAW_LORA);
    }
    else if (portnum == PORT_POSITION)
    {
        MeshPosition pos = {};
        pos.rssi = rssi;
        pos.snr  = snr;

        if (_parsePosition(payload, payloadLen, pos))
        {
            ESP_LOGI(TAG,
                "Position from 0x%08" PRIx32
                " lat=%.5f lon=%.5f alt=%dm sats=%" PRIu32
                " rssi=%d snr=%.1f",
                from,
                (double)pos.lat_i / 1e7,
                (double)pos.lon_i / 1e7,
                (int)pos.alt_m, pos.sats,
                rssi, (double)snr);

            portENTER_CRITICAL(&_statsLock);
            _stats.lastRssi = rssi;
            _stats.lastSnr  = snr;
            portEXIT_CRITICAL(&_statsLock);

            _upsertNeighbor(from, &pos, nullptr);
            Heltec.notifyDraw(Hardware::DRAW_LORA_POS);
        }
        else
        {
            ESP_LOGD(TAG, "Position parse failed for pkt 0x08" PRIx32, pktId);
        }
    }
    else if (portnum == PORT_NODEINFO)
    {
        MeshUser user = {};
        user.rssi = rssi;
        user.snr  = snr;

        if (_parseUser(payload, payloadLen, user))
        {
            ESP_LOGI(TAG,
                "NodeInfo from 0x%08" PRIx32
                " id=%s long=\"%s\" short=\"%s\" hw=%" PRIu32
                " rssi=%d snr=%.1f want_resp=%d pubkey=%s",
                from,
                user.id, user.longName, user.shortName, user.hwModel,
                rssi, (double)snr, (int)wantResp,
                user.hasPublicKey ? "yes" : "no");

            portENTER_CRITICAL(&_statsLock);
            _stats.lastRssi = rssi;
            _stats.lastSnr  = snr;
            portEXIT_CRITICAL(&_statsLock);

            _upsertNeighbor(from, nullptr, &user);
            Heltec.notifyDraw(Hardware::DRAW_LORA_NODE);

#if !CONFIG_LORA_IS_LICENSED
            if (user.hasPublicKey)
                _retryPkcBuffer(from);
#endif

#if CONFIG_LORA_TX_ENABLED
            if (wantResp && (to == Node.nodeId() || to == 0xFFFFFFFF))
            {
                ESP_LOGI(TAG, "Responding to NodeInfo request from 0x%08" PRIx32
                         " (request_id=0x%08" PRIx32 ")", from, pktId);
                sendNodeInfo(from, /*wantResponse=*/false, /*requestId=*/pktId);
            }
#endif
        }
        else
        {
            ESP_LOGD(TAG, "NodeInfo parse failed for pkt 0x08" PRIx32, pktId);
        }
    }
    else if (portnum == PORT_TRACEROUTE && to == Node.nodeId())
    {
        // ── Traceroute destination response ───────────────────────────────
        //
        // Meshtastic TRACEROUTE_APP (portnum 70) protocol (firmware 2.3+):
        //
        // Forward phase: the initiating app sends a RouteDiscovery proto with
        // an empty `route` field to the destination node.  Each intermediate
        // router appends its own node ID (and SNR) to `route`/`snr_towards`.
        //
        // Destination phase (this code): we are the final `to` target.
        // We append our node ID + received SNR, then unicast the
        // RouteDiscovery back to `from` with request_id = incoming pktId.
        //
        // RouteDiscovery proto wire format (meshtastic/mesh.proto):
        //   Field 1 (route,        repeated fixed32):  tag 0x0D per element
        //   Field 2 (snr_towards,  repeated sint32):   tag 0x10 per element (zigzag)

        static constexpr size_t TRACEROUTE_MAX = 8;

        uint32_t route[TRACEROUTE_MAX] = {};
        int32_t  snrTowards[TRACEROUTE_MAX] = {};
        size_t   routeLen = 0;
        size_t   snrLen   = 0;

        if (payload != nullptr && payloadLen > 0)
        {
            const uint8_t* rp  = payload;
            const uint8_t* end = payload + payloadLen;
            while (rp < end)
            {
                uint8_t tagByte = *rp++;
                uint8_t field   = tagByte >> 3;
                uint8_t wt      = tagByte & 0x07;

                if (field == 1 && wt == 5) // route: fixed32
                {
                    if (rp + 4 > end) break;
                    uint32_t id = (uint32_t)rp[0]
                                | (uint32_t)rp[1] <<  8
                                | (uint32_t)rp[2] << 16
                                | (uint32_t)rp[3] << 24;
                    if (routeLen < TRACEROUTE_MAX) route[routeLen++] = id;
                    rp += 4;
                }
                else if (field == 2 && wt == 0) // snr_towards: zigzag sint32
                {
                    uint32_t zz = 0; int sh = 0;
                    while (rp < end) {
                        uint8_t b = *rp++;
                        zz |= (uint32_t)(b & 0x7F) << sh;
                        if (!(b & 0x80)) break;
                        sh += 7;
                    }
                    if (snrLen < TRACEROUTE_MAX) snrTowards[snrLen++] = (int32_t)zz;
                }
                else if (wt == 5) { if (rp + 4 <= end) rp += 4; else break; }
                else if (wt == 0) { while (rp < end && (*rp & 0x80)) rp++; if (rp < end) rp++; }
                else if (wt == 1) { if (rp + 8 <= end) rp += 8; else break; }
                else if (wt == 2)
                {
                    uint32_t l = 0; int sh = 0;
                    while (rp < end) {
                        uint8_t b = *rp++;
                        l |= (uint32_t)(b & 0x7F) << sh;
                        if (!(b & 0x80)) break;
                        sh += 7;
                    }
                    if (rp + l <= end) rp += l; else break;
                }
                else break;
            }
        }

        if (routeLen < TRACEROUTE_MAX)
            route[routeLen++] = Node.nodeId();

        const int32_t ourSnrRaw = static_cast<int32_t>(snr * 4.0f);
        const uint32_t ourSnrZz = mc_pbZigzag(ourSnrRaw);
        if (snrLen < TRACEROUTE_MAX)
            snrTowards[snrLen++] = static_cast<int32_t>(ourSnrZz);

        {
            char routeStr[12 * TRACEROUTE_MAX + 1] = {};
            size_t rs = 0;
            for (size_t i = 0; i < routeLen; i++)
                rs += snprintf(routeStr + rs, sizeof(routeStr) - rs,
                               "!%08" PRIx32 " ", route[i]);
            ESP_LOGI(TAG, "TRACEROUTE from 0x%08" PRIx32 " → route: %s(rssi=%d snr=%.1f)",
                     from, routeStr, rssi, (double)snr);
        }

        // Encode response RouteDiscovery
        uint8_t rdBuf[5 * TRACEROUTE_MAX + 6 * TRACEROUTE_MAX + 4] = {};
        size_t  rdLen = 0;

        for (size_t i = 0; i < routeLen; i++)
        {
            rdBuf[rdLen++] = 0x0D; // field 1, wire type 5 (fixed32)
            rdBuf[rdLen++] = static_cast<uint8_t>(route[i]);
            rdBuf[rdLen++] = static_cast<uint8_t>(route[i] >>  8);
            rdBuf[rdLen++] = static_cast<uint8_t>(route[i] >> 16);
            rdBuf[rdLen++] = static_cast<uint8_t>(route[i] >> 24);
        }
        for (size_t i = 0; i < snrLen; i++)
        {
            rdBuf[rdLen++] = 0x10; // field 2, wire type 0 (varint)
            rdLen += mc_pbVarint(rdBuf + rdLen, static_cast<uint32_t>(snrTowards[i]));
        }

#if CONFIG_LORA_TX_ENABLED
        {
            uint8_t pkt[256] = {};
            uint8_t pktLen2  = 0;
            if (_buildTxPacket(pkt, pktLen2, PORT_TRACEROUTE,
                               rdBuf, rdLen,
                               /*want_response=*/false,
                               /*to=*/from,
                               /*requestId=*/pktId))
            {
                ESP_LOGI(TAG, "TX TRACEROUTE reply to 0x%08" PRIx32
                         " req_id=0x%08" PRIx32 " route_hops=%u",
                         from, pktId, (unsigned)routeLen);
                transmit(pkt, pktLen2);
            }
            else
            {
                ESP_LOGW(TAG, "TRACEROUTE: _buildTxPacket failed");
            }
        }
#endif
    }
    else
    {
        ESP_LOGD(TAG, "Unhandled portnum %" PRIu32 " from 0x%08" PRIx32,
                 portnum, from);
    }
}

// ── lastMessage ───────────────────────────────────────────────────────────
MeshMessage LoRa::lastMessage() const
{
    portENTER_CRITICAL(&_msgLock);
    const MeshMessage copy = _lastMsg;
    portEXIT_CRITICAL(&_msgLock);
    return copy;
}

// ── stats ─────────────────────────────────────────────────────────────────
LoRaStats LoRa::stats() const
{
    portENTER_CRITICAL(&_statsLock);
    const LoRaStats copy = _stats;
    portEXIT_CRITICAL(&_statsLock);
    return copy;
}
