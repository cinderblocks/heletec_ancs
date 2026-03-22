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
 * meshtastic_proto.cxx — Meshtastic 2.7.x protocol implementation.
 *
 * Protocol target: Meshtastic firmware 2.7.x (mesh.proto / telemetry.proto 2.7.15).
 *
 * Covers:
 *   - AES-128-CTR channel decryption and encryption (_decrypt)
 *   - AES-256-CCM X25519 PKC direct message decryption (_decryptPkc)
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
#include "mesh_crypto.h"
#include "gps.h"
#include "hardware.h"
#include "meshnode.h"

#include <esp_log.h>
#include <esp_random.h>
#include <mbedtls/platform_util.h>
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
// Meshtastic device).  Used for both RX decryption and TX encryption.
//
// Even in IsLicensed mode, TX packets are encrypted with this PSK so that
// stock Meshtastic nodes (matching chanHash=0x08) can decrypt and parse them.
// The PSK is publicly known (published in Meshtastic docs and source code),
// so using it does not violate ham-radio regulations — it only provides
// transport-layer framing that the mesh expects.
/* static */ const uint8_t LoRa::DEFAULT_PSK[16] = {
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};

// ── _decrypt ──────────────────────────────────────────────────────────────
// Meshtastic AES-128-CTR channel cipher — delegates to platform-free
// mc_channelCrypt() in mesh_crypto.cxx.
// CTR mode is symmetric: same function encrypts and decrypts.
bool LoRa::_decrypt(const uint8_t* in, size_t len,
                    uint32_t packetId, uint32_t fromNode,
                    uint8_t* out)
{
    return mc_channelCrypt(DEFAULT_PSK, packetId, fromNode, in, len, out);
}

// ── _decryptPkc ───────────────────────────────────────────────────────────
// PKC direct-message decryption — standard Meshtastic 2.5+ format.
//
// Wire layout: [ciphertext(n)] [8-byte CCM tag]
// Key:   SHA-256(X25519(ourPriv, remotePub))
// Nonce: [packetId LE(4)] [0x00(4)] [fromNode LE(4)] [0x00(4)], truncated to 13 bytes
// AAD:   none
bool LoRa::_decryptPkc(const uint8_t* in, size_t len,
                        const uint8_t* hdr, size_t hdrLen,
                        uint32_t packetId, uint32_t fromNode,
                        uint32_t toNode,
                        uint8_t* out, size_t& plainLen)
{
    plainLen = 0;
    (void)hdr; (void)hdrLen; (void)toNode;

    if (len <= MC_PKC_OVERHEAD) return false;
    if (!Node.hasPkcKeys()) return false;

    // ── Find sender's public key ──────────────────────────────────────────
    uint8_t remotePub[32] = {};
    {
        bool found = false;
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
        if (!found)
        {
            ESP_LOGD(TAG, "PKC: no public key for 0x%08" PRIx32, fromNode);
            return false;
        }
    }

    // ── Derive AES-256 key: SHA-256(X25519 shared secret) ────────────────
    uint8_t rawEcdh[32] = {}, aesKey[32] = {};
    if (!mc_x25519SharedSecret(Node.privateKey(), remotePub, rawEcdh))
    {
        mbedtls_platform_zeroize(remotePub, sizeof(remotePub));
        return false;
    }
    mc_sha256(rawEcdh, 32, aesKey);
    mbedtls_platform_zeroize(rawEcdh, sizeof(rawEcdh));
    mbedtls_platform_zeroize(remotePub, sizeof(remotePub));

    // ── AES-256-CCM decrypt, nonce[12]=0, no AAD ─────────────────────────
    uint8_t nonce[16] = {};
    memcpy(nonce,     &packetId, 4);
    memcpy(nonce + 8, &fromNode, 4);

    const bool ok = mc_pkcDecryptCcmFlex(aesKey, nonce, 13,
                                          in, len, out, MC_PKC_OVERHEAD);
    if (ok)
        plainLen = len - MC_PKC_OVERHEAD;

    mbedtls_platform_zeroize(aesKey, sizeof(aesKey));
    return ok;
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
        size_t  retryPlainLen = 0;

        if (!_decryptPkc(ciphertext, cipherLen, buf, MESH_HDR,
                         pktId, from, to, plain, retryPlainLen))
        {
            ESP_LOGW(TAG, "PKC: retry decrypt still failed for 0x%08" PRIx32, pktId);
            _pkcPending[i].occupied = false;
            continue;
        }

        ESP_LOGI(TAG, "PKC: retry decrypt SUCCESS for pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32 " (%u bytes plaintext)",
                 pktId, from, (unsigned)retryPlainLen);

        portENTER_CRITICAL(&_statsLock);
        _stats.decryptOk++;
        portEXIT_CRITICAL(&_statsLock);

        // Parse Data proto
        uint32_t       portnum    = 0;
        const uint8_t* payload    = nullptr;
        size_t         payloadLen = 0;
        bool           wantResp   = false;

        if (!_parseData(plain, retryPlainLen,
                        portnum, payload, payloadLen, wantResp))
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
// meshtastic/mesh.proto User message fields (Meshtastic 2.7.x):
//   Field 1 (id,               string):  "!xxxxxxxx"
//   Field 2 (long_name,        string):  up to 32 chars
//   Field 3 (short_name,       string):  up to 4 chars
//   Field 4 (macaddr,          bytes):   6-byte BT MAC
//   Field 5 (hw_model,         enum):    HardwareModel varint (tag 0x28)
//   Field 6 (is_licensed,      bool):    true in ham-radio (licensed) mode
//   Field 7 (role,             enum):    DeviceRole varint (tag 0x38);
//                                        2.7.x values: 0=CLIENT, 1=CLIENT_MUTE,
//                                        2=ROUTER, 3=ROUTER_CLIENT, 4=REPEATER,
//                                        5=TRACKER, 6=SENSOR, 7=TAK,
//                                        8=CLIENT_HIDDEN, 9=LOST_AND_FOUND,
//                                        10=TAK_TRACKER
//   Field 8 (public_key,       bytes):   32-byte X25519 public key (tag 0x42);
//                                        omitted in licensed mode or when unavailable
//   Field 9 (is_unmessageable, bool):    false — this device has a display and CAN
//                                        receive and show direct messages
//
// Note: isUnmessageable stays false even for TRACKER/SENSOR roles because this
// firmware has a TFT display and ANCS notification forwarding — messages ARE processed.
//
// buf must be at least 120 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeUser(uint8_t* buf, size_t cap,
                                       uint32_t nodeId,
                                       const char* longName,
                                       const char* shortName)
{
    const uint8_t* pubKey = Node.hasPkcKeys() ? Node.publicKey() : nullptr;
    return mc_encodeUser(buf, cap, nodeId, longName, shortName,
                         Node.macaddr(), HW_MODEL,
                         static_cast<uint8_t>(CONFIG_MESH_NODE_ROLE),
                         pubKey,
                         /*isLicensed=*/static_cast<bool>(CONFIG_LORA_IS_LICENSED),
                         /*isUnmessageable=*/false);
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
//
// Two modes controlled by CONFIG_LORA_IS_LICENSED:
//
// Encrypted mode (IS_LICENSED=n, default):
//   chanHash = 0x08 (DEFAULT_CHAN_HASH — encrypted LongFast PSK)
//   Payload  = AES-128-CTR(PSK, nonce, Data proto)
//   Stock Meshtastic nodes match chanHash=0x08, AES-CTR decrypt, and parse.
//
// Licensed mode (IS_LICENSED=y):
//   chanHash = 0x0A (UNENCRYPTED_CHAN_HASH — empty PSK LongFast)
//   Payload  = plaintext Data proto (no encryption)
//   Meshtastic protocol rule: is_licensed=true nodes MUST NOT encrypt their
//   transmissions (FCC Part 97 — operator identity must not be concealed).
//   Remote NodeDB::updateUser() validates that a packet carrying
//   is_licensed=true arrived on a plaintext channel hash; if it arrives on
//   chanHash=0x08 (encrypted) the firmware logs "is_licensed mismatch" and
//   may suppress the node from the peer's list.
//   Trade-off: only nodes also on the unencrypted channel (chanHash=0x0A)
//   will receive these packets; default encrypted nodes will not.
//
// Layout of out[]:
//   [0..15]  16-byte OTA header (plaintext)
//   [16..]   Data proto (encrypted in normal mode, plaintext in licensed mode)
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

    // For NODEINFO: dest goes in OTA header only, NOT in the Data proto.
    // Real Meshtastic 2.7.x never puts dest in NODEINFO Data protos.
    // For TEXT/DM sends: dest should be in the Data proto for routing.
    const bool includeDestInData = (unicast && portnum != PORT_NODEINFO);

    // 1. Encode Data proto.
    uint8_t dataBuf[244] = {};
    const size_t dataLen = _encodeData(dataBuf, sizeof(dataBuf),
                                        portnum, payload, payloadLen,
                                        want_response,
                                        includeDestInData ? to : 0,
                                        0,
                                        unicast ? requestId : 0);
    if (dataLen == 0 || dataLen > 239) return false;

    // Hex dump of plaintext Data proto for wire-level debugging.
    {
        char hex[120 * 3 + 1] = {};
        for (size_t i = 0; i < dataLen && i < 120; i++)
            snprintf(hex + i * 3, 4, "%02x ", dataBuf[i]);
        ESP_LOGD(TAG, "TX Data proto (%u bytes): %s", (unsigned)dataLen, hex);
    }

    // 2. Build 16-byte OTA header.
    //    chanHash selects which channel receiving nodes will match this packet
    //    against.  It must be consistent with whether the payload is encrypted.
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
#if CONFIG_LORA_IS_LICENSED
    out[13] = UNENCRYPTED_CHAN_HASH;  // 0x0A — plaintext LongFast channel
#else
    out[13] = DEFAULT_CHAN_HASH;      // 0x08 — encrypted LongFast channel
#endif
    out[14] = 0x00;
    out[15] = 0x00;

    // 3. Payload: plaintext (licensed mode) or AES-128-CTR encrypted (normal).
#if CONFIG_LORA_IS_LICENSED
    // Licensed mode — copy plaintext Data proto directly.
    // FCC Part 97 requires that licensed amateur operators not encrypt their
    // transmissions.  Meshtastic enforces this at the protocol layer:
    // NodeDB::updateUser() rejects NODEINFO packets where is_licensed=true
    // arrived on an encrypted channel hash.
    memcpy(out + MESH_HDR, dataBuf, dataLen);
#else
    // Encrypted mode — AES-128-CTR with the default PSK.
    // CTR mode is symmetric so _decrypt() serves as encrypt here.
    if (!_decrypt(dataBuf, dataLen, packetId, fromNode, out + MESH_HDR))
    {
        ESP_LOGW(TAG, "_buildTxPacket: AES-CTR encrypt failed");
        return false;
    }
#endif
    outLen = static_cast<uint8_t>(MESH_HDR + dataLen);

    ESP_LOGD(TAG,
        "OTA hdr: %02x%02x%02x%02x %02x%02x%02x%02x "
        "%02x%02x%02x%02x flags=%02x hash=%02x port=%u pktlen=%u",
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
    if (userLen == 0) {
        ESP_LOGW(TAG, "sendNodeInfo: _encodeUser returned 0 — nothing to send");
        return false;
    }

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_NODEINFO, userBuf, userLen,
                        wantResponse, to, requestId)) {
        ESP_LOGW(TAG, "sendNodeInfo: _buildTxPacket failed");
        return false;
    }

    const bool ok = transmit(pkt, pktLen);
    if (ok) {
        ESP_LOGI(TAG,
            "TX NODEINFO ok: id=%s long=\"%s\" short=\"%s\""
            " hw=%u role=%u licensed=%d pkc=%s"
            " to=0x%08" PRIx32 " want_resp=%d req_id=0x%08" PRIx32 " pktlen=%u",
            Node.nodeIdStr(), Node.longName(), Node.shortName(),
            (unsigned)HW_MODEL,
            (unsigned)CONFIG_MESH_NODE_ROLE,
            (int)CONFIG_LORA_IS_LICENSED,
            Node.hasPkcKeys() ? "yes" : "no",
            to, (int)wantResponse, requestId, (unsigned)pktLen);
    } else {
        ESP_LOGW(TAG,
            "TX NODEINFO FAILED: id=%s to=0x%08" PRIx32 " pktlen=%u",
            Node.nodeIdStr(), to, (unsigned)pktLen);
    }
    return ok;
}

// ── sendKeyVerification ───────────────────────────────────────────────────
// Unicast our X25519 public key to @p to via KEY_VERIFICATION_APP (port 77).
// Used two ways:
//   1. Reactively: when we receive a KEY_VERIFICATION_APP from a node that
//      has sent us their key and wantResponse=true, we reply with ours.
//   2. Proactively: when a peer sends a PKC DM we can't decrypt (their key
//      unknown), we can call this instead of a NodeInfo request to offer a
//      mutual exchange.
bool LoRa::sendKeyVerification(uint32_t to)
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif
#if CONFIG_LORA_IS_LICENSED
    // Licensed (ham) mode has no PKC keypair.
    return false;
#else
    if (!Node.hasPkcKeys()) return false;

    uint8_t pkiBuf[40] = {};
    const size_t pkiLen = _encodePkiReport(pkiBuf, sizeof(pkiBuf),
                                            Node.publicKey(), Node.nodeId());
    if (pkiLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_KEY_VERIFICATION, pkiBuf, pkiLen,
                        /*want_response=*/false, to, 0)) return false;

    ESP_LOGI(TAG, "TX KEY_VERIFICATION to 0x%08" PRIx32 " pktlen=%u",
             to, (unsigned)pktLen);
    return transmit(pkt, pktLen);
#endif
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
// Meshtastic firmware mesh.proto 2.7.x:
//   Field  1 (long_name,           string):  tag 0x0A
//   Field  2 (short_name,          string):  tag 0x12
//   Field  3 (hw_model,            varint):  tag 0x18
//   Field  4 (firmware_version,    string):  tag 0x22  NEW in 2.7.x
//   Field  5 (region,              varint):  tag 0x28  RegionCode enum
//   Field  6 (modem_preset,        varint):  tag 0x30  ModemPreset enum
//   Field  7 (has_default_channel, bool):    tag 0x38  true when using factory PSK
//   Field  8 (latitude_i,          fixed32): tag 0x45
//   Field  9 (longitude_i,         fixed32): tag 0x4D
//   Field 10 (altitude,            varint):  tag 0x50
//   Field 11 (position_precision,  varint):  tag 0x58  32 = full GPS precision
//   Field 12 (num_online_local_nodes,varint):tag 0x60 neighbour count
//
// buf must be at least 192 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeMapReport(uint8_t* buf, size_t cap,
                                            const char* longName,
                                            const char* shortName,
                                            int32_t lat_i, int32_t lon_i,
                                            int32_t alt_m,
                                            uint32_t numNeighbors,
                                            const char* firmwareVersion,
                                            bool hasPosition)
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
                               MODEM_PRESET,
                               firmwareVersion,
                               hasPosition);
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

    // When a GPS fix is available, include position fields (8–11) so the node
    // appears on the public Meshtastic map.  When no fix is available, send a
    // degraded report that still carries identity, firmware version, and region
    // so the node stays visible in node lists even without coordinates.
    const bool hasPos = gps.isFixed();
    int32_t lat_i = 0, lon_i = 0, alt_m = 0;
    if (hasPos) {
        lat_i = static_cast<int32_t>(gps.lat() * 1e7);
        lon_i = static_cast<int32_t>(gps.lng() * 1e7);
        alt_m = static_cast<int32_t>(gps.altitude());
    } else {
        ESP_LOGD(TAG, "MAP_REPORT: no GPS fix — sending identity-only report");
    }

    uint8_t mrBuf[192] = {};
    const size_t mrLen = _encodeMapReport(mrBuf, sizeof(mrBuf),
                                           Node.longName(), Node.shortName(),
                                           lat_i, lon_i, alt_m,
                                           neighborCount(),
                                           CONFIG_MESH_FIRMWARE_VERSION,
                                           hasPos);
    if (mrLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_MAP_REPORT, mrBuf, mrLen,
                        /*want_response=*/false)) return false;

    if (hasPos) {
        ESP_LOGI(TAG, "TX MAP_REPORT lat=%.6f lon=%.6f alt=%dm neighbors=%u fw=%s",
                 (double)lat_i / 1e7, (double)lon_i / 1e7, (int)alt_m,
                 (unsigned)neighborCount(), CONFIG_MESH_FIRMWARE_VERSION);
    } else {
        ESP_LOGI(TAG, "TX MAP_REPORT (no-pos) neighbors=%u fw=%s",
                 (unsigned)neighborCount(), CONFIG_MESH_FIRMWARE_VERSION);
    }
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
// pos / user / nodeStatus may each be nullptr to leave that field unchanged.
void LoRa::_upsertNeighbor(uint32_t fromNode,
                             const MeshPosition*   pos,
                             const MeshUser*       user,
                             const MeshNodeStatus* nodeStatus)
{
    portENTER_CRITICAL(&_neighborLock);

    int slot = -1;
    TickType_t oldest = portMAX_DELAY;
    int oldestSlot = 0;

    for (int i = 0; i < (int)NEIGHBOR_MAX; i++)
    {
        if (_neighbors[i].occupied &&
            (_neighbors[i].pos.fromNode == fromNode ||
             _neighbors[i].user.fromNode == fromNode ||
             _neighbors[i].nodeStatus.fromNode == fromNode))
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

    const bool isExisting = _neighbors[slot].occupied &&
        (_neighbors[slot].pos.fromNode == fromNode ||
         _neighbors[slot].user.fromNode == fromNode ||
         _neighbors[slot].nodeStatus.fromNode == fromNode);
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
    }
    else
    {
        _neighbors[slot].user.fromNode = fromNode;
    }

    if (nodeStatus)
    {
        _neighbors[slot].nodeStatus          = *nodeStatus;
        _neighbors[slot].nodeStatus.fromNode = fromNode;
        _neighbors[slot].nodeStatus.valid    = true;
    }
    else
    {
        _neighbors[slot].nodeStatus.fromNode = fromNode;
    }

    portEXIT_CRITICAL(&_neighborLock);
}

// ── _parseNodeStatus ──────────────────────────────────────────────────────
/* static */ bool LoRa::_parseNodeStatus(const uint8_t* data, size_t len,
                                          MeshNodeStatus& status)
{
    return mc_parseNodeStatus(data, len, status);
}

// ── _parsePkiReport ───────────────────────────────────────────────────────
/* static */ bool LoRa::_parsePkiReport(const uint8_t* data, size_t len,
                                         MeshPkiReport& report)
{
    return mc_parsePkiReport(data, len, report);
}

// ── _encodePkiReport ──────────────────────────────────────────────────────
/* static */ size_t LoRa::_encodePkiReport(uint8_t* buf, size_t cap,
                                             const uint8_t publicKey[32],
                                             uint32_t requestorNodeNum)
{
    return mc_encodePkiReport(buf, cap, publicKey, requestorNodeNum);
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

// ── neighborStatus ────────────────────────────────────────────────────────
MeshNodeStatus LoRa::neighborStatus(size_t idx) const
{
    portENTER_CRITICAL(&_neighborLock);
    size_t count = 0;
    MeshNodeStatus result = {};
    for (size_t i = 0; i < NEIGHBOR_MAX; i++)
    {
        if (_neighbors[i].occupied)
        {
            if (count == idx) { result = _neighbors[i].nodeStatus; break; }
            count++;
        }
    }
    portEXIT_CRITICAL(&_neighborLock);
    return result;
}

// ── _processPacket ────────────────────────────────────────────────────────
// Parse the raw LoRa payload, decrypt, decode the Data protobuf, and dispatch
// by portnum.
//
// Encrypted mode: AES-128-CTR for channel (chanHash == DEFAULT_CHAN_HASH),
//                 AES-256-CCM for PKC DMs (chanHash == 0x00).
// Licensed mode:  AES-128-CTR decrypt first; if the decrypted output fails
//                 protobuf parsing, fall back to raw plaintext (handles packets
//                 from other unencrypted nodes in a mixed mesh).
//                 PKC packets (chanHash == 0x00) are silently ignored since no
//                 keypair is available.
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

    ESP_LOGD(TAG,
        "Rx pkt: to=0x%08" PRIx32 " from=0x%08" PRIx32 " id=0x%08" PRIx32
        " hop=%u ch=0x%02x len=%u rssi=%d snr=%.1f",
        to, from, pktId, hopLim, chanHash, pktLen, rssi, (double)snr);

    // ── Filter out our own transmissions ─────────────────────────────────
    if (from == Node.nodeId())
    {
        ESP_LOGD(TAG, "Ignoring own echo (hop=%u rssi=%d) — nearby node rebroadcast",
                 hopLim, rssi);
        return;
    }

    // ── Decrypt payload ───────────────────────────────────────────────────
    const uint8_t* ciphertext = buf + MESH_HDR;
    const size_t   cipherLen  = pktLen - MESH_HDR;
    uint8_t plain[256] = {};
    size_t  plainLen   = cipherLen;

    if (chanHash == 0x00)
    {
        // ── PKC direct message (chanHash == 0x00) ─────────────────────────
        // AES-256-CCM encrypted with SHA-256(X25519 ECDH shared secret).
        // Only available in encrypted mode (requires a valid PKC keypair).
#if CONFIG_LORA_IS_LICENSED
        // Licensed mode has no PKC keypair — silently discard.
        ESP_LOGD(TAG, "PKC pkt 0x%08" PRIx32 " from 0x%08" PRIx32
                 " ignored — no PKC keys (licensed mode)", pktId, from);
        return;
#else
        if (!Node.hasPkcKeys())
        {
            ESP_LOGD(TAG, "PKC pkt 0x%08" PRIx32 " from 0x%08" PRIx32
                     " ignored — PKC keys unavailable", pktId, from);
            return;
        }

        size_t pkcPlainLen = 0;
        if (!_decryptPkc(ciphertext, cipherLen, buf, MESH_HDR,
                         pktId, from, to, plain, pkcPlainLen))
        {
            // Decryption failed — likely missing the sender's public key.
            // Buffer the packet so it can be retried once we learn their key
            // from a NODEINFO broadcast, then request their NodeInfo.
            _bufferPkcPacket(buf, pktLen, rssi, snr);

#if CONFIG_LORA_TX_ENABLED
            // Rate-limited NODEINFO request to obtain the sender's public key.
            const TickType_t now = xTaskGetTickCount();
            bool doReq = true;
            for (size_t ri = 0; ri < PKC_REQ_RING_MAX; ri++)
            {
                if (_pkcKeyReqs[ri].nodeId == from &&
                    (now - _pkcKeyReqs[ri].tick) < pdMS_TO_TICKS(PKC_REQ_COOLDOWN_MS))
                {
                    doReq = false;
                    break;
                }
            }
            if (doReq)
            {
                // Advance the rate-limiter ring cursor (class member, not static local)
                _pkcKeyReqs[_pkcReqCursor].nodeId = from;
                _pkcKeyReqs[_pkcReqCursor].tick   = now;
                _pkcReqCursor = (_pkcReqCursor + 1) % PKC_REQ_RING_MAX;

                ESP_LOGI(TAG, "PKC: no key for 0x%08" PRIx32
                         " — requesting NodeInfo", from);
                sendNodeInfo(from, /*wantResponse=*/true);
            }
            else
            {
                ESP_LOGD(TAG, "PKC: key request for 0x%08" PRIx32
                         " on cooldown", from);
            }
#endif
            return;
        }
        plainLen = pkcPlainLen;

        portENTER_CRITICAL(&_statsLock);
        _stats.decryptOk++;
        portEXIT_CRITICAL(&_statsLock);
#endif // !CONFIG_LORA_IS_LICENSED
    }
    else
    {
        // ── Channel payload — encrypted (0x08) or plaintext (0x0A) ──────
        // chanHash=0x08: default encrypted LongFast PSK.
        // chanHash=0x0A: unencrypted LongFast (licensed / empty-PSK nodes).
        // Any other value: unknown channel — try AES-CTR, fall back to plain.
        //
        // Strategy: if the chanHash matches our own TX mode, take the fast
        // path.  Otherwise try AES-CTR first and fall back to plaintext so
        // we can hear both encrypted and unencrypted nodes regardless of our
        // own mode.
#if CONFIG_LORA_IS_LICENSED
        if (chanHash == UNENCRYPTED_CHAN_HASH)
        {
            // Packet arrived on the plaintext channel — use as-is.
            memcpy(plain, ciphertext, cipherLen);
        }
        else
        {
            // Encrypted packet received while in licensed mode.
            // Try AES-CTR (handles default encrypted Meshtastic neighbours);
            // fall back to raw plaintext if it doesn't parse.
            if (!_decrypt(ciphertext, cipherLen, pktId, from, plain))
                return; // should never happen

            uint32_t       tmpPortnum    = 0;
            const uint8_t* tmpPayload    = nullptr;
            size_t         tmpPayloadLen = 0;
            bool           tmpWantResp   = false;
            if (!mc_parseData(plain, cipherLen, tmpPortnum, tmpPayload,
                              tmpPayloadLen, tmpWantResp) || tmpPortnum == 0)
                memcpy(plain, ciphertext, cipherLen);
        }
#else
        // Normal encrypted mode: AES-CTR decrypt, fall back to plaintext
        // for unencrypted neighbours (chanHash=0x0A licensed nodes).
        if (!_decrypt(ciphertext, cipherLen, pktId, from, plain))
            return; // should never happen

        {
            uint32_t       tmpPortnum    = 0;
            const uint8_t* tmpPayload    = nullptr;
            size_t         tmpPayloadLen = 0;
            bool           tmpWantResp   = false;
            if (!mc_parseData(plain, cipherLen, tmpPortnum, tmpPayload,
                              tmpPayloadLen, tmpWantResp) || tmpPortnum == 0)
            {
                // AES-CTR output didn't parse — fall back to raw plaintext
                // (handles licensed / unencrypted-channel neighbours).
                memcpy(plain, ciphertext, cipherLen);
            }
        }
#endif

        portENTER_CRITICAL(&_statsLock);
        _stats.decryptOk++;
        portEXIT_CRITICAL(&_statsLock);
    }

    uint32_t       portnum    = 0;
    const uint8_t* payload    = nullptr;
    size_t         payloadLen = 0;
    bool           wantResp   = false;

    if (!_parseData(plain, plainLen, portnum, payload, payloadLen, wantResp))
    {
        ESP_LOGD(TAG, "Parse failed for pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32 " ch=0x%02x",
                 pktId, from, chanHash);
        return;
    }
    if ((payload == nullptr || payloadLen == 0) && portnum != PORT_TRACEROUTE)
    {
        ESP_LOGD(TAG, "No payload in Data proto for pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32 " portnum=%u",
                 pktId, from, portnum);
        return;
    }

    ESP_LOGD(TAG, "Decoded pkt from 0x%08" PRIx32 " portnum=%u payloadLen=%u want_resp=%d",
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

        // Populate sender short name from the neighbour table so the TFT can
        // display it instead of a raw hex node ID.
        for (size_t ni = 0; ni < NEIGHBOR_MAX; ni++)
        {
            portENTER_CRITICAL(&_neighborLock);
            const bool occ  = _neighbors[ni].occupied;
            const bool match = occ && (_neighbors[ni].user.fromNode == from ||
                                       _neighbors[ni].pos.fromNode  == from);
            char sn[5] = {};
            if (match) memcpy(sn, _neighbors[ni].user.shortName, sizeof(sn));
            portEXIT_CRITICAL(&_neighborLock);
            if (match && sn[0] != '\0')
            {
                strncpy(msg.shortName, sn, sizeof(msg.shortName) - 1);
                break;
            }
        }

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
                " spd=%" PRIu32 "cm/s hdg=%.2f"
                " rssi=%d snr=%.1f",
                from,
                (double)pos.lat_i / 1e7,
                (double)pos.lon_i / 1e7,
                (int)pos.alt_m, pos.sats,
                pos.speed_cm_s, (double)pos.track_x100 / 100.0,
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
                " role=%u licensed=%d unmessageable=%d"
                " rssi=%d snr=%.1f want_resp=%d pubkey=%s",
                from,
                user.id, user.longName, user.shortName, user.hwModel,
                (unsigned)user.role, (int)user.isLicensed,
                (int)user.isUnmessageable,
                rssi, (double)snr, (int)wantResp,
                user.hasPublicKey ? "yes" : "no");

            portENTER_CRITICAL(&_statsLock);
            _stats.lastRssi = rssi;
            _stats.lastSnr  = snr;
            portEXIT_CRITICAL(&_statsLock);

            _upsertNeighbor(from, nullptr, &user);
            Heltec.notifyDraw(Hardware::DRAW_LORA_NODE);

            // Retry any PKC packets buffered from this node now that we have
            // their public key (hasPublicKey == true).  No-op if the buffer is
            // empty or the key is absent (licensed node).
            if (user.hasPublicKey)
                _retryPkcBuffer(from);

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
    else if (portnum == PORT_NODE_STATUS)
    {
        // ── NODE_STATUS_APP (portnum 36, Meshtastic 2.7.x) ───────────────
        // Node status string — broadcasts on change and on a periodic timer
        // (approximately once a day).  Payload is a NodeStatus protobuf.
        // Update the neighbour table so callers can see whether a nearby
        // node is an MQTT gateway or acting as a router.
        MeshNodeStatus ns = {};
        ns.rssi = rssi;
        ns.snr  = snr;

        if (_parseNodeStatus(payload, payloadLen, ns))
        {
            ESP_LOGI(TAG,
                "NodeStatus from 0x%08" PRIx32
                " uptime=%" PRIu32 "s mqtt=%d router=%d"
                " rssi=%d snr=%.1f",
                from,
                ns.uptimeSec,
                (int)ns.isMqttConnected,
                (int)ns.isRouter,
                rssi, (double)snr);

            portENTER_CRITICAL(&_statsLock);
            _stats.lastRssi = rssi;
            _stats.lastSnr  = snr;
            portEXIT_CRITICAL(&_statsLock);

            _upsertNeighbor(from, nullptr, nullptr, &ns);
        }
        else
        {
            ESP_LOGD(TAG, "NodeStatus parse failed for pkt 0x%08" PRIx32, pktId);
        }
    }
    else if (portnum == PORT_ALERT)
    {
        // ── ALERT_APP (portnum 11, Meshtastic 2.7.15) ────────────────────
        // Critical alert broadcast.  Payload is raw UTF-8 text (same wire
        // format as TEXT_MESSAGE_APP).  Alerts are ALWAYS shown regardless
        // of the `to` field — they are inherently broadcast-only.
        MeshMessage msg;
        msg.fromNode = from;
        msg.rssi     = rssi;
        msg.snr      = snr;
        msg.isAlert  = true;
        msg.valid    = true;
        const size_t copyLen = std::min(payloadLen, sizeof(msg.text) - 1);
        memcpy(msg.text, payload, copyLen);
        msg.text[copyLen] = '\0';

        // Populate sender short name from neighbour table (same as TEXT).
        for (size_t ni = 0; ni < NEIGHBOR_MAX; ni++)
        {
            portENTER_CRITICAL(&_neighborLock);
            const bool occ   = _neighbors[ni].occupied;
            const bool match = occ && (_neighbors[ni].user.fromNode == from ||
                                       _neighbors[ni].pos.fromNode  == from);
            char sn[5] = {};
            if (match) memcpy(sn, _neighbors[ni].user.shortName, sizeof(sn));
            portEXIT_CRITICAL(&_neighborLock);
            if (match && sn[0] != '\0')
            {
                strncpy(msg.shortName, sn, sizeof(msg.shortName) - 1);
                break;
            }
        }

        ESP_LOGW(TAG, "ALERT from 0x%08" PRIx32 " (rssi=%d snr=%.1f): %s",
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
    else if (portnum == PORT_KEY_VERIFICATION)
    {
        // ── KEY_VERIFICATION_APP (portnum 12, Meshtastic 2.7.15) ─────────
        // Public key exchange for PKC direct messages.
        //
        // The sender is sharing their 32-byte X25519 public key (PKIReport
        // field 1) so we can encrypt PKC DMs to them.  Store it in the
        // neighbour table and retry any buffered packets from this node.
        //
        // If wantResponse is set the sender is requesting our key in return.
        MeshPkiReport report = {};
        if (_parsePkiReport(payload, payloadLen, report))
        {
            ESP_LOGI(TAG,
                "KeyVerification from 0x%08" PRIx32
                " requestor=0x%08" PRIx32
                " pubkey=%s rssi=%d snr=%.1f",
                from, report.requestorNodeNum,
                report.hasPublicKey ? "yes" : "no",
                rssi, (double)snr);

            if (report.hasPublicKey)
            {
                // Merge the key into the neighbour's user record.
                // Create a minimal MeshUser carrying only the public key so
                // _upsertNeighbor does not clobber existing identity fields.
                MeshUser keyUpdate = {};
                keyUpdate.fromNode     = from;
                keyUpdate.hasPublicKey = true;
                memcpy(keyUpdate.publicKey, report.publicKey, 32);

                _upsertNeighbor(from, nullptr, &keyUpdate);

                // Retry any PKC DMs that arrived before we had the key.
                _retryPkcBuffer(from);
            }

#if CONFIG_LORA_TX_ENABLED
            // Reply with our own key if requested.
            if (wantResp)
            {
                ESP_LOGI(TAG, "KeyVerification: responding with our pubkey to 0x%08" PRIx32, from);
                sendKeyVerification(from);
            }
#endif
        }
        else
        {
            ESP_LOGD(TAG, "KeyVerification parse failed for pkt 0x%08" PRIx32, pktId);
        }
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
