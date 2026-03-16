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
 * mesh_codec.h — Meshtastic over-the-air protobuf codec, zero platform deps.
 *
 * Contains:
 *  - Plain-old-data structs (MeshMessage, MeshPosition, MeshUser) — no FreeRTOS types.
 *  - Free encoder/decoder functions (mc_* prefix) for all Meshtastic proto messages.
 *
 * This header deliberately has NO ESP-IDF, FreeRTOS, or mbedTLS includes so that
 * its contents can be compiled and tested on a POSIX host without any firmware SDK.
 *
 * The LoRa class in lora.h uses these types and delegates its private static codec
 * methods to these free functions.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

// ── Meshtastic application-layer message structs ───────────────────────────
// All plain-old-data, safe to copy across tasks.
// NOTE: lastSeen is uint32_t (FreeRTOS ticks on the target, raw uint32 in tests).

struct MeshMessage {
    uint32_t fromNode = 0;   ///< sender node ID
    char     text[65] = {};  ///< UTF-8 text, null-terminated, max 64 chars
    int16_t  rssi     = 0;   ///< signal strength in dBm
    float    snr      = 0.f; ///< signal-to-noise ratio in dB
    bool     valid    = false;
};

struct MeshPosition {
    uint32_t fromNode  = 0;
    int32_t  lat_i     = 0;   ///< latitude  × 1e7 (degrees)
    int32_t  lon_i     = 0;   ///< longitude × 1e7 (degrees)
    int32_t  alt_m     = 0;   ///< altitude metres above MSL
    uint32_t sats      = 0;
    int16_t  rssi      = 0;
    float    snr       = 0.f;
    uint32_t unixTime  = 0;   ///< timestamp from Position proto (0 if absent)
    uint32_t lastSeen  = 0;   ///< xTaskGetTickCount() when last updated (uint32_t = TickType_t on ESP32)
    bool     valid     = false;
};

struct MeshUser {
    uint32_t fromNode     = 0;
    char     id[12]       = {}; ///< "!xxxxxxxx\0"
    char     longName[33] = {};
    char     shortName[5] = {};
    uint32_t hwModel      = 0;
    uint8_t  publicKey[32]= {}; ///< X25519 public key (field 8), for PKC decryption
    bool     hasPublicKey = false;
    int16_t  rssi         = 0;
    float    snr          = 0.f;
    bool     valid        = false;
};

// ── Protobuf encoder primitives ───────────────────────────────────────────

/// Encode a 64-bit unsigned value as a protobuf base-128 varint.
/// Returns the number of bytes written (1–10).
size_t   mc_pbVarint(uint8_t* buf, uint64_t val);

/// Zigzag-encode a signed 32-bit integer → unsigned (sint32 wire format).
/// Maps: 0→0, -1→1, 1→2, -2→3, 2→4 …
uint32_t mc_pbZigzag(int32_t val);

/// Write a length-delimited field (tag byte, length varint, raw bytes).
/// tag must be pre-computed as (fieldNum << 3) | 2.
size_t   mc_pbLenField(uint8_t* buf, uint8_t tag, const uint8_t* data, size_t dataLen);

/// Write a protobuf string field (length-delimited, from a null-terminated src).
size_t   mc_pbString(uint8_t* buf, uint8_t tag, const char* s);

// ── Meshtastic proto encoders ─────────────────────────────────────────────

/**
 * Encode a Meshtastic Position proto into buf.
 *
 * Field layout (verified against live OTA hex dumps, Meshtastic firmware 2.5+):
 *   Field  1 (latitude_i,    sfixed32): degrees × 1e7
 *   Field  2 (longitude_i,   sfixed32): degrees × 1e7
 *   Field  3 (altitude,      int32):    metres above MSL (varint)
 *   Field  4 (time,          sfixed32): UTC Unix seconds  ← NOT field 9 (pos_flags)
 *   Field  5 (location_source,varint):  PositionSource::GPS = 2
 *   Field 10 (ground_speed,   varint):  cm/s
 *   Field 11 (ground_track,   varint):  heading × 100 (0.01°)
 *   Field 14 (sats_in_view,   varint):  satellite count  ← NOT field 7 (google_plus_code)
 *   Field 18 (precision_bits, varint):  32 = full GPS precision
 *
 * buf must be at least 80 bytes.  Returns bytes written.
 */
size_t mc_encodePosition(uint8_t* buf, size_t cap,
                          int32_t lat_i, int32_t lon_i, int32_t alt_m,
                          uint32_t sats, uint32_t unixTime,
                          uint32_t speed_cm_s = 0, uint32_t track_x100 = 0);

/**
 * Encode a Meshtastic User (NodeInfo) proto into buf.
 *
 *   Field 1 (id,                string):  "!xxxxxxxx"
 *   Field 2 (long_name,         string):  up to 32 chars
 *   Field 3 (short_name,        string):  up to 4 chars
 *   Field 4 (macaddr,           bytes):   6-byte BT MAC
 *   Field 5 (hw_model,          varint):  HardwareModel enum
 *   Field 7 (role,              varint):  DeviceRole (omitted when 0 = CLIENT)
 *   Field 8 (public_key,        bytes):   32-byte X25519 key (nullptr = omit)
 *   Field 9 (is_unmessageable,  bool):    false
 *
 * buf must be at least 120 bytes.  Returns bytes written.
 */
size_t mc_encodeUser(uint8_t* buf, size_t cap,
                     uint32_t nodeId,
                     const char* longName, const char* shortName,
                     const uint8_t macaddr[6],
                     uint32_t hwModel,
                     uint8_t  deviceRole,      ///< 0 = CLIENT (field 7 omitted), 5 = TRACKER
                     const uint8_t* publicKey, ///< nullptr to omit field 8
                     bool isLicensed = false);  ///< true → emit is_licensed=true, skip pubkey

/**
 * Wrap an encoded payload in a Meshtastic Data proto.
 *
 *   Field 1 (portnum,       PortNum):  varint
 *   Field 2 (payload,       bytes):    LEN
 *   Field 3 (want_response, bool):     varint (omitted when false)
 *   Field 4 (dest,          fixed32):  omitted when 0
 *   Field 6 (request_id,    fixed32):  tag 0x35  ← NOT field 7 (reply_id, 0x3D)
 *   Field 9 (ok_to_mqtt,    bool):     false, emitted alongside request_id
 *
 * buf must be at least payloadLen + 32 bytes.  Returns bytes written.
 */
size_t mc_encodeData(uint8_t* buf, size_t cap,
                     uint32_t portnum,
                     const uint8_t* payload, size_t payloadLen,
                     bool want_response = false,
                     uint32_t dest      = 0,
                     uint32_t requestId = 0);

/**
 * Encode a Meshtastic Telemetry (Device Metrics) proto into buf.
 *
 *   Field 1 (time,           fixed32):  UTC Unix seconds
 *   Field 2 (device_metrics, message):
 *       Inner DeviceMetrics:
 *         Field 1 (battery_level,  uint32): 0-100 %
 *         Field 2 (voltage,        float):  volts
 *         Field 5 (uptime_seconds, uint32): varint
 *
 * buf must be at least 32 bytes.  Returns bytes written.
 */
size_t mc_encodeTelemetry(uint8_t* buf, size_t cap,
                           uint32_t unixTime, uint32_t uptimeSec,
                           uint8_t batteryLevel, float batteryVoltage);

/**
 * Encode a Meshtastic MapReport proto (MAP_REPORT_APP payload, port 73).
 *
 * buf must be at least 120 bytes.  Returns bytes written.
 */
size_t mc_encodeMapReport(uint8_t* buf, size_t cap,
                           const char* longName, const char* shortName,
                           int32_t lat_i, int32_t lon_i, int32_t alt_m,
                           uint32_t numNeighbors,
                           uint32_t hwModel,
                           uint8_t  regionCode,   ///< RegionCode enum: US=1, EU_868=3
                           uint8_t  modemPreset);  ///< ModemPreset: LONG_FAST=0, LONG_SLOW=1

// ── Meshtastic proto decoders ─────────────────────────────────────────────

/**
 * Decode a Meshtastic Data proto.
 *
 * Returns true when portnum was found.
 * payload pointer points INTO data — keep data alive while using payload.
 */
bool mc_parseData(const uint8_t* data, size_t len,
                  uint32_t&       portnum,
                  const uint8_t*& payload, size_t& payloadLen,
                  bool&           wantResponse);

/**
 * Decode a Meshtastic Position proto.
 *
 * Returns true when at least lat_i/lon_i were found.
 *
 * Correct field mapping (Meshtastic firmware 2.5+):
 *   time       → field 4 (sfixed32)  ← NOT field 9 (pos_flags varint)
 *   sats       → field 14 (varint)   ← NOT field 7 (google_plus_code string)
 */
bool mc_parsePosition(const uint8_t* data, size_t len, MeshPosition& pos);

/**
 * Decode a Meshtastic User (NodeInfo) proto.
 * Returns true when the id field (field 1) was found.
 */
bool mc_parseUser(const uint8_t* data, size_t len, MeshUser& user);
