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
 * mesh_codec.h — Meshtastic 2.7.x over-the-air protobuf codec, zero platform deps.
 *
 * Protocol target: Meshtastic firmware 2.7.x (mesh.proto / telemetry.proto 2.7.15).
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
    uint32_t fromNode   = 0;
    char     text[65]   = {};  ///< UTF-8 text, null-terminated, max 64 chars
    char     shortName[5] = {}; ///< sender short name from neighbour table; "" if unknown
    int16_t  rssi       = 0;
    float    snr        = 0.f;
    bool     isAlert    = false; ///< true when received on PORT_ALERT (portnum 76)
    bool     valid      = false;
};

struct MeshPosition {
    uint32_t fromNode    = 0;
    int32_t  lat_i       = 0;   ///< latitude  × 1e7 (degrees)
    int32_t  lon_i       = 0;   ///< longitude × 1e7 (degrees)
    int32_t  alt_m       = 0;   ///< altitude metres above MSL
    uint32_t sats        = 0;
    uint32_t speed_cm_s  = 0;   ///< ground speed in cm/s (field 10)
    uint32_t track_x100  = 0;   ///< ground track (heading) × 100 in 0.01° (field 11)
    int16_t  rssi        = 0;
    float    snr         = 0.f;
    uint32_t unixTime    = 0;   ///< timestamp from Position proto (0 if absent)
    uint32_t lastSeen    = 0;   ///< xTaskGetTickCount() when last updated (uint32_t = TickType_t on ESP32)
    bool     valid       = false;
};

struct MeshUser {
    uint32_t fromNode       = 0;
    char     id[12]         = {}; ///< "!xxxxxxxx\0"
    char     longName[33]   = {};
    char     shortName[5]   = {};
    uint8_t  macaddr[6]     = {}; ///< BT MAC address (field 4)
    bool     hasMacaddr     = false;
    uint32_t hwModel        = 0;
    bool     isLicensed     = false; ///< User proto field 6
    uint8_t  role           = 0;     ///< DeviceRole enum (field 7): 0=CLIENT, 5=TRACKER
    uint8_t  publicKey[32]  = {};    ///< X25519 public key (field 8), for PKC decryption
    bool     hasPublicKey   = false;
    bool     isUnmessageable= false; ///< User proto field 9
    int16_t  rssi           = 0;
    float    snr            = 0.f;
    bool     valid          = false;
};

/**
 * NodeStatus — decoded from NODE_STATUS_APP (portnum 75, Meshtastic 2.7.x).
 *
 * Lightweight heartbeat broadcast that nodes send periodically to indicate
 * they are online and to share basic connectivity state.  Distinct from
 * TELEMETRY_APP (battery/voltage metrics) and NODEINFO_APP (identity).
 *
 * Proto: meshtastic/mesh.proto, message NodeStatus (2.7.x):
 *   Field 1 (uptime,            uint32): seconds since last reboot  — tag 0x08
 *   Field 2 (is_mqtt_connected, bool):  connected to MQTT broker    — tag 0x10
 *   Field 3 (is_router,         bool):  acting as mesh router       — tag 0x18
 */
struct MeshNodeStatus {
    uint32_t fromNode        = 0;
    uint32_t uptimeSec       = 0;    ///< seconds since last reboot (field 1)
    bool     isMqttConnected = false;///< connected to MQTT broker (field 2)
    bool     isRouter        = false;///< acting as mesh router (field 3)
    int16_t  rssi            = 0;
    float    snr             = 0.f;
    bool     valid           = false;
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
 * Field layout (verified against Meshtastic 2.7.x mesh.proto + live OTA captures):
 *   Field  1 (latitude_i,    sfixed32): degrees × 1e7
 *   Field  2 (longitude_i,   sfixed32): degrees × 1e7
 *   Field  3 (altitude,      int32):    metres above MSL (varint); omit when 0
 *   Field  4 (time,          fixed32):  GPS fix epoch (UTC seconds); omit when 0
 *   Field  5 (location_source,varint):  PositionSource::GPS = 2
 *   Field  7 (timestamp,     fixed32):  Device wall-clock time (UTC seconds) — NEW in 2.7.x.
 *                                       MQTT bridges use this for "last seen" display.
 *                                       Emitted alongside field 4 when unixTime != 0.
 *                                       Omit when 0.
 *   Field 10 (ground_speed,   varint):  cm/s; omit when 0
 *   Field 11 (ground_track,   varint):  heading × 100 (0.01°); omit when 0
 *   Field 14 (sats_in_view,   varint):  satellite count; omit when 0
 *   Field 18 (precision_bits, varint):  32 = full GPS precision (always emitted)
 *
 * buf must be at least 90 bytes.  Returns bytes written.
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
 *   Field 6 (is_licensed,       bool):    true when operating under ham licence
 *   Field 7 (role,              varint):  DeviceRole (omitted when 0 = CLIENT)
 *   Field 8 (public_key,        bytes):   32-byte X25519 key (nullptr = omit)
 *   Field 9 (is_unmessageable,  bool):    true when node cannot receive DMs
 *                                         (e.g. headless sensors/trackers without
 *                                          a display or DM-processing firmware)
 *
 * isLicensed=true → emit field 6 and suppress field 8 (no PKC in licensed mode).
 * isUnmessageable=false → emit field 9 = 0 (always required by Meshtastic 2.5+).
 *
 * buf must be at least 120 bytes.  Returns bytes written.
 */
size_t mc_encodeUser(uint8_t* buf, size_t cap,
                     uint32_t nodeId,
                     const char* longName, const char* shortName,
                     const uint8_t macaddr[6],
                     uint32_t hwModel,
                     uint8_t  deviceRole,         ///< DeviceRole varint (2.7.x: 0=CLIENT … 10=TAK_TRACKER); field 7 omitted when 0
                     const uint8_t* publicKey,    ///< nullptr to omit field 8
                     bool isLicensed        = false, ///< true → emit is_licensed=true (field 6), skip pubkey
                     bool isUnmessageable   = false); ///< true → emit is_unmessageable=true (field 9)

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
 *       Inner DeviceMetrics (Meshtastic 2.7.x):
 *         Field 1 (battery_level,       uint32): 0-100 %
 *         Field 2 (voltage,             float):  volts
 *         Field 3 (channel_utilization, float):  0.0 — no airtime tracking
 *         Field 4 (air_util_tx,         float):  0.0 — no airtime tracking
 *         Field 5 (uptime_seconds,      uint32): seconds
 *       Fields 3 and 4 are always emitted as 0.0f; real Meshtastic firmware
 *       always sends them so the app shows "0%" rather than "N/A".
 *
 * buf must be at least 48 bytes.  Returns bytes written.
 */
size_t mc_encodeTelemetry(uint8_t* buf, size_t cap,
                           uint32_t unixTime, uint32_t uptimeSec,
                           uint8_t batteryLevel, float batteryVoltage);

/**
 * Encode a Meshtastic MapReport proto (MAP_REPORT_APP payload, port 73).
 *
 * Field layout (verified against Meshtastic firmware 2.7.x mesh.proto):
 *   Field  1 (long_name,           string):  tag 0x0A
 *   Field  2 (short_name,          string):  tag 0x12
 *   Field  3 (hw_model,            varint):  tag 0x18  HardwareModel enum
 *   Field  4 (firmware_version,    string):  tag 0x22  NEW in 2.7.x; omit when nullptr
 *   Field  5 (region,              varint):  tag 0x28  RegionCode enum
 *   Field  6 (modem_preset,        varint):  tag 0x30  ModemPreset enum
 *   Field  7 (has_default_channel, bool):    tag 0x38  true = factory default PSK
 *   Field  8 (latitude_i,          sfixed32):tag 0x45
 *   Field  9 (longitude_i,         sfixed32):tag 0x4D
 *   Field 10 (altitude,            varint):  tag 0x50  omit when 0
 *   Field 11 (position_precision,  varint):  tag 0x58  32 = full GPS precision
 *   Field 12 (num_online_local_nodes,varint):tag 0x60  omit when 0
 *
 * buf must be at least 160 bytes.  Returns bytes written.
 */
size_t mc_encodeMapReport(uint8_t* buf, size_t cap,
                           const char* longName, const char* shortName,
                           int32_t lat_i, int32_t lon_i, int32_t alt_m,
                           uint32_t numNeighbors,
                           uint32_t hwModel,
                           uint8_t  regionCode,         ///< RegionCode: US=1, EU_868=3
                           uint8_t  modemPreset,        ///< ModemPreset: LONG_FAST=0
                           const char* firmwareVersion = nullptr); ///< 2.7.x field 4; nullptr = omit

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
 * Decoded fields (Meshtastic 2.7.x mesh.proto):
 *   lat_i       ← field 1  (sfixed32)
 *   lon_i       ← field 2  (sfixed32)
 *   alt_m       ← field 3  (int32 varint)
 *   unixTime    ← field 4  (fixed32) GPS fix time — highest priority
 *               ← field 7  (fixed32) device timestamp — fallback when field 4 absent
 *                 (field 7 = "timestamp", new in 2.7.x; pre-2.7.x had google_plus_code
 *                  string here which is safely ignored as a different wire type)
 *   speed_cm_s  ← field 10 (uint32 varint, ground_speed in cm/s)
 *   track_x100  ← field 11 (uint32 varint, ground_track heading × 100)
 *   sats        ← field 14 (uint32 varint) ← NOT field 7 (google_plus_code in pre-2.7.x)
 */
bool mc_parsePosition(const uint8_t* data, size_t len, MeshPosition& pos);

/**
 * Decode a Meshtastic User (NodeInfo) proto.
 * Returns true when the id field (field 1) was found.
 *
 * Decoded fields (Meshtastic 2.7.x mesh.proto):
 *   id              ← field 1  (string)
 *   longName        ← field 2  (string)
 *   shortName       ← field 3  (string)
 *   macaddr         ← field 4  (bytes, 6)  — sets hasMacaddr when length==6
 *   hwModel         ← field 5  (varint)    HardwareModel enum
 *   isLicensed      ← field 6  (bool varint)
 *   role            ← field 7  (varint)    DeviceRole enum (2.7.x values 0–10):
 *                                          0=CLIENT 1=CLIENT_MUTE 2=ROUTER
 *                                          3=ROUTER_CLIENT 4=REPEATER 5=TRACKER
 *                                          6=SENSOR 7=TAK 8=CLIENT_HIDDEN
 *                                          9=LOST_AND_FOUND 10=TAK_TRACKER
 *   publicKey       ← field 8  (bytes, 32) — sets hasPublicKey when length==32
 *   isUnmessageable ← field 9  (bool varint)
 */
bool mc_parseUser(const uint8_t* data, size_t len, MeshUser& user);

/**
 * Decode a Meshtastic NodeStatus proto (NODE_STATUS_APP payload, portnum 75).
 * Returns true when at least the uptime field (field 1) was present.
 *
 * Decoded fields (Meshtastic 2.7.x mesh.proto):
 *   uptimeSec       ← field 1 (uint32 varint)  seconds since last reboot
 *   isMqttConnected ← field 2 (bool varint)    connected to MQTT broker
 *   isRouter        ← field 3 (bool varint)    acting as mesh router
 *
 * All fields are optional (proto3 defaults to 0/false).  Returns true even
 * when only field 1 is present, which is the normal case for non-gateway nodes.
 */
bool mc_parseNodeStatus(const uint8_t* data, size_t len, MeshNodeStatus& status);

/**
 * Encode / decode a Meshtastic PKIReport proto (KEY_VERIFICATION_APP payload,
 * portnum 77, Meshtastic 2.7.x).
 *
 * Nodes broadcast their X25519 public key via KEY_VERIFICATION_APP so that
 * peers can perform authenticated PKC DM encryption.  This is the explicit
 * key-exchange complement to the implicit key distribution via NODEINFO_APP
 * (field 8).
 *
 * Proto: meshtastic/mesh.proto, message PKIReport (2.7.x):
 *   Field 1 (public_key,         bytes):  32-byte X25519 public key — tag 0x0A
 *   Field 2 (requestor_node_num, uint32): requesting node num        — tag 0x10
 */
struct MeshPkiReport {
    uint8_t  publicKey[32]    = {}; ///< sender's X25519 public key (field 1)
    bool     hasPublicKey     = false;
    uint32_t requestorNodeNum = 0;  ///< node that requested the verification (field 2)
    bool     valid            = false; ///< true when field 1 was present and 32 bytes
};

/**
 * Decode a PKIReport proto payload into report.
 * Returns true when field 1 (public_key, 32 bytes) was found.
 */
bool mc_parsePkiReport(const uint8_t* data, size_t len, MeshPkiReport& report);

/**
 * Encode a PKIReport proto into buf.
 *   Field 1 (public_key,         bytes):  publicKey[32]     — always emitted
 *   Field 2 (requestor_node_num, uint32): requestorNodeNum  — omitted when 0
 * buf must be at least 38 bytes.  Returns bytes written.
 */
size_t mc_encodePkiReport(uint8_t* buf, size_t cap,
                           const uint8_t publicKey[32],
                           uint32_t requestorNodeNum = 0);
