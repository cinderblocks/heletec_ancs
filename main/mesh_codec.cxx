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
 * mesh_codec.cxx — Meshtastic protobuf encoder/decoder implementations.
 *
 * No ESP-IDF, FreeRTOS, or mbedTLS headers included — deliberately kept
 * platform-free so this file can be compiled and tested on the host.
 */

#include "mesh_codec.h"
#include <cinttypes>
#include <cstdio>
#include <algorithm>

// ── Encoder primitives ────────────────────────────────────────────────────

size_t mc_pbVarint(uint8_t* buf, uint64_t val)
{
    size_t n = 0;
    while (val > 0x7F)
    {
        buf[n++] = static_cast<uint8_t>((val & 0x7F) | 0x80);
        val >>= 7;
    }
    buf[n++] = static_cast<uint8_t>(val & 0x7F);
    return n;
}

uint32_t mc_pbZigzag(int32_t val)
{
    return (static_cast<uint32_t>(val) << 1) ^ static_cast<uint32_t>(val >> 31);
}

size_t mc_pbLenField(uint8_t* buf, uint8_t tag,
                     const uint8_t* data, size_t dataLen)
{
    size_t n = 0;
    buf[n++] = tag;
    n += mc_pbVarint(buf + n, dataLen);
    memcpy(buf + n, data, dataLen);
    n += dataLen;
    return n;
}

size_t mc_pbString(uint8_t* buf, uint8_t tag, const char* s)
{
    const size_t sLen = (s != nullptr) ? strlen(s) : 0;
    return mc_pbLenField(buf, tag,
                         reinterpret_cast<const uint8_t*>(s ? s : ""), sLen);
}

// ── mc_encodePosition ─────────────────────────────────────────────────────

size_t mc_encodePosition(uint8_t* buf, size_t /*cap*/,
                          int32_t lat_i, int32_t lon_i, int32_t alt_m,
                          uint32_t sats, uint32_t unixTime,
                          uint32_t speed_cm_s, uint32_t track_x100)
{
    size_t n = 0;

    auto writeFixed32 = [&](uint8_t tag, uint32_t val) {
        buf[n++] = tag;
        buf[n++] = static_cast<uint8_t>(val);
        buf[n++] = static_cast<uint8_t>(val >>  8);
        buf[n++] = static_cast<uint8_t>(val >> 16);
        buf[n++] = static_cast<uint8_t>(val >> 24);
    };

    auto writeVarint = [&](uint8_t tag, uint64_t val) {
        buf[n++] = tag;
        n += mc_pbVarint(buf + n, val);
    };

    writeFixed32(0x0D, static_cast<uint32_t>(lat_i));   // Field 1: latitude_i (sfixed32)
    writeFixed32(0x15, static_cast<uint32_t>(lon_i));   // Field 2: longitude_i (sfixed32)

    if (alt_m != 0)
        writeVarint(0x18, static_cast<uint64_t>(static_cast<int64_t>(alt_m))); // Field 3: altitude

    if (unixTime != 0)
        writeFixed32(0x25, unixTime);                   // Field 4: time (sfixed32) ← NOT field 9

    writeVarint(0x28, 2);                               // Field 5: location_source = GPS

    if (speed_cm_s != 0) writeVarint(0x50, speed_cm_s); // Field 10: ground_speed
    if (track_x100 != 0) writeVarint(0x58, track_x100); // Field 11: ground_track
    if (sats != 0)       writeVarint(0x70, sats);       // Field 14: sats_in_view ← NOT field 7

    // Field 18: precision_bits = 32 — 2-byte tag (field > 15)
    buf[n++] = 0x90; buf[n++] = 0x01;
    n += mc_pbVarint(buf + n, 32);

    return n;
}

// ── mc_encodeUser ─────────────────────────────────────────────────────────

size_t mc_encodeUser(uint8_t* buf, size_t /*cap*/,
                     uint32_t nodeId,
                     const char* longName, const char* shortName,
                     const uint8_t macaddr[6],
                     uint32_t hwModel,
                     uint8_t  deviceRole,
                     const uint8_t* publicKey,
                     bool isLicensed)
{
    size_t n = 0;

    // Field 1: id — "!xxxxxxxx"
    char idStr[12] = {};
    snprintf(idStr, sizeof(idStr), "!%08" PRIx32, nodeId);
    n += mc_pbString(buf + n, 0x0A, idStr);

    n += mc_pbString(buf + n, 0x12, longName);   // Field 2: long_name
    n += mc_pbString(buf + n, 0x1A, shortName);  // Field 3: short_name

    if (macaddr)
        n += mc_pbLenField(buf + n, 0x22, macaddr, 6); // Field 4: macaddr (6 bytes)

    buf[n++] = 0x28;                             // Field 5: hw_model (varint)
    n += mc_pbVarint(buf + n, hwModel);

    if (deviceRole != 0)
    {
        buf[n++] = 0x38;                         // Field 7: role (varint)
        n += mc_pbVarint(buf + n, deviceRole);
    }

    if (isLicensed)
    {
        buf[n++] = 0x30; buf[n++] = 0x01;       // Field 6: is_licensed = true
    }
    else if (publicKey != nullptr)
    {
        n += mc_pbLenField(buf + n, 0x42, publicKey, 32); // Field 8: public_key (32 bytes)
    }

    buf[n++] = 0x48; buf[n++] = 0x00;           // Field 9: is_unmessageable = false
    return n;
}

// ── mc_encodeData ─────────────────────────────────────────────────────────

size_t mc_encodeData(uint8_t* buf, size_t /*cap*/,
                     uint32_t portnum,
                     const uint8_t* payload, size_t payloadLen,
                     bool want_response,
                     uint32_t dest,
                     uint32_t requestId)
{
    size_t n = 0;

    auto writeFixed32 = [&](uint8_t tag, uint32_t val) {
        buf[n++] = tag;
        buf[n++] = static_cast<uint8_t>(val);
        buf[n++] = static_cast<uint8_t>(val >>  8);
        buf[n++] = static_cast<uint8_t>(val >> 16);
        buf[n++] = static_cast<uint8_t>(val >> 24);
    };

    buf[n++] = 0x08;                                    // Field 1: portnum (varint)
    n += mc_pbVarint(buf + n, portnum);

    n += mc_pbLenField(buf + n, 0x12, payload, payloadLen); // Field 2: payload (bytes)

    if (want_response) { buf[n++] = 0x18; buf[n++] = 0x01; } // Field 3: want_response

    if (dest != 0) writeFixed32(0x25, dest);            // Field 4: dest (fixed32)

    // source (field 5) intentionally omitted — only relay nodes set it

    if (requestId != 0)
    {
        writeFixed32(0x35, requestId);                  // Field 6: request_id ← tag 0x35 NOT 0x3D
        buf[n++] = 0x48; buf[n++] = 0x00;              // Field 9: ok_to_mqtt = false
    }

    return n;
}

// ── mc_encodeTelemetry ────────────────────────────────────────────────────

size_t mc_encodeTelemetry(uint8_t* buf, size_t /*cap*/,
                           uint32_t unixTime, uint32_t uptimeSec,
                           uint8_t batteryLevel, float batteryVoltage)
{
    auto writeFixed32 = [](uint8_t* dst, uint8_t tag, uint32_t val) -> size_t {
        dst[0] = tag;
        dst[1] = static_cast<uint8_t>(val);
        dst[2] = static_cast<uint8_t>(val >> 8);
        dst[3] = static_cast<uint8_t>(val >> 16);
        dst[4] = static_cast<uint8_t>(val >> 24);
        return 5;
    };

    uint8_t dm[20] = {};
    size_t  dmn    = 0;

    dm[dmn++] = 0x08;                                  // DeviceMetrics Field 1: battery_level
    dmn += mc_pbVarint(dm + dmn, batteryLevel);

    { uint32_t vbits; memcpy(&vbits, &batteryVoltage, 4);
      dmn += writeFixed32(dm + dmn, 0x15, vbits); }    // DeviceMetrics Field 2: voltage (float)

    dm[dmn++] = 0x28;                                  // DeviceMetrics Field 5: uptime_seconds
    dmn += mc_pbVarint(dm + dmn, uptimeSec);

    size_t n = 0;
    n += writeFixed32(buf + n, 0x0D, unixTime);        // Telemetry Field 1: time
    n += mc_pbLenField(buf + n, 0x12, dm, dmn);        // Telemetry Field 2: device_metrics
    return n;
}

// ── mc_encodeMapReport ────────────────────────────────────────────────────

size_t mc_encodeMapReport(uint8_t* buf, size_t /*cap*/,
                           const char* longName, const char* shortName,
                           int32_t lat_i, int32_t lon_i, int32_t alt_m,
                           uint32_t numNeighbors,
                           uint32_t hwModel,
                           uint8_t  regionCode,
                           uint8_t  modemPreset)
{
    size_t n = 0;

    auto writeFixed32 = [&](uint8_t tag, uint32_t val) {
        buf[n++] = tag;
        buf[n++] = static_cast<uint8_t>(val);
        buf[n++] = static_cast<uint8_t>(val >>  8);
        buf[n++] = static_cast<uint8_t>(val >> 16);
        buf[n++] = static_cast<uint8_t>(val >> 24);
    };

    n += mc_pbString(buf + n, 0x0A, longName);         // Field 1: long_name
    n += mc_pbString(buf + n, 0x12, shortName);        // Field 2: short_name
    buf[n++] = 0x18; n += mc_pbVarint(buf + n, hwModel);    // Field 3: hw_model
    buf[n++] = 0x28; n += mc_pbVarint(buf + n, regionCode); // Field 5: region
    buf[n++] = 0x30; n += mc_pbVarint(buf + n, modemPreset);// Field 6: modem_preset
    buf[n++] = 0x38; buf[n++] = 0x01;                 // Field 7: has_default_channel = true
    writeFixed32(0x45, static_cast<uint32_t>(lat_i)); // Field 8: latitude_i
    writeFixed32(0x4D, static_cast<uint32_t>(lon_i)); // Field 9: longitude_i

    if (alt_m != 0)
    {
        buf[n++] = 0x50;
        n += mc_pbVarint(buf + n, static_cast<uint64_t>(static_cast<int64_t>(alt_m))); // Field 10
    }

    buf[n++] = 0x58; n += mc_pbVarint(buf + n, 32);   // Field 11: position_precision

    if (numNeighbors > 0)
    {
        buf[n++] = 0x60; n += mc_pbVarint(buf + n, numNeighbors); // Field 12
    }

    return n;
}

// ── mc_parseData ──────────────────────────────────────────────────────────

bool mc_parseData(const uint8_t* data, size_t len,
                  uint32_t&       portnum,
                  const uint8_t*& payload, size_t& payloadLen,
                  bool&           wantResponse)
{
    portnum      = 0;
    payload      = nullptr;
    payloadLen   = 0;
    wantResponse = false;

    if (data == nullptr || len == 0) return false;

    size_t pos = 0;
    while (pos < len)
    {
        uint32_t tag   = 0;
        int      shift = 0;
        while (pos < len)
        {
            uint8_t b = data[pos++];
            tag |= static_cast<uint32_t>(b & 0x7F) << shift;
            if (!(b & 0x80)) break;
            shift += 7;
            if (shift > 28) return false;
        }

        const uint32_t fieldNum = tag >> 3;
        const uint8_t  wireType = tag & 0x07;

        if (wireType == 0) // varint
        {
            uint64_t val    = 0;
            int      vshift = 0;
            while (pos < len)
            {
                uint8_t b = data[pos++];
                val |= static_cast<uint64_t>(b & 0x7F) << vshift;
                if (!(b & 0x80)) break;
                vshift += 7;
                if (vshift > 63) return false;
            }
            if (fieldNum == 1) portnum      = static_cast<uint32_t>(val);
            if (fieldNum == 3) wantResponse = (val != 0);
        }
        else if (wireType == 2) // length-delimited
        {
            uint32_t msgLen = 0;
            int      lshift = 0;
            while (pos < len)
            {
                uint8_t b = data[pos++];
                msgLen |= static_cast<uint32_t>(b & 0x7F) << lshift;
                if (!(b & 0x80)) break;
                lshift += 7;
                if (lshift > 28) return false;
            }
            if (pos + msgLen > len) return false;
            if (fieldNum == 2) { payload = data + pos; payloadLen = msgLen; }
            pos += msgLen;
        }
        else if (wireType == 5) { if (pos + 4 > len) return false; pos += 4; }
        else if (wireType == 1) { if (pos + 8 > len) return false; pos += 8; }
        else return false;
    }

    return portnum != 0;
}

// ── mc_parsePosition ──────────────────────────────────────────────────────

bool mc_parsePosition(const uint8_t* data, size_t len, MeshPosition& pos)
{
    if (data == nullptr || len == 0) return false;

    size_t p       = 0;
    bool gotLatLon = false;

    auto readFixed32 = [&]() -> uint32_t {
        uint32_t v = data[p] | (uint32_t)data[p+1]<<8
                   | (uint32_t)data[p+2]<<16 | (uint32_t)data[p+3]<<24;
        p += 4;
        return v;
    };

    while (p < len)
    {
        uint32_t tag = 0; int shift = 0;
        while (p < len) {
            uint8_t b = data[p++];
            tag |= (uint32_t)(b & 0x7F) << shift;
            if (!(b & 0x80)) break;
            shift += 7; if (shift > 28) return false;
        }
        const uint32_t field    = tag >> 3;
        const uint8_t  wireType = tag & 0x07;

        if (wireType == 0) // varint
        {
            uint64_t val = 0; int vs = 0;
            while (p < len) {
                uint8_t b = data[p++];
                val |= (uint64_t)(b & 0x7F) << vs;
                if (!(b & 0x80)) break;
                vs += 7; if (vs > 63) return false;
            }
            switch (field) {
                case 3:  pos.alt_m = (int32_t)(int64_t)val; break;
                // field 9 = pos_flags (varint) — NOT time; skip silently
                case 14: pos.sats  = (uint32_t)val; break;  // NOT field 7
                default: break;
            }
        }
        else if (wireType == 5) // fixed32 / sfixed32
        {
            if (p + 4 > len) return false;
            uint32_t val = readFixed32();
            switch (field) {
                case 1: pos.lat_i    = static_cast<int32_t>(val); gotLatLon = true; break;
                case 2: pos.lon_i    = static_cast<int32_t>(val); break;
                case 4: pos.unixTime = val; break;  // time at field 4 — NOT field 9
                default: break;
            }
        }
        else if (wireType == 2) // length-delimited — skip (e.g. field 7 = google_plus_code)
        {
            uint32_t msgLen = 0; int ls = 0;
            while (p < len) {
                uint8_t b = data[p++];
                msgLen |= (uint32_t)(b & 0x7F) << ls;
                if (!(b & 0x80)) break;
                ls += 7; if (ls > 28) return false;
            }
            if (p + msgLen > len) return false;
            p += msgLen;
        }
        else if (wireType == 1) { if (p + 8 > len) return false; p += 8; }
        else return false;
    }
    return gotLatLon;
}

// ── mc_parseUser ──────────────────────────────────────────────────────────

bool mc_parseUser(const uint8_t* data, size_t len, MeshUser& user)
{
    if (data == nullptr || len == 0) return false;

    size_t p   = 0;
    bool gotId = false;

    auto readVarint = [&](uint64_t& out) -> bool {
        out = 0; int s = 0;
        while (p < len) {
            uint8_t b = data[p++];
            out |= (uint64_t)(b & 0x7F) << s;
            if (!(b & 0x80)) return true;
            s += 7; if (s > 63) return false;
        }
        return false;
    };

    while (p < len)
    {
        uint64_t tag64 = 0;
        if (!readVarint(tag64)) break;
        const uint32_t field    = (uint32_t)(tag64 >> 3);
        const uint8_t  wireType = (uint8_t)(tag64 & 0x07);

        if (wireType == 2) // string / bytes
        {
            uint64_t slen = 0;
            if (!readVarint(slen)) return false;
            if (p + slen > len) return false;
            const char* src = reinterpret_cast<const char*>(data + p);
            switch (field) {
                case 1:
                    snprintf(user.id, sizeof(user.id), "%.*s",
                             (int)std::min(slen, (uint64_t)(sizeof(user.id) - 1)), src);
                    gotId = true;
                    break;
                case 2:
                    snprintf(user.longName, sizeof(user.longName), "%.*s",
                             (int)std::min(slen, (uint64_t)(sizeof(user.longName) - 1)), src);
                    break;
                case 3:
                    snprintf(user.shortName, sizeof(user.shortName), "%.*s",
                             (int)std::min(slen, (uint64_t)(sizeof(user.shortName) - 1)), src);
                    break;
                case 4: break; // macaddr — skip
                case 8:
                    if (slen == 32) { memcpy(user.publicKey, src, 32); user.hasPublicKey = true; }
                    break;
                default: break;
            }
            p += (size_t)slen;
        }
        else if (wireType == 0)
        {
            uint64_t val = 0;
            if (!readVarint(val)) return false;
            if (field == 5) user.hwModel = (uint32_t)val;
        }
        else if (wireType == 5) { if (p + 4 > len) return false; p += 4; }
        else if (wireType == 1) { if (p + 8 > len) return false; p += 8; }
        else return false;
    }
    return gotId;
}
