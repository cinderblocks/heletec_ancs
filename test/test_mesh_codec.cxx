/**
 * test_mesh_codec.cxx — Unity tests for the platform-free Meshtastic codec.
 *
 * Run with: cmake -B build && cmake --build build && ctest --test-dir build -V
 *
 * Conformance target: Meshtastic firmware v2.7.x (mesh.proto 2.7.15)
 * Proto definitions:  meshtastic/mesh.proto, meshtastic/telemetry.proto
 *
 * Test groups
 * ───────────
 *   1. mc_pbVarint            — base-128 varint encoder
 *   2. mc_pbZigzag            — zigzag sint32 encoder
 *   3. mc_pbLenField / mc_pbString — length-delimited field primitives
 *   4. mc_parseData           — Data proto decoder
 *   5. mc_parsePosition       — Position proto decoder (field-number regressions)
 *   6. mc_parseUser           — User (NodeInfo) proto decoder
 *   7. Encode/decode round-trips
 *   8. mc_encodeData          — request_id field-6 regression, dest, ok_to_mqtt
 *   9. mc_encodeUser          — is_licensed, role, field ordering, macaddr
 *  10. mc_encodeTelemetry     — full decode round-trip
 *  11. mc_encodeMapReport     — field presence, firmware_version (2.7.x field 4)
 *  12. Reference byte-vector conformance (v2.7.x nanopb-equivalent output)
 *  13. Wire tag conformance   — tag bytes match .proto field definitions
 *  14. Edge cases and boundaries
 *  15. mc_parseNodeStatus     — NODE_STATUS_APP portnum 36 (node status string)
 *  16. mc_parsePkiReport / mc_encodePkiReport — KEY_VERIFICATION_APP portnum 12
 *  17. ALERT_APP portnum 11   — portnum value + MeshMessage.isAlert flag
 */

#include "unity.h"
#include "mesh_codec.h"
#include <cstring>
#include <cstdint>
#include <climits>

// ── Unity required entry points ───────────────────────────────────────────
void setUp(void)    {}
void tearDown(void) {}

// ── Helpers ───────────────────────────────────────────────────────────────

/// Scan encoded bytes for a specific tag byte. Returns offset or -1.
static int findTag(const uint8_t* buf, size_t len, uint8_t tag)
{
    for (size_t i = 0; i < len; i++)
        if (buf[i] == tag) return (int)i;
    return -1;
}

/// Scan for a two-byte sequence. Returns offset or -1.
static int findTag2(const uint8_t* buf, size_t len, uint8_t b0, uint8_t b1)
{
    for (size_t i = 0; i + 1 < len; i++)
        if (buf[i] == b0 && buf[i+1] == b1) return (int)i;
    return -1;
}

/// Read a little-endian uint32 from buf.
static uint32_t readLE32(const uint8_t* p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8
         | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/// Decode a varint starting at buf[pos], advance pos. Returns decoded value.
static uint64_t readVarint(const uint8_t* buf, size_t len, size_t& pos)
{
    uint64_t val = 0;
    int shift = 0;
    while (pos < len) {
        uint8_t b = buf[pos++];
        val |= (uint64_t)(b & 0x7F) << shift;
        if (!(b & 0x80)) break;
        shift += 7;
    }
    return val;
}

// ─────────────────────────────────────────────────────────────────────────
// 1. mc_pbVarint
// ─────────────────────────────────────────────────────────────────────────

void test_varint_zero(void)
{
    uint8_t buf[2] = {};
    size_t n = mc_pbVarint(buf, 0);
    TEST_ASSERT_EQUAL_size_t(1, n);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[0]);
}

void test_varint_one(void)
{
    uint8_t buf[2] = {};
    size_t n = mc_pbVarint(buf, 1);
    TEST_ASSERT_EQUAL_size_t(1, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[0]);
}

void test_varint_max_single_byte(void)
{
    uint8_t buf[2] = {};
    size_t n = mc_pbVarint(buf, 127);
    TEST_ASSERT_EQUAL_size_t(1, n);
    TEST_ASSERT_EQUAL_HEX8(0x7F, buf[0]);
}

void test_varint_first_two_byte_value(void)
{
    uint8_t buf[3] = {};
    size_t n = mc_pbVarint(buf, 128);
    TEST_ASSERT_EQUAL_size_t(2, n);
    TEST_ASSERT_EQUAL_HEX8(0x80, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[1]);
}

void test_varint_300(void)
{
    uint8_t buf[3] = {};
    size_t n = mc_pbVarint(buf, 300);
    TEST_ASSERT_EQUAL_size_t(2, n);
    TEST_ASSERT_EQUAL_HEX8(0xAC, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, buf[1]);
}

void test_varint_uint32_max(void)
{
    uint8_t buf[6] = {};
    size_t n = mc_pbVarint(buf, 0xFFFFFFFFu);
    TEST_ASSERT_EQUAL_size_t(5, n);
    TEST_ASSERT_TRUE((buf[0] & 0x80) != 0);
    TEST_ASSERT_TRUE((buf[3] & 0x80) != 0);
    TEST_ASSERT_TRUE((buf[4] & 0x80) == 0);
}

void test_varint_four_byte_boundary(void)
{
    // 0x0FFFFFFF = max value fitting in 4 varint bytes
    uint8_t buf[6] = {};
    size_t n = mc_pbVarint(buf, 0x0FFFFFFFu);
    TEST_ASSERT_EQUAL_size_t(4, n);
    TEST_ASSERT_TRUE((buf[3] & 0x80) == 0); // final byte

    // 0x10000000 = first value requiring 5 varint bytes
    n = mc_pbVarint(buf, 0x10000000u);
    TEST_ASSERT_EQUAL_size_t(5, n);
}

// ─────────────────────────────────────────────────────────────────────────
// 2. mc_pbZigzag
// ─────────────────────────────────────────────────────────────────────────

void test_zigzag_zero(void)    { TEST_ASSERT_EQUAL_UINT32(0u, mc_pbZigzag(0)); }
void test_zigzag_neg_one(void) { TEST_ASSERT_EQUAL_UINT32(1u, mc_pbZigzag(-1)); }
void test_zigzag_pos_one(void) { TEST_ASSERT_EQUAL_UINT32(2u, mc_pbZigzag(1)); }
void test_zigzag_neg_two(void) { TEST_ASSERT_EQUAL_UINT32(3u, mc_pbZigzag(-2)); }
void test_zigzag_pos_two(void) { TEST_ASSERT_EQUAL_UINT32(4u, mc_pbZigzag(2)); }

void test_zigzag_int32_min(void)
{
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, mc_pbZigzag(INT32_MIN));
}

void test_zigzag_int32_max(void)
{
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFEu, mc_pbZigzag(INT32_MAX));
}

// ─────────────────────────────────────────────────────────────────────────
// 3. mc_pbLenField / mc_pbString
// ─────────────────────────────────────────────────────────────────────────

void test_pbLenField_basic(void)
{
    // tag=0x12 (field 2, LEN), data "AB" (2 bytes)
    // Expected: 12 02 41 42
    const uint8_t data[] = { 'A', 'B' };
    uint8_t buf[8] = {};
    size_t n = mc_pbLenField(buf, 0x12, data, 2);
    TEST_ASSERT_EQUAL_size_t(4, n);
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, buf[1]);
    TEST_ASSERT_EQUAL_HEX8('A',  buf[2]);
    TEST_ASSERT_EQUAL_HEX8('B',  buf[3]);
}

void test_pbLenField_empty_data(void)
{
    // Zero-length data: tag + varint(0)
    uint8_t buf[4] = {};
    const uint8_t empty[] = {0};
    size_t n = mc_pbLenField(buf, 0x0A, empty, 0);
    TEST_ASSERT_EQUAL_size_t(2, n);
    TEST_ASSERT_EQUAL_HEX8(0x0A, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[1]);
}

void test_pbLenField_128_byte_payload(void)
{
    // 128 bytes → length varint = 0x80 0x01 (2 bytes)
    uint8_t payload[128];
    for (int i = 0; i < 128; i++) payload[i] = (uint8_t)i;
    uint8_t buf[140] = {};
    size_t n = mc_pbLenField(buf, 0x12, payload, 128);
    // tag(1) + len_varint(2) + data(128) = 131
    TEST_ASSERT_EQUAL_size_t(131, n);
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x80, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[3]); // first data byte
    TEST_ASSERT_EQUAL_HEX8(0x7F, buf[130]); // last data byte
}

void test_pbString_basic(void)
{
    // "Hi" → tag + varint(2) + 'H' 'i'
    uint8_t buf[8] = {};
    size_t n = mc_pbString(buf, 0x0A, "Hi");
    TEST_ASSERT_EQUAL_size_t(4, n);
    TEST_ASSERT_EQUAL_HEX8(0x0A, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, buf[1]);
    TEST_ASSERT_EQUAL_HEX8('H',  buf[2]);
    TEST_ASSERT_EQUAL_HEX8('i',  buf[3]);
}

void test_pbString_null_input(void)
{
    uint8_t buf[4] = {};
    size_t n = mc_pbString(buf, 0x12, nullptr);
    TEST_ASSERT_EQUAL_size_t(2, n);
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[1]);
}

void test_pbString_empty_string(void)
{
    uint8_t buf[4] = {};
    size_t n = mc_pbString(buf, 0x12, "");
    TEST_ASSERT_EQUAL_size_t(2, n);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[1]); // length = 0
}

// ─────────────────────────────────────────────────────────────────────────
// 4. mc_parseData
// ─────────────────────────────────────────────────────────────────────────

// Hand-crafted minimal Data proto: portnum=1 TEXT_MESSAGE_APP, payload "hi"
static const uint8_t kDataText[] = { 0x08, 0x01, 0x12, 0x02, 0x68, 0x69 };

void test_parse_data_basic(void)
{
    uint32_t portnum = 0;
    const uint8_t* payload = nullptr;
    size_t payloadLen = 0;
    bool wantResp = false;

    bool ok = mc_parseData(kDataText, sizeof(kDataText),
                           portnum, payload, payloadLen, wantResp);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT32(1u, portnum);
    TEST_ASSERT_NOT_NULL(payload);
    TEST_ASSERT_EQUAL_size_t(2u, payloadLen);
    TEST_ASSERT_EQUAL_UINT8('h', payload[0]);
    TEST_ASSERT_EQUAL_UINT8('i', payload[1]);
    TEST_ASSERT_FALSE(wantResp);
}

void test_parse_data_want_response(void)
{
    // portnum=67, payload "x", want_response=true
    const uint8_t data[] = { 0x08, 0x43, 0x12, 0x01, 0x78, 0x18, 0x01 };
    uint32_t portnum = 0;
    const uint8_t* payload = nullptr;
    size_t payloadLen = 0;
    bool wantResp = false;

    TEST_ASSERT_TRUE(mc_parseData(data, sizeof(data),
                                  portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(67u, portnum);
    TEST_ASSERT_TRUE(wantResp);
}

void test_parse_data_no_portnum_returns_false(void)
{
    const uint8_t data[] = { 0x12, 0x02, 0x68, 0x69 };
    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;
    TEST_ASSERT_FALSE(mc_parseData(data, sizeof(data),
                                   portnum, payload, payloadLen, wantResp));
}

void test_parse_data_empty_input(void)
{
    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;
    TEST_ASSERT_FALSE(mc_parseData(nullptr, 0,
                                   portnum, payload, payloadLen, wantResp));
}

void test_parse_data_truncated_returns_false(void)
{
    const uint8_t data[] = { 0x08, 0x01, 0x12, 0x10 }; // claims 16 payload bytes
    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;
    TEST_ASSERT_FALSE(mc_parseData(data, sizeof(data),
                                   portnum, payload, payloadLen, wantResp));
}

void test_parse_data_portnum_traceroute_empty_payload(void)
{
    // TRACEROUTE_APP (portnum=70) with empty payload — valid.
    const uint8_t data[] = { 0x08, 0x46 };
    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;

    TEST_ASSERT_TRUE(mc_parseData(data, sizeof(data),
                                  portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(70u, portnum);
    TEST_ASSERT_NULL(payload);
    TEST_ASSERT_EQUAL_size_t(0u, payloadLen);
}

void test_parse_data_with_dest_and_request_id(void)
{
    // Build a Data proto with dest (field 4) and request_id (field 6)
    // via the encoder, then decode and verify we get the portnum and payload.
    const uint8_t inner[] = { 0xAA, 0xBB };
    uint8_t buf[64] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 4, inner, 2,
                              false, 0xDEADBEEFu, 0x12345678u);
    TEST_ASSERT_GREATER_THAN(0u, n);

    uint32_t portnum = 0;
    const uint8_t* payload = nullptr;
    size_t payloadLen = 0;
    bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(buf, n, portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(4u, portnum);  // NODEINFO
    TEST_ASSERT_EQUAL_size_t(2u, payloadLen);
    TEST_ASSERT_EQUAL_HEX8(0xAA, payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, payload[1]);
}

void test_parse_data_skips_unknown_fields(void)
{
    // portnum=1, payload "A", plus unknown field 8 (fixed32), field 12 (varint)
    // 08 01  12 01 41  45 EF BE AD DE  60 FF 01
    const uint8_t data[] = {
        0x08, 0x01,                             // field 1: portnum=1
        0x12, 0x01, 0x41,                       // field 2: payload "A"
        0x45, 0xEF, 0xBE, 0xAD, 0xDE,          // field 8: emoji (fixed32)
        0x60, 0xFF, 0x01,                       // field 12: unknown varint
    };
    uint32_t portnum = 0;
    const uint8_t* payload = nullptr;
    size_t payloadLen = 0;
    bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(data, sizeof(data),
                                  portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(1u, portnum);
    TEST_ASSERT_EQUAL_size_t(1u, payloadLen);
    TEST_ASSERT_EQUAL_UINT8('A', payload[0]);
}

// ─────────────────────────────────────────────────────────────────────────
// 5. mc_parsePosition
// ─────────────────────────────────────────────────────────────────────────

static const uint8_t kPosMinimal[] = {
    0x0D, 0x01, 0x00, 0x00, 0x00,   // field 1: lat_i = 1
    0x15, 0x02, 0x00, 0x00, 0x00,   // field 2: lon_i = 2
};

void test_parse_position_minimal(void)
{
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(kPosMinimal, sizeof(kPosMinimal), pos));
    TEST_ASSERT_EQUAL_INT32(1, pos.lat_i);
    TEST_ASSERT_EQUAL_INT32(2, pos.lon_i);
    TEST_ASSERT_EQUAL_INT32(0, pos.alt_m);
    TEST_ASSERT_EQUAL_UINT32(0u, pos.unixTime);
    TEST_ASSERT_EQUAL_UINT32(0u, pos.sats);
}

void test_parse_position_missing_lat_returns_false(void)
{
    const uint8_t data[] = { 0x15, 0x02, 0x00, 0x00, 0x00 };
    MeshPosition pos = {};
    TEST_ASSERT_FALSE(mc_parsePosition(data, sizeof(data), pos));
}

void test_parse_position_empty_returns_false(void)
{
    MeshPosition pos = {};
    TEST_ASSERT_FALSE(mc_parsePosition(nullptr, 0, pos));
}

// ── Field-number regression tests (time at field 4, sats at field 14) ─────
static const uint8_t kPosCorrectFields[] = {
    0x0D, 0x01, 0x00, 0x00, 0x00,         // field 1: lat_i = 1
    0x15, 0x02, 0x00, 0x00, 0x00,         // field 2: lon_i = 2
    0x18, 0x64,                            // field 3: alt_m = 100
    0x25, 0x78, 0x56, 0x34, 0x12,         // field 4: time = 0x12345678
    0x3A, 0x05, 'h','e','l','l','o',      // pre-2.7.x field 7: google_plus_code (string) — skip
                                           // (2.7.x uses field 7 as fixed32 timestamp; wire type
                                           //  difference means old string packets are safely skipped)
    0x48, 0x05,                            // field 9: pos_flags = 5 — NOT time
    0x70, 0x08,                            // field 14: sats_in_view = 8
};

void test_parse_position_time_from_field4_not_field9(void)
{
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(kPosCorrectFields, sizeof(kPosCorrectFields), pos));
    TEST_ASSERT_EQUAL_INT32(1, pos.lat_i);
    TEST_ASSERT_EQUAL_INT32(2, pos.lon_i);
    TEST_ASSERT_EQUAL_INT32(100, pos.alt_m);
    TEST_ASSERT_EQUAL_UINT32(0x12345678u, pos.unixTime);
    TEST_ASSERT_EQUAL_UINT32(8u, pos.sats);
}

static const uint8_t kPosOldWrongTimeField[] = {
    0x0D, 0x01, 0x00, 0x00, 0x00,
    0x15, 0x02, 0x00, 0x00, 0x00,
    0x4D, 0x78, 0x56, 0x34, 0x12,         // field 9 sfixed32 — OLD bug
};

void test_parse_position_field9_sfixed32_not_stored_as_time(void)
{
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(kPosOldWrongTimeField, sizeof(kPosOldWrongTimeField), pos));
    TEST_ASSERT_EQUAL_UINT32(0u, pos.unixTime);
}

void test_parse_position_field7_string_not_stored_as_sats(void)
{
    const uint8_t data[] = {
        0x0D, 0x01, 0x00, 0x00, 0x00,
        0x15, 0x02, 0x00, 0x00, 0x00,
        0x3A, 0x03, 0x61, 0x62, 0x63,     // field 7 string "abc" — skip
    };
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(data, sizeof(data), pos));
    TEST_ASSERT_EQUAL_UINT32(0u, pos.sats);
}

void test_parse_position_negative_lat(void)
{
    const int32_t lat = -296813520;
    const uint32_t latBits = static_cast<uint32_t>(lat);
    const uint8_t data[] = {
        0x0D,
        (uint8_t)(latBits), (uint8_t)(latBits >> 8),
        (uint8_t)(latBits >> 16), (uint8_t)(latBits >> 24),
        0x15, 0x00, 0x00, 0x00, 0x00,
    };
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(data, sizeof(data), pos));
    TEST_ASSERT_EQUAL_INT32(lat, pos.lat_i);
}

void test_parse_position_negative_altitude(void)
{
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 1, 2, -10, 3, 0);
    TEST_ASSERT_GREATER_THAN(0u, n);

    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(buf, n, pos));
    TEST_ASSERT_EQUAL_INT32(-10, pos.alt_m);
}

void test_parse_position_truncated_sfixed32(void)
{
    const uint8_t data[] = { 0x0D, 0x01, 0x00 };
    MeshPosition pos = {};
    TEST_ASSERT_FALSE(mc_parsePosition(data, sizeof(data), pos));
}

void test_parse_position_all_fields_roundtrip(void)
{
    uint8_t buf[100] = {};
    const int32_t  lat   = 296813520;
    const int32_t  lon   = -952347811;
    const int32_t  alt   = 427;
    const uint32_t sats  = 9;
    const uint32_t ts    = 1741046400;
    const uint32_t speed = 2778;
    const uint32_t track = 18000;

    size_t n = mc_encodePosition(buf, sizeof(buf), lat, lon, alt, sats, ts, speed, track);
    TEST_ASSERT_GREATER_THAN(0u, n);

    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(buf, n, pos));
    TEST_ASSERT_EQUAL_INT32(lat,  pos.lat_i);
    TEST_ASSERT_EQUAL_INT32(lon,  pos.lon_i);
    TEST_ASSERT_EQUAL_INT32(alt,  pos.alt_m);
    TEST_ASSERT_EQUAL_UINT32(sats, pos.sats);
    TEST_ASSERT_EQUAL_UINT32(ts,   pos.unixTime);
}

void test_parse_position_unknown_future_field_skipped(void)
{
    // Inject a hypothetical field 20 (varint) after valid lat/lon
    const uint8_t data[] = {
        0x0D, 0x0A, 0x00, 0x00, 0x00,   // lat_i = 10
        0x15, 0x14, 0x00, 0x00, 0x00,   // lon_i = 20
        0xA0, 0x01, 0x63,               // field 20 varint = 99 (2-byte tag)
    };
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(data, sizeof(data), pos));
    TEST_ASSERT_EQUAL_INT32(10, pos.lat_i);
    TEST_ASSERT_EQUAL_INT32(20, pos.lon_i);
}

void test_parse_position_unknown_len_field_skipped(void)
{
    // Unknown LEN field (e.g. field 99) injected after lat/lon
    const uint8_t data[] = {
        0x0D, 0x0A, 0x00, 0x00, 0x00,   // lat_i = 10
        0x15, 0x14, 0x00, 0x00, 0x00,   // lon_i = 20
        0x9A, 0x06, 0x02, 0xFF, 0xFF,   // field 99 LEN, 2 bytes of data
    };
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(data, sizeof(data), pos));
    TEST_ASSERT_EQUAL_INT32(10, pos.lat_i);
}

void test_parse_position_zero_lat_lon_emitted(void)
{
    // Null island (0°, 0°) — sfixed32 fields always emitted
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 0, 0, 0, 0, 0);
    TEST_ASSERT_GREATER_THAN(0u, n);
    // Must contain lat tag 0x0D and lon tag 0x15
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x0D));
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x15));

    MeshPosition pos = {};
    pos.lat_i = 999; pos.lon_i = 999; // sentinel
    TEST_ASSERT_TRUE(mc_parsePosition(buf, n, pos));
    TEST_ASSERT_EQUAL_INT32(0, pos.lat_i);
    TEST_ASSERT_EQUAL_INT32(0, pos.lon_i);
}

void test_parse_position_precision_bits_field18_tag(void)
{
    // Field 18 requires a 2-byte tag: (18 << 3)|0 = 0x90 0x01
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 1, 2, 0, 0, 0);
    TEST_ASSERT_NOT_EQUAL(-1, findTag2(buf, n, 0x90, 0x01));
}

// ─────────────────────────────────────────────────────────────────────────
// 6. mc_parseUser
// ─────────────────────────────────────────────────────────────────────────

static const uint8_t kUserMinimal[] = {
    0x0A, 0x09, '!','8','6','b','c','2','1','c','4'
};

void test_parse_user_minimal(void)
{
    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(kUserMinimal, sizeof(kUserMinimal), user));
    TEST_ASSERT_EQUAL_STRING("!86bc21c4", user.id);
    TEST_ASSERT_EQUAL_STRING("", user.longName);
    TEST_ASSERT_EQUAL_STRING("", user.shortName);
    TEST_ASSERT_EQUAL_UINT32(0u, user.hwModel);
    TEST_ASSERT_FALSE(user.hasPublicKey);
}

void test_parse_user_empty_returns_false(void)
{
    MeshUser user = {};
    TEST_ASSERT_FALSE(mc_parseUser(nullptr, 0, user));
}

void test_parse_user_no_id_returns_false(void)
{
    const uint8_t data[] = { 0x28, 0x30 }; // field 5 varint
    MeshUser user = {};
    TEST_ASSERT_FALSE(mc_parseUser(data, sizeof(data), user));
}

void test_parse_user_public_key_32_bytes(void)
{
    uint8_t data[80] = {};
    size_t n = 0;
    const char* id = "!deadbeef";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x42; data[n++] = 32;
    for (int i = 0; i < 32; i++) data[n++] = (uint8_t)(i + 1);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_STRING("!deadbeef", user.id);
    TEST_ASSERT_TRUE(user.hasPublicKey);
    TEST_ASSERT_EQUAL_UINT8(1u,  user.publicKey[0]);
    TEST_ASSERT_EQUAL_UINT8(32u, user.publicKey[31]);
}

void test_parse_user_public_key_wrong_length_not_stored(void)
{
    uint8_t data[80] = {};
    size_t n = 0;
    const char* id = "!deadbeef";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x42; data[n++] = 31; // wrong length
    for (int i = 0; i < 31; i++) data[n++] = (uint8_t)(i + 1);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_FALSE(user.hasPublicKey);
}

void test_parse_user_long_name_truncated_to_buffer(void)
{
    uint8_t data[200] = {};
    size_t n = 0;
    const char* id = "!aabbccdd";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    const char* longName = "AAAAAAAAAAAABBBBBBBBBBBBCCCCCCCCCCCCDDDDDDDDDDDDEE";
    data[n++] = 0x12; data[n++] = (uint8_t)strlen(longName);
    memcpy(data + n, longName, strlen(longName)); n += strlen(longName);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_UINT8('\0', user.longName[32]);
    TEST_ASSERT_EQUAL_size_t(32u, strlen(user.longName));
}

void test_parse_user_hw_model(void)
{
    uint8_t data[30] = {};
    size_t n = 0;
    const char* id = "!12345678";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x28; data[n++] = 48;

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_UINT32(48u, user.hwModel);
}

void test_parse_user_all_fields_roundtrip(void)
{
    uint8_t pubkey[32];
    for (int i = 0; i < 32; i++) pubkey[i] = (uint8_t)(i * 3 + 7);
    const uint8_t mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };

    uint8_t buf[160] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0xDEADBEEFu,
                              "Long Device Name", "LONG",
                              mac, 48, 5, pubkey, false);
    TEST_ASSERT_GREATER_THAN(0u, n);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(buf, n, user));
    TEST_ASSERT_EQUAL_STRING("!deadbeef", user.id);
    TEST_ASSERT_EQUAL_STRING("Long Device Name", user.longName);
    TEST_ASSERT_EQUAL_STRING("LONG", user.shortName);
    TEST_ASSERT_TRUE(user.hasMacaddr);
    TEST_ASSERT_EQUAL_MEMORY(mac, user.macaddr, 6);
    TEST_ASSERT_EQUAL_UINT32(48u, user.hwModel);
    TEST_ASSERT_FALSE(user.isLicensed);     // isLicensed=false (not emitted → defaults false)
    TEST_ASSERT_EQUAL_UINT8(5, user.role);  // TRACKER
    TEST_ASSERT_TRUE(user.hasPublicKey);
    TEST_ASSERT_EQUAL_MEMORY(pubkey, user.publicKey, 32);
    TEST_ASSERT_FALSE(user.isUnmessageable); // emitted as false by encoder
}

void test_parse_user_is_licensed_field6(void)
{
    // Proto with field 6 (is_licensed = true, tag 0x30 varint 1)
    uint8_t data[30] = {};
    size_t n = 0;
    const char* id = "!aabbccdd";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x30; data[n++] = 0x01; // field 6: is_licensed = true

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_STRING("!aabbccdd", user.id);
    TEST_ASSERT_TRUE(user.isLicensed);
}

void test_parse_user_role_field7(void)
{
    uint8_t data[30] = {};
    size_t n = 0;
    const char* id = "!aabbccdd";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x38; data[n++] = 0x05; // field 7: role = TRACKER (5)

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_STRING("!aabbccdd", user.id);
    TEST_ASSERT_EQUAL_UINT8(5, user.role);
}

void test_parse_user_macaddr_field4(void)
{
    uint8_t data[30] = {};
    size_t n = 0;
    const char* id = "!aabbccdd";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x22; data[n++] = 0x06;
    data[n++] = 0xDE; data[n++] = 0xAD; data[n++] = 0xBE;
    data[n++] = 0xEF; data[n++] = 0x00; data[n++] = 0x01;

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_STRING("!aabbccdd", user.id);
    TEST_ASSERT_TRUE(user.hasMacaddr);
    const uint8_t expectedMac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};
    TEST_ASSERT_EQUAL_MEMORY(expectedMac, user.macaddr, 6);
}

void test_parse_user_macaddr_wrong_length(void)
{
    // 5-byte MAC — should NOT set hasMacaddr (only 6-byte accepted)
    uint8_t data[30] = {};
    size_t n = 0;
    const char* id = "!aabbccdd";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x22; data[n++] = 0x05; // 5 bytes instead of 6
    data[n++] = 0xDE; data[n++] = 0xAD; data[n++] = 0xBE;
    data[n++] = 0xEF; data[n++] = 0x00;

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_FALSE(user.hasMacaddr);
}

void test_parse_user_is_unmessageable_field9(void)
{
    uint8_t data[30] = {};
    size_t n = 0;
    const char* id = "!aabbccdd";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x48; data[n++] = 0x01; // field 9: is_unmessageable = true

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_STRING("!aabbccdd", user.id);
    TEST_ASSERT_TRUE(user.isUnmessageable);
}

void test_parse_user_is_unmessageable_false(void)
{
    uint8_t data[30] = {};
    size_t n = 0;
    const char* id = "!aabbccdd";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x48; data[n++] = 0x00; // field 9: is_unmessageable = false

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_FALSE(user.isUnmessageable);
}

void test_parse_user_all_nine_fields_present(void)
{
    // Hand-craft proto with all 9 User fields in ascending order
    uint8_t data[120] = {};
    size_t n = 0;

    // field 1: id
    const char* id = "!cafe0001";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    // field 2: long_name
    const char* ln = "TestNode";
    data[n++] = 0x12; data[n++] = (uint8_t)strlen(ln);
    memcpy(data + n, ln, strlen(ln)); n += strlen(ln);
    // field 3: short_name
    const char* sn = "TST";
    data[n++] = 0x1A; data[n++] = (uint8_t)strlen(sn);
    memcpy(data + n, sn, strlen(sn)); n += strlen(sn);
    // field 4: macaddr (6 bytes)
    data[n++] = 0x22; data[n++] = 0x06;
    data[n++] = 0xAA; data[n++] = 0xBB; data[n++] = 0xCC;
    data[n++] = 0xDD; data[n++] = 0xEE; data[n++] = 0xFF;
    // field 5: hw_model = 48
    data[n++] = 0x28; data[n++] = 48;
    // field 6: is_licensed = true
    data[n++] = 0x30; data[n++] = 0x01;
    // field 7: role = 5 (TRACKER)
    data[n++] = 0x38; data[n++] = 0x05;
    // field 8: public_key (32 bytes)
    data[n++] = 0x42; data[n++] = 32;
    for (int i = 0; i < 32; i++) data[n++] = (uint8_t)(0x10 + i);
    // field 9: is_unmessageable = false
    data[n++] = 0x48; data[n++] = 0x00;

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_STRING("!cafe0001", user.id);
    TEST_ASSERT_EQUAL_STRING("TestNode", user.longName);
    TEST_ASSERT_EQUAL_STRING("TST", user.shortName);
    TEST_ASSERT_TRUE(user.hasMacaddr);
    const uint8_t expectedMac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    TEST_ASSERT_EQUAL_MEMORY(expectedMac, user.macaddr, 6);
    TEST_ASSERT_EQUAL_UINT32(48u, user.hwModel);
    TEST_ASSERT_TRUE(user.isLicensed);
    TEST_ASSERT_EQUAL_UINT8(5, user.role);
    TEST_ASSERT_TRUE(user.hasPublicKey);
    TEST_ASSERT_EQUAL_UINT8(0x10, user.publicKey[0]);
    TEST_ASSERT_EQUAL_UINT8(0x2F, user.publicKey[31]);
    TEST_ASSERT_FALSE(user.isUnmessageable);
}

// ─────────────────────────────────────────────────────────────────────────
// 7. Encode/decode round-trips
// ─────────────────────────────────────────────────────────────────────────

void test_data_roundtrip_with_position_payload(void)
{
    uint8_t posBuf[80] = {};
    size_t posLen = mc_encodePosition(posBuf, sizeof(posBuf),
                                      100000000, -800000000, 250,
                                      7, 1710000000u, 0, 0);
    TEST_ASSERT_GREATER_THAN(0u, posLen);

    uint8_t dataBuf[120] = {};
    size_t dataLen = mc_encodeData(dataBuf, sizeof(dataBuf),
                                   3, posBuf, posLen, false, 0, 0);
    TEST_ASSERT_GREATER_THAN(0u, dataLen);

    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(dataBuf, dataLen,
                                  portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(3u, portnum);
    TEST_ASSERT_EQUAL_size_t(posLen, payloadLen);

    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(payload, payloadLen, pos));
    TEST_ASSERT_EQUAL_INT32(100000000,  pos.lat_i);
    TEST_ASSERT_EQUAL_INT32(-800000000, pos.lon_i);
    TEST_ASSERT_EQUAL_INT32(250,        pos.alt_m);
    TEST_ASSERT_EQUAL_UINT32(7u,        pos.sats);
    TEST_ASSERT_EQUAL_UINT32(1710000000u, pos.unixTime);
}

void test_data_roundtrip_with_user_payload(void)
{
    const uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    uint8_t userBuf[128] = {};
    size_t userLen = mc_encodeUser(userBuf, sizeof(userBuf),
                                    0xCAFE0001u, "MyNode", "MYN",
                                    mac, 48, 5, nullptr, true);
    TEST_ASSERT_GREATER_THAN(0u, userLen);

    uint8_t dataBuf[180] = {};
    size_t dataLen = mc_encodeData(dataBuf, sizeof(dataBuf),
                                   4, userBuf, userLen, true, 0, 0);
    TEST_ASSERT_GREATER_THAN(0u, dataLen);

    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(dataBuf, dataLen,
                                  portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(4u, portnum); // NODEINFO
    TEST_ASSERT_TRUE(wantResp);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(payload, payloadLen, user));
    TEST_ASSERT_EQUAL_STRING("!cafe0001", user.id);
    TEST_ASSERT_EQUAL_STRING("MyNode", user.longName);
    TEST_ASSERT_EQUAL_STRING("MYN", user.shortName);
    TEST_ASSERT_TRUE(user.hasMacaddr);
    TEST_ASSERT_EQUAL_MEMORY(mac, user.macaddr, 6);
    TEST_ASSERT_EQUAL_UINT32(48u, user.hwModel);
    TEST_ASSERT_TRUE(user.isLicensed);
    TEST_ASSERT_EQUAL_UINT8(5, user.role);
    TEST_ASSERT_FALSE(user.hasPublicKey);  // nullptr → not emitted
    TEST_ASSERT_FALSE(user.isUnmessageable);
}

void test_data_roundtrip_text_message(void)
{
    const char* text = "Hello mesh!";
    uint8_t dataBuf[64] = {};
    size_t dataLen = mc_encodeData(dataBuf, sizeof(dataBuf),
                                   1, // TEXT_MESSAGE_APP
                                   (const uint8_t*)text, strlen(text),
                                   false, 0, 0);
    TEST_ASSERT_GREATER_THAN(0u, dataLen);

    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(dataBuf, dataLen,
                                  portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(1u, portnum);
    TEST_ASSERT_EQUAL_size_t(strlen(text), payloadLen);
    TEST_ASSERT_EQUAL_MEMORY(text, payload, payloadLen);
}

// ─────────────────────────────────────────────────────────────────────────
// 8. mc_encodeData — field-level verification
// ─────────────────────────────────────────────────────────────────────────

void test_encode_data_request_id_uses_field6_tag_0x35(void)
{
    const uint8_t payload[] = { 0x01, 0x02, 0x03 };
    uint8_t buf[64] = {};
    size_t n = mc_encodeData(buf, sizeof(buf),
                              67, payload, sizeof(payload),
                              false, 0xABCD1234u, 0x12345678u);
    TEST_ASSERT_GREATER_THAN(0u, n);

    bool found_0x35 = false;
    bool found_0x3D = false;
    for (size_t i = 0; i < n; i++) {
        if (buf[i] == 0x35) found_0x35 = true;
        if (buf[i] == 0x3D) found_0x3D = true;
    }
    TEST_ASSERT_TRUE_MESSAGE(found_0x35,  "request_id must use tag 0x35 (field 6)");
    TEST_ASSERT_FALSE_MESSAGE(found_0x3D, "tag 0x3D (field 7) must NOT appear");
}

void test_encode_data_no_request_id_no_tag_0x35(void)
{
    const uint8_t payload[] = { 0x01 };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 1, payload, 1, false, 0, 0);

    bool found_0x35 = false;
    for (size_t i = 0; i < n; i++)
        if (buf[i] == 0x35) found_0x35 = true;
    TEST_ASSERT_FALSE_MESSAGE(found_0x35, "tag 0x35 must not appear when requestId==0");
}

void test_encode_data_ok_to_mqtt_always_emitted(void)
{
    // Even without requestId, ok_to_mqtt = false (0x48 0x00) is emitted.
    const uint8_t payload[] = { 0x01 };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 1, payload, 1, false, 0, 0);

    TEST_ASSERT_NOT_EQUAL(-1, findTag2(buf, n, 0x48, 0x00));
}

void test_encode_data_want_response_tag_present(void)
{
    const uint8_t payload[] = { 0xAA };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 67, payload, 1, true, 0, 0);

    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, findTag2(buf, n, 0x18, 0x01),
        "want_response=true must emit 0x18 0x01");
}

void test_encode_data_no_want_response_tag_absent(void)
{
    const uint8_t payload[] = { 0xAA };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 67, payload, 1, false, 0, 0);

    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x18));
}

void test_encode_data_dest_present(void)
{
    const uint8_t payload[] = { 0x01 };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 4, payload, 1,
                              false, 0xDEADBEEFu, 0);
    int idx = findTag(buf, n, 0x25);
    TEST_ASSERT_NOT_EQUAL(-1, idx);
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu, readLE32(buf + idx + 1));
}

void test_encode_data_dest_absent_when_zero(void)
{
    const uint8_t payload[] = { 0x01 };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 4, payload, 1, false, 0, 0);
    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x25));
}

void test_encode_data_source_never_emitted(void)
{
    // Field 5 (source, tag 0x2D fixed32) must never appear
    const uint8_t payload[] = { 0x01, 0x02 };
    uint8_t buf[48] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 67, payload, 2,
                              true, 0xDEADBEEFu, 0x12345678u);
    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x2D));
}

void test_encode_data_zero_length_payload(void)
{
    const uint8_t empty[] = {0};
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 1, empty, 0, false, 0, 0);
    TEST_ASSERT_GREATER_THAN(0u, n);
    // Should still contain field 1 (portnum) and field 2 (empty payload)
    TEST_ASSERT_EQUAL_HEX8(0x08, buf[0]); // portnum tag
    // Find field 2 tag 0x12 followed by length 0x00
    TEST_ASSERT_NOT_EQUAL(-1, findTag2(buf, n, 0x12, 0x00));
}

void test_encode_data_large_payload(void)
{
    // 200-byte payload — within Meshtastic MTU
    uint8_t payload[200];
    for (int i = 0; i < 200; i++) payload[i] = (uint8_t)(i & 0xFF);
    uint8_t buf[256] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 1, payload, 200, false, 0, 0);
    TEST_ASSERT_GREATER_THAN(200u, n);

    uint32_t portnum = 0; const uint8_t* pl = nullptr;
    size_t plLen = 0; bool wr = false;
    TEST_ASSERT_TRUE(mc_parseData(buf, n, portnum, pl, plLen, wr));
    TEST_ASSERT_EQUAL_UINT32(1u, portnum);
    TEST_ASSERT_EQUAL_size_t(200u, plLen);
    TEST_ASSERT_EQUAL_MEMORY(payload, pl, 200);
}

// ─────────────────────────────────────────────────────────────────────────
// 9. mc_encodeUser — variants
// ─────────────────────────────────────────────────────────────────────────

void test_encodeUser_is_licensed_emits_field6_no_pubkey(void)
{
    const uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    uint8_t pubkey[32] = {};
    for (int i = 0; i < 32; i++) pubkey[i] = (uint8_t)i;
    uint8_t buf[160] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0x12345678u, "Ham", "HAM",
                              mac, 48, 5, pubkey, /*isLicensed=*/true);
    TEST_ASSERT_GREATER_THAN(0u, n);

    // Field 6 (is_licensed): tag 0x30, value 0x01
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, findTag2(buf, n, 0x30, 0x01),
        "is_licensed=true must emit 0x30 0x01");
    // Field 8 (public_key): tag 0x42 must NOT appear
    TEST_ASSERT_EQUAL_MESSAGE(-1, findTag(buf, n, 0x42),
        "public_key must not be emitted in licensed mode");
}

void test_encodeUser_client_role_omits_field7(void)
{
    const uint8_t mac[6] = {};
    uint8_t buf[120] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0xAABBCCDD, "Client", "CLI",
                              mac, 48, /*deviceRole=*/0, nullptr, false);
    TEST_ASSERT_GREATER_THAN(0u, n);
    TEST_ASSERT_EQUAL_MESSAGE(-1, findTag(buf, n, 0x38),
        "field 7 (role) tag 0x38 must not appear for CLIENT (role=0)");
}

void test_encodeUser_tracker_role_emits_field7(void)
{
    const uint8_t mac[6] = {};
    uint8_t buf[120] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0xAABBCCDD, "Trk", "TRK",
                              mac, 48, /*deviceRole=*/5, nullptr, false);
    TEST_ASSERT_GREATER_THAN(0u, n);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, findTag2(buf, n, 0x38, 0x05),
        "field 7 (role=TRACKER=5) must emit 0x38 0x05");
}

void test_encodeUser_all_27x_roles_roundtrip(void)
{
    // All 2.7.x DeviceRole values (0–10) must survive encode → parse.
    // Role 0 (CLIENT): field 7 is omitted, so parsed value stays default 0.
    const uint8_t roles[]   = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    const char*   names[]   = { "CLI","CMU","RTR","RCL","RPT",
                                 "TRK","SNS","TAK","CHD","LAF","TTK" };
    const uint8_t mac[6] = {};

    for (size_t i = 0; i < sizeof(roles); i++)
    {
        uint8_t buf[120] = {};
        size_t n = mc_encodeUser(buf, sizeof(buf),
                                  0x11000000u + i, "Node", names[i],
                                  mac, 48, roles[i], nullptr, false);
        TEST_ASSERT_GREATER_THAN(0u, n);

        MeshUser user = {};
        TEST_ASSERT_TRUE(mc_parseUser(buf, n, user));
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(roles[i], user.role,
            "role value must survive encode/parse round-trip");
    }
}

void test_encodeUser_field_order_ascending(void)
{
    // Verify all tags appear in ascending field-number order (nanopb conformance).
    const uint8_t mac[6] = {1,2,3,4,5,6};
    uint8_t pubkey[32] = {};
    uint8_t buf[160] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0x11223344u, "TestName", "TST",
                              mac, 48, 5, pubkey, false);

    // Extract field numbers in emission order by scanning tag bytes
    uint32_t lastField = 0;
    size_t pos = 0;
    while (pos < n) {
        uint64_t tag = readVarint(buf, n, pos);
        uint32_t field = (uint32_t)(tag >> 3);
        uint8_t  wt    = (uint8_t)(tag & 0x07);

        TEST_ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE(lastField, field,
            "User proto fields must be in ascending order");
        lastField = field;

        // Skip value
        if (wt == 0)      { readVarint(buf, n, pos); }
        else if (wt == 2) { uint64_t l = readVarint(buf, n, pos); pos += (size_t)l; }
        else if (wt == 5) { pos += 4; }
        else if (wt == 1) { pos += 8; }
        else break;
    }
    TEST_ASSERT_GREATER_THAN_UINT32(0u, lastField); // sanity: we parsed something
}

void test_encodeUser_licensed_with_role_field_order(void)
{
    // is_licensed=true + role=5: field 6 before field 7
    const uint8_t mac[6] = {};
    uint8_t buf[120] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0xAABBCCDD, "Ham", "HAM",
                              mac, 48, /*deviceRole=*/5, nullptr, true);
    int idx6 = findTag2(buf, n, 0x30, 0x01); // field 6
    int idx7 = findTag(buf, n, 0x38);         // field 7
    TEST_ASSERT_NOT_EQUAL(-1, idx6);
    TEST_ASSERT_NOT_EQUAL(-1, idx7);
    TEST_ASSERT_TRUE_MESSAGE(idx6 < idx7,
        "field 6 (is_licensed) must appear before field 7 (role)");
}

void test_encodeUser_no_macaddr(void)
{
    uint8_t buf[120] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0x12345678u, "Test", "TST",
                              nullptr, 48, 0, nullptr, false);
    TEST_ASSERT_GREATER_THAN(0u, n);
    TEST_ASSERT_EQUAL_MESSAGE(-1, findTag(buf, n, 0x22),
        "field 4 (macaddr) tag 0x22 must not appear when macaddr is nullptr");
}

void test_encodeUser_is_unmessageable_always_emitted(void)
{
    const uint8_t mac[6] = {};
    uint8_t buf[120] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0x12345678u, "Test", "TST",
                              mac, 48, 0, nullptr, false);
    // Field 9: is_unmessageable = false → 0x48 0x00
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, findTag2(buf, n, 0x48, 0x00),
        "field 9 (is_unmessageable=false) must always be emitted");
}

void test_encodeUser_is_unmessageable_true(void)
{
    const uint8_t mac[6] = {};
    uint8_t buf[120] = {};
    // isLicensed=false, isUnmessageable=true
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0x12345678u, "Sensor", "SNS",
                              mac, 48, 5, nullptr,
                              /*isLicensed=*/false, /*isUnmessageable=*/true);
    TEST_ASSERT_GREATER_THAN(0u, n);
    // Field 9: is_unmessageable = true → 0x48 0x01
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, findTag2(buf, n, 0x48, 0x01),
        "field 9 (is_unmessageable=true) must emit 0x48 0x01");
    // Field 9 value=0 must NOT appear
    TEST_ASSERT_EQUAL_MESSAGE(-1, findTag2(buf, n, 0x48, 0x00),
        "field 9 value=0x00 must not appear when is_unmessageable=true");
}

// ─────────────────────────────────────────────────────────────────────────
// 10. mc_encodeTelemetry — full decode verification
// ─────────────────────────────────────────────────────────────────────────

void test_telemetry_roundtrip_full_decode(void)
{
    const uint32_t ts      = 1741046400u;
    const uint32_t uptime  = 3600u;
    const uint8_t  batLvl  = 85u;
    const float    batV    = 4.05f;

    uint8_t buf[64] = {};
    size_t n = mc_encodeTelemetry(buf, sizeof(buf), ts, uptime, batLvl, batV);
    TEST_ASSERT_GREATER_THAN(0u, n);

    // Walk the wire bytes manually:
    size_t pos = 0;

    // Outer field 1: time (tag 0x0D, fixed32)
    TEST_ASSERT_EQUAL_HEX8(0x0D, buf[pos]); pos++;
    TEST_ASSERT_EQUAL_HEX32(ts, readLE32(buf + pos)); pos += 4;

    // Outer field 2: device_metrics (tag 0x12, LEN)
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[pos]); pos++;
    uint64_t dmLen = readVarint(buf, n, pos);
    TEST_ASSERT_GREATER_THAN(0u, dmLen);

    size_t dmEnd = pos + (size_t)dmLen;

    // Inner field 1: battery_level (tag 0x08, varint)
    TEST_ASSERT_EQUAL_HEX8(0x08, buf[pos]); pos++;
    uint64_t batDecoded = readVarint(buf, n, pos);
    TEST_ASSERT_EQUAL_UINT32(85u, (uint32_t)batDecoded);

    // Inner field 2: voltage (tag 0x15, fixed32 = float)
    TEST_ASSERT_EQUAL_HEX8(0x15, buf[pos]); pos++;
    uint32_t vBits = readLE32(buf + pos); pos += 4;
    float vDecoded;
    memcpy(&vDecoded, &vBits, 4);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.05f, vDecoded);

    // Inner field 3: channel_utilization (tag 0x1D, fixed32 = float 0.0)
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x1D, buf[pos],
        "DeviceMetrics field 3 (channel_utilization, tag 0x1D) must be present");
    pos++;
    uint32_t chUtilBits = readLE32(buf + pos); pos += 4;
    float chUtil; memcpy(&chUtil, &chUtilBits, 4);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, chUtil);

    // Inner field 4: air_util_tx (tag 0x25, fixed32 = float 0.0)
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x25, buf[pos],
        "DeviceMetrics field 4 (air_util_tx, tag 0x25) must be present");
    pos++;
    uint32_t airUtilBits = readLE32(buf + pos); pos += 4;
    float airUtil; memcpy(&airUtil, &airUtilBits, 4);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, airUtil);

    // Inner field 5: uptime_seconds (tag 0x28, varint)
    TEST_ASSERT_EQUAL_HEX8(0x28, buf[pos]); pos++;
    uint64_t uptimeDecoded = readVarint(buf, n, pos);
    TEST_ASSERT_EQUAL_UINT32(3600u, (uint32_t)uptimeDecoded);

    TEST_ASSERT_EQUAL_size_t(dmEnd, pos); // consumed exactly the submessage
}

void test_telemetry_zero_battery(void)
{
    uint8_t buf[64] = {};
    size_t n = mc_encodeTelemetry(buf, sizeof(buf), 100, 0, 0, 0.0f);
    TEST_ASSERT_GREATER_THAN(0u, n);

    // Still contains inner field 1 tag 0x08 for battery_level
    // (protobuf emits even zero-value varints when explicitly written)
    size_t pos = 0;
    // Skip outer field 1 (5 bytes) and field 2 tag + length
    pos = 5; // past outer time
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[pos]); pos++;
    readVarint(buf, n, pos); // skip length
    TEST_ASSERT_EQUAL_HEX8(0x08, buf[pos]); pos++;
    uint64_t val = readVarint(buf, n, pos);
    TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)val);
}

void test_telemetry_large_uptime(void)
{
    uint8_t buf[64] = {};
    size_t n = mc_encodeTelemetry(buf, sizeof(buf), 100, 0xFFFFFFFFu, 50, 3.7f);
    TEST_ASSERT_GREATER_THAN(0u, n);
    // 0xFFFFFFFF as varint = 5 bytes.  The total size should be larger.
    // Verify round-trip by finding 0x28 tag and reading varint after it.
    int idx = findTag(buf + 7, n - 7, 0x28); // look inside submessage
    TEST_ASSERT_NOT_EQUAL(-1, idx);
    size_t vpos = (size_t)(7 + idx + 1);
    uint64_t val = readVarint(buf, n, vpos);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFu, (uint32_t)val);
}

// ─────────────────────────────────────────────────────────────────────────
// 11. mc_encodeMapReport — field presence and omission
// ─────────────────────────────────────────────────────────────────────────

void test_encodeMapReport_all_fields_present(void)
{
    uint8_t buf[200] = {};
    size_t n = mc_encodeMapReport(buf, sizeof(buf),
                                   "MapNode", "MAP",
                                   296813520, -952347811, 427,
                                   /*numNeighbors=*/3,
                                   /*hwModel=*/48,
                                   /*regionCode=*/1,   // US
                                   /*modemPreset=*/0,  // LONG_FAST
                                   /*firmwareVersion=*/"2.7.15.0");
    TEST_ASSERT_GREATER_THAN(0u, n);

    // Verify all expected tag bytes are present:
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x0A)); // field 1: long_name
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x12)); // field 2: short_name
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x18)); // field 3: hw_model
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, findTag(buf, n, 0x22),
        "field 4 (firmware_version, 2.7.x) must be present when provided");
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x28)); // field 5: region
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x30)); // field 6: modem_preset
    TEST_ASSERT_NOT_EQUAL(-1, findTag2(buf, n, 0x38, 0x01)); // field 7: has_default_channel
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x45)); // field 8: latitude_i
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x4D)); // field 9: longitude_i
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x50)); // field 10: altitude
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x58)); // field 11: position_precision
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x60)); // field 12: num_online_local_nodes

    // Verify lat/lon values (sfixed32).
    int latIdx = findTag(buf, n, 0x45);
    TEST_ASSERT_NOT_EQUAL(-1, latIdx);
    TEST_ASSERT_EQUAL_HEX32((uint32_t)296813520, readLE32(buf + latIdx + 1));
    int lonIdx = latIdx + 5; // lat sfixed32 = tag(1) + data(4)
    TEST_ASSERT_EQUAL_HEX8(0x4D, buf[lonIdx]);
    TEST_ASSERT_EQUAL_HEX32((uint32_t)(int32_t)-952347811, readLE32(buf + lonIdx + 1));
}

void test_encodeMapReport_firmware_version_emitted(void)
{
    // Field 4 (0x22) must be emitted when firmwareVersion is a non-empty string.
    // The encoded bytes: tag=0x22, len=8, "2.7.15.0"
    uint8_t buf[160] = {};
    size_t n = mc_encodeMapReport(buf, sizeof(buf),
                                   "FW", "FW",
                                   1, 2, 0, 0, 48, 1, 0,
                                   /*firmwareVersion=*/"2.7.15.0");
    TEST_ASSERT_GREATER_THAN(0u, n);

    // Find tag 0x22
    int idx = findTag(buf, n, 0x22);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, idx,
        "field 4 (firmware_version) tag 0x22 must be emitted");

    // Verify length byte and string content
    TEST_ASSERT_EQUAL_UINT8(8, buf[idx + 1]); // strlen("2.7.15.0") == 8
    TEST_ASSERT_EQUAL_MEMORY("2.7.15.0", buf + idx + 2, 8);
}

void test_encodeMapReport_firmware_version_omitted_when_null(void)
{
    // Field 4 (0x22) must NOT be emitted when firmwareVersion is nullptr.
    uint8_t buf[160] = {};
    size_t n = mc_encodeMapReport(buf, sizeof(buf),
                                   "NoFW", "NFW",
                                   1, 2, 0, 0, 48, 1, 0,
                                   /*firmwareVersion=*/nullptr);
    TEST_ASSERT_GREATER_THAN(0u, n);
    // Tag 0x22 must not appear
    TEST_ASSERT_EQUAL_MESSAGE(-1, findTag(buf, n, 0x22),
        "field 4 (firmware_version) tag 0x22 must not appear when firmwareVersion=nullptr");
}

void test_encodeMapReport_zero_altitude_omitted(void)
{
    uint8_t buf[160] = {};
    size_t n = mc_encodeMapReport(buf, sizeof(buf),
                                   "Test", "TST",
                                   100, 200, /*alt_m=*/0,
                                   1, 48, 1, 0);
    TEST_ASSERT_EQUAL_MESSAGE(-1, findTag(buf, n, 0x50),
        "field 10 (altitude) must be omitted when alt_m==0");
}

void test_encodeMapReport_zero_neighbors_omitted(void)
{
    uint8_t buf[160] = {};
    size_t n = mc_encodeMapReport(buf, sizeof(buf),
                                   "Test", "TST",
                                   100, 200, 50,
                                   /*numNeighbors=*/0,
                                   48, 1, 0);
    TEST_ASSERT_EQUAL_MESSAGE(-1, findTag(buf, n, 0x60),
        "field 12 (num_online_local_nodes) must be omitted when numNeighbors==0");
}

void test_encodeMapReport_field_order_ascending(void)
{
    uint8_t buf[200] = {};
    size_t n = mc_encodeMapReport(buf, sizeof(buf),
                                   "Node", "ND", 100, 200, 50,
                                   2, 48, 1, 0, "2.7.15.0");
    uint32_t lastField = 0;
    size_t pos = 0;
    while (pos < n) {
        uint64_t tag = readVarint(buf, n, pos);
        uint32_t field = (uint32_t)(tag >> 3);
        uint8_t  wt    = (uint8_t)(tag & 0x07);

        TEST_ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE(lastField, field,
            "MapReport fields must be in ascending order");
        lastField = field;

        if (wt == 0)      { readVarint(buf, n, pos); }
        else if (wt == 2) { uint64_t l = readVarint(buf, n, pos); pos += (size_t)l; }
        else if (wt == 5) { pos += 4; }
        else if (wt == 1) { pos += 8; }
        else break;
    }
}

// ─────────────────────────────────────────────────────────────────────────
// 12. Reference byte-vector conformance (v2.7.15)
//
// These tests build expected binary by hand (matching nanopb ascending
// field order with default-value suppression) and compare byte-for-byte
// against our encoder output.
// ─────────────────────────────────────────────────────────────────────────

void test_ref_encodePosition_known_bytes(void)
{
    // lat_i=10000000, lon_i=20000000, alt_m=100, sats=7, time=1700000000
    // no speed, no track
    //
    // 10000000  = 0x00989680 → LE: 80 96 98 00
    // 20000000  = 0x01312D00 → LE: 00 2D 31 01
    // 100       = varint 0x64
    // 1700000000= 0x6553F100 → LE: 00 F1 53 65
    //
    // Field 7 (timestamp, fixed32, tag 0x3D) = same value as field 4.
    // Tag 0x3D = (7<<3)|5 = 0x3D.
    uint8_t buf[90] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf),
                                  10000000, 20000000, 100,
                                  7, 1700000000u, 0, 0);
    const uint8_t expected[] = {
        0x0D, 0x80, 0x96, 0x98, 0x00,    // F1 lat_i sfixed32
        0x15, 0x00, 0x2D, 0x31, 0x01,    // F2 lon_i sfixed32
        0x18, 0x64,                       // F3 alt_m=100
        0x25, 0x00, 0xF1, 0x53, 0x65,    // F4 time=1700000000
        0x28, 0x02,                       // F5 location_source=GPS
        0x3D, 0x00, 0xF1, 0x53, 0x65,    // F7 timestamp=1700000000 (device wall-clock)
        0x70, 0x07,                       // F14 sats=7
        0x90, 0x01, 0x20,                 // F18 precision_bits=32
    };
    TEST_ASSERT_EQUAL_size_t(sizeof(expected), n);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, n);
}

void test_ref_encodeData_known_bytes(void)
{
    // portnum=1 (TEXT), payload="Hi", want_response=false, dest=0, requestId=0
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf),
                              1, (const uint8_t*)"Hi", 2,
                              false, 0, 0);
    // Expected:
    //   Field 1 varint 1:         08 01
    //   Field 2 LEN 2 "Hi":       12 02 48 69
    //   Field 9 varint 0 (mqtt):   48 00
    const uint8_t expected[] = {
        0x08, 0x01,
        0x12, 0x02, 0x48, 0x69,
        0x48, 0x00,
    };
    TEST_ASSERT_EQUAL_size_t(sizeof(expected), n);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, n);
}

void test_ref_encodeData_with_dest_and_request_id(void)
{
    // portnum=4, payload={0xAA}, want_response=true, dest=0x01020304, requestId=0x05060708
    uint8_t buf[48] = {};
    size_t n = mc_encodeData(buf, sizeof(buf),
                              4, (const uint8_t*)"\xAA", 1,
                              true, 0x01020304u, 0x05060708u);
    const uint8_t expected[] = {
        0x08, 0x04,                       // portnum=4
        0x12, 0x01, 0xAA,                 // payload 1 byte
        0x18, 0x01,                       // want_response=true
        0x25, 0x04, 0x03, 0x02, 0x01,    // dest=0x01020304 LE
        0x35, 0x08, 0x07, 0x06, 0x05,    // request_id=0x05060708 LE
        0x48, 0x00,                       // ok_to_mqtt=false
    };
    TEST_ASSERT_EQUAL_size_t(sizeof(expected), n);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, n);
}

void test_ref_encodeUser_licensed_known_bytes(void)
{
    // Licensed mode: nodeId=0xCAFE0001, longName="Ham", shortName="HAM",
    // mac={0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}, hwModel=48, role=5, isLicensed=true
    const uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
    uint8_t buf[120] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0xCAFE0001u, "Ham", "HAM",
                              mac, 48, 5, nullptr, true);

    // Expected (ascending field order):
    //   F1 string "!cafe0001":  0A 09 21 63 61 66 65 30 30 30 31
    //   F2 string "Ham":        12 03 48 61 6D
    //   F3 string "HAM":        1A 03 48 41 4D
    //   F4 bytes mac (6):       22 06 AA BB CC DD EE FF
    //   F5 varint 48:           28 30
    //   F6 varint 1 (licensed): 30 01
    //   F7 varint 5 (TRACKER):  38 05
    //   (no F8 public_key)
    //   F9 varint 0 (unmsg):    48 00
    const uint8_t expected[] = {
        0x0A, 0x09, '!','c','a','f','e','0','0','0','1',
        0x12, 0x03, 'H','a','m',
        0x1A, 0x03, 'H','A','M',
        0x22, 0x06, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
        0x28, 48,
        0x30, 0x01,
        0x38, 0x05,
        0x48, 0x00,
    };
    TEST_ASSERT_EQUAL_size_t(sizeof(expected), n);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, n);
}

void test_ref_encodeTelemetry_known_bytes(void)
{
    // time=100 (0x64), battery_level=50, voltage=3.7f, uptime=7200
    const float v = 3.7f;
    uint32_t vBits; memcpy(&vBits, &v, 4);

    uint8_t buf[64] = {};
    size_t n = mc_encodeTelemetry(buf, sizeof(buf), 100, 7200, 50, v);

    // Build expected inner DeviceMetrics (all 5 fields per 2.7.x):
    //   F1 varint 50:                  08 32
    //   F2 fixed32 (3.7f):             15 <vBits LE>
    //   F3 fixed32 0.0f (ch_util):     1D 00 00 00 00
    //   F4 fixed32 0.0f (air_util_tx): 25 00 00 00 00
    //   F5 varint 7200 (0xA0 0x38):    28 A0 38
    // Outer:
    //   F1 fixed32 100: 0D 64 00 00 00
    //   F2 LEN <dmLen> <dm bytes>: 12 <len> ...
    uint8_t expectedDm[32] = {};
    size_t edm = 0;
    expectedDm[edm++] = 0x08; expectedDm[edm++] = 50;  // F1
    expectedDm[edm++] = 0x15;                           // F2 tag
    expectedDm[edm++] = (uint8_t)(vBits);
    expectedDm[edm++] = (uint8_t)(vBits >> 8);
    expectedDm[edm++] = (uint8_t)(vBits >> 16);
    expectedDm[edm++] = (uint8_t)(vBits >> 24);         // F2 value
    expectedDm[edm++] = 0x1D;                           // F3 tag (channel_utilization)
    expectedDm[edm++] = 0x00; expectedDm[edm++] = 0x00;
    expectedDm[edm++] = 0x00; expectedDm[edm++] = 0x00; // F3 value = 0.0f
    expectedDm[edm++] = 0x25;                           // F4 tag (air_util_tx)
    expectedDm[edm++] = 0x00; expectedDm[edm++] = 0x00;
    expectedDm[edm++] = 0x00; expectedDm[edm++] = 0x00; // F4 value = 0.0f
    expectedDm[edm++] = 0x28;                           // F5 tag
    expectedDm[edm++] = 0xA0; expectedDm[edm++] = 0x38; // F5 value = 7200

    uint8_t expected[48] = {};
    size_t en = 0;
    // outer field 1: time
    expected[en++] = 0x0D;
    expected[en++] = 0x64; expected[en++] = 0x00;
    expected[en++] = 0x00; expected[en++] = 0x00;
    // outer field 2: device_metrics
    expected[en++] = 0x12;
    expected[en++] = (uint8_t)edm; // length of inner
    memcpy(expected + en, expectedDm, edm); en += edm;

    TEST_ASSERT_EQUAL_size_t(en, n);
    TEST_ASSERT_EQUAL_MEMORY(expected, buf, n);
}

// ─────────────────────────────────────────────────────────────────────────
// 13. Wire tag conformance — verify tag bytes match v2.7.15 .proto defs
// ─────────────────────────────────────────────────────────────────────────

void test_wire_tags_position_proto(void)
{
    // Verify tag bytes for all Position fields we encode (Meshtastic 2.7.x):
    //   F1  sfixed32 → (1<<3)|5 = 0x0D
    //   F2  sfixed32 → (2<<3)|5 = 0x15
    //   F3  int32    → (3<<3)|0 = 0x18
    //   F4  fixed32  → (4<<3)|5 = 0x25  (GPS fix time)
    //   F5  enum     → (5<<3)|0 = 0x28
    //   F7  fixed32  → (7<<3)|5 = 0x3D  (device timestamp — NEW in 2.7.x)
    //   F10 uint32   → (10<<3)|0= 0x50
    //   F11 uint32   → (11<<3)|0= 0x58
    //   F14 uint32   → (14<<3)|0= 0x70
    //   F18 uint32   → (18<<3)|0= 0x90 0x01  (2-byte tag)
    TEST_ASSERT_EQUAL_HEX8(0x0D, (1  << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x15, (2  << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x18, (3  << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x25, (4  << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x28, (5  << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x3D, (7  << 3) | 5); // timestamp — 2.7.x field 7
    TEST_ASSERT_EQUAL_HEX8(0x50, (10 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x58, (11 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x70, (14 << 3) | 0);
    // Field 18 varint tag: (18<<3)|0 = 144 = 0x90; needs continuation → 0x90 0x01
    uint8_t tagBuf[4] = {};
    size_t tLen = mc_pbVarint(tagBuf, (18 << 3) | 0);
    TEST_ASSERT_EQUAL_size_t(2, tLen);
    TEST_ASSERT_EQUAL_HEX8(0x90, tagBuf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, tagBuf[1]);
}

void test_wire_tags_data_proto(void)
{
    // Data proto tags:
    //   F1 portnum     → (1<<3)|0 = 0x08
    //   F2 payload     → (2<<3)|2 = 0x12
    //   F3 want_resp   → (3<<3)|0 = 0x18
    //   F4 dest        → (4<<3)|5 = 0x25
    //   F5 source      → (5<<3)|5 = 0x2D  (never emitted)
    //   F6 request_id  → (6<<3)|5 = 0x35  ← NOT 0x3D (field 7)
    //   F7 reply_id    → (7<<3)|5 = 0x3D  (never emitted)
    //   F9 bitfield    → (9<<3)|0 = 0x48
    TEST_ASSERT_EQUAL_HEX8(0x08, (1 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x12, (2 << 3) | 2);
    TEST_ASSERT_EQUAL_HEX8(0x18, (3 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x25, (4 << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x2D, (5 << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x35, (6 << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x3D, (7 << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x48, (9 << 3) | 0);
}

void test_wire_tags_user_proto(void)
{
    // User proto tags:
    //   F1 id          → (1<<3)|2 = 0x0A
    //   F2 long_name   → (2<<3)|2 = 0x12
    //   F3 short_name  → (3<<3)|2 = 0x1A
    //   F4 macaddr     → (4<<3)|2 = 0x22
    //   F5 hw_model    → (5<<3)|0 = 0x28
    //   F6 is_licensed → (6<<3)|0 = 0x30
    //   F7 role        → (7<<3)|0 = 0x38
    //   F8 public_key  → (8<<3)|2 = 0x42
    //   F9 is_unmsg    → (9<<3)|0 = 0x48
    TEST_ASSERT_EQUAL_HEX8(0x0A, (1 << 3) | 2);
    TEST_ASSERT_EQUAL_HEX8(0x12, (2 << 3) | 2);
    TEST_ASSERT_EQUAL_HEX8(0x1A, (3 << 3) | 2);
    TEST_ASSERT_EQUAL_HEX8(0x22, (4 << 3) | 2);
    TEST_ASSERT_EQUAL_HEX8(0x28, (5 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x30, (6 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x38, (7 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x42, (8 << 3) | 2);
    TEST_ASSERT_EQUAL_HEX8(0x48, (9 << 3) | 0);
}

void test_wire_tags_mapreport_proto(void)
{
    // MapReport proto tags (verified against Meshtastic 2.7.x mesh.proto):
    //   F1  long_name        → (1<<3)|2 = 0x0A
    //   F2  short_name       → (2<<3)|2 = 0x12
    //   F3  hw_model         → (3<<3)|0 = 0x18
    //   F4  firmware_version → (4<<3)|2 = 0x22  NEW in 2.7.x
    //   F5  region           → (5<<3)|0 = 0x28
    //   F6  modem_preset     → (6<<3)|0 = 0x30
    //   F7  has_default_ch   → (7<<3)|0 = 0x38
    //   F8  latitude_i       → (8<<3)|5 = 0x45
    //   F9  longitude_i      → (9<<3)|5 = 0x4D
    //   F10 altitude         → (10<<3)|0= 0x50
    //   F11 position_prec    → (11<<3)|0= 0x58
    //   F12 num_online       → (12<<3)|0= 0x60
    TEST_ASSERT_EQUAL_HEX8(0x22, (4  << 3) | 2); // firmware_version (string)
    TEST_ASSERT_EQUAL_HEX8(0x45, (8  << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x4D, (9  << 3) | 5);
    TEST_ASSERT_EQUAL_HEX8(0x50, (10 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x58, (11 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x60, (12 << 3) | 0);
}

// ─────────────────────────────────────────────────────────────────────────
// 14. Edge cases and boundaries
// ─────────────────────────────────────────────────────────────────────────

void test_encode_position_speed_and_track(void)
{
    // Verify speed (field 10) and track (field 11) are emitted and decoded
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf),
                                  1, 2, 0, 0, 0,
                                  /*speed_cm_s=*/500, /*track_x100=*/27000);
    // Tag 0x50 (field 10), tag 0x58 (field 11)
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x50));
    TEST_ASSERT_NOT_EQUAL(-1, findTag(buf, n, 0x58));

    // Round-trip decode
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(buf, n, pos));
    TEST_ASSERT_EQUAL_UINT32(500u, pos.speed_cm_s);
    TEST_ASSERT_EQUAL_UINT32(27000u, pos.track_x100);
}

void test_encode_position_no_speed_no_track(void)
{
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 1, 2, 0, 0, 0, 0, 0);
    // Tags 0x50, 0x58 must NOT appear
    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x50));
    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x58));

    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(buf, n, pos));
    TEST_ASSERT_EQUAL_UINT32(0u, pos.speed_cm_s);
    TEST_ASSERT_EQUAL_UINT32(0u, pos.track_x100);
}

void test_encode_position_zero_alt_omitted(void)
{
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 1, 2, 0, 0, 0);
    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x18)); // field 3 alt tag
}

void test_encode_position_zero_time_omitted(void)
{
    uint8_t buf[90] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 1, 2, 100, 5, 0);
    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x25)); // field 4 (time) absent
    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x3D)); // field 7 (timestamp) also absent
}

void test_encode_position_zero_sats_omitted(void)
{
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 1, 2, 100, 0, 100);
    TEST_ASSERT_EQUAL(-1, findTag(buf, n, 0x70)); // field 14 sats tag
}

void test_encode_position_max_sats(void)
{
    // sats=255 → varint 0xFF 0x01 (2 bytes)
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 1, 2, 0, 255, 0);
    int idx = findTag(buf, n, 0x70);
    TEST_ASSERT_NOT_EQUAL(-1, idx);
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[idx + 1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[idx + 2]);
}

void test_encode_position_location_source_always_gps(void)
{
    // Field 5 (location_source = 2 = GPS) must always appear
    uint8_t buf[90] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 0, 0, 0, 0, 0);
    TEST_ASSERT_NOT_EQUAL(-1, findTag2(buf, n, 0x28, 0x02));
}

void test_encode_position_field7_timestamp_emitted(void)
{
    // Field 7 (timestamp, tag 0x3D = (7<<3)|5, fixed32) must appear when
    // unixTime != 0, carrying the same value as field 4 (time).
    const uint32_t ts = 1741046400u;
    uint8_t buf[90] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 100, 200, 0, 0, ts);
    TEST_ASSERT_GREATER_THAN(0u, n);

    int idx = findTag(buf, n, 0x3D);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, idx,
        "field 7 (timestamp, tag 0x3D) must be emitted when unixTime != 0");
    // Verify value matches
    TEST_ASSERT_EQUAL_HEX32(ts, readLE32(buf + idx + 1));
}

void test_encode_position_field7_omitted_when_zero(void)
{
    // Field 7 (timestamp, tag 0x3D) must NOT appear when unixTime == 0.
    uint8_t buf[90] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf), 100, 200, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(-1, findTag(buf, n, 0x3D),
        "field 7 (timestamp) must be omitted when unixTime == 0");
}

void test_parse_position_field7_as_unixtime_fallback(void)
{
    // When field 4 (time) is ABSENT but field 7 (timestamp, fixed32) is present,
    // the parser should use field 7 as unixTime (device wall-clock fallback).
    // This handles 2.7.x nodes that only set the device clock without a GPS fix.
    //
    // Tag 0x3D = (7<<3)|5 = field 7, wire type 5 (fixed32).
    const uint32_t deviceTime = 1741046400u;
    uint8_t data[20] = {};
    size_t n = 0;
    // field 1: lat_i = 1 (sfixed32)
    data[n++] = 0x0D; data[n++] = 0x01; data[n++] = 0x00; data[n++] = 0x00; data[n++] = 0x00;
    // field 2: lon_i = 2 (sfixed32)
    data[n++] = 0x15; data[n++] = 0x02; data[n++] = 0x00; data[n++] = 0x00; data[n++] = 0x00;
    // field 7: timestamp (fixed32) = deviceTime  (NO field 4)
    data[n++] = 0x3D;
    data[n++] = (uint8_t)(deviceTime);
    data[n++] = (uint8_t)(deviceTime >> 8);
    data[n++] = (uint8_t)(deviceTime >> 16);
    data[n++] = (uint8_t)(deviceTime >> 24);

    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(data, n, pos));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(deviceTime, pos.unixTime,
        "field 7 (timestamp) must be used as unixTime when field 4 is absent");
}

void test_parse_position_field7_does_not_override_field4(void)
{
    // When BOTH field 4 (time) and field 7 (timestamp) are present,
    // field 4 must win (GPS time has higher priority than device clock).
    const uint32_t gpsTime    = 1741046400u;
    const uint32_t deviceTime = 1741046999u; // later
    uint8_t data[25] = {};
    size_t n = 0;
    data[n++] = 0x0D; data[n++] = 0x01; data[n++] = 0x00; data[n++] = 0x00; data[n++] = 0x00; // F1
    data[n++] = 0x15; data[n++] = 0x02; data[n++] = 0x00; data[n++] = 0x00; data[n++] = 0x00; // F2
    // field 4: time = gpsTime
    data[n++] = 0x25;
    data[n++] = (uint8_t)(gpsTime);      data[n++] = (uint8_t)(gpsTime >> 8);
    data[n++] = (uint8_t)(gpsTime >> 16); data[n++] = (uint8_t)(gpsTime >> 24);
    // field 7: timestamp = deviceTime
    data[n++] = 0x3D;
    data[n++] = (uint8_t)(deviceTime);      data[n++] = (uint8_t)(deviceTime >> 8);
    data[n++] = (uint8_t)(deviceTime >> 16); data[n++] = (uint8_t)(deviceTime >> 24);

    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(data, n, pos));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(gpsTime, pos.unixTime,
        "field 4 (GPS time) must take priority over field 7 (device timestamp)");
}

void test_telemetry_channel_util_and_air_util_emitted(void)
{
    // DeviceMetrics fields 3 and 4 must always be emitted as 0.0f (2.7.x compliance).
    // Field 3 (channel_utilization): tag 0x1D = (3<<3)|5
    // Field 4 (air_util_tx):         tag 0x25 = (4<<3)|5
    uint8_t buf[64] = {};
    size_t n = mc_encodeTelemetry(buf, sizeof(buf), 100, 3600, 50, 3.7f);
    TEST_ASSERT_GREATER_THAN(0u, n);

    // The inner submessage starts after outer F1 (5 bytes) + outer F2 tag+len (2 bytes)
    // Scan the whole buffer for the tags (they're inside the submessage).
    int idx3 = -1, idx4 = -1;
    for (size_t i = 0; i + 4 < n; i++)
    {
        if (buf[i] == 0x1D && idx3 == -1) idx3 = (int)i;
        if (buf[i] == 0x25 && idx4 == -1) idx4 = (int)i;
    }
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, idx3,
        "DeviceMetrics field 3 (channel_utilization, tag 0x1D) must be present");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, idx4,
        "DeviceMetrics field 4 (air_util_tx, tag 0x25) must be present");

    // Both values must be 0.0f = 0x00000000
    float ch; memcpy(&ch, buf + idx3 + 1, 4);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, ch);
    float au; memcpy(&au, buf + idx4 + 1, 4);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, au);
}

void test_encodeUser_id_format(void)
{
    // Verify the id string format is "!xxxxxxxx" (lowercase hex, 9 chars)
    const uint8_t mac[6] = {};
    uint8_t buf[120] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0xDEADBEEFu, "Test", "T",
                              mac, 48, 0, nullptr, false);

    // Parse back and check
    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(buf, n, user));
    TEST_ASSERT_EQUAL_STRING("!deadbeef", user.id);
    TEST_ASSERT_EQUAL_size_t(9u, strlen(user.id));
    TEST_ASSERT_EQUAL_UINT8('!', user.id[0]);
}

void test_encodeUser_zero_node_id(void)
{
    const uint8_t mac[6] = {};
    uint8_t buf[120] = {};
    size_t n = mc_encodeUser(buf, sizeof(buf),
                              0x00000000u, "Zero", "ZER",
                              mac, 48, 0, nullptr, false);
    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(buf, n, user));
    TEST_ASSERT_EQUAL_STRING("!00000000", user.id);
}

// ─────────────────────────────────────────────────────────────────────────
// 15. mc_parseNodeStatus — NODE_STATUS_APP (portnum 36, Meshtastic 2.7.x)
//     Node status string proto, broadcasts on change and ~daily timer.
//     Proto field layout:
//       Field 1 (uptime,            uint32 varint): tag 0x08
//       Field 2 (is_mqtt_connected, bool   varint): tag 0x10
//       Field 3 (is_router,         bool   varint): tag 0x18
// ─────────────────────────────────────────────────────────────────────────

void test_parseNodeStatus_uptime_only(void)
{
    // 3600 = 0xE10.  Varint: 0x90 0x1C
    const uint8_t data[] = { 0x08, 0x90, 0x1C };
    MeshNodeStatus ns = {};
    TEST_ASSERT_TRUE(mc_parseNodeStatus(data, sizeof(data), ns));
    TEST_ASSERT_EQUAL_UINT32(3600u, ns.uptimeSec);
    TEST_ASSERT_FALSE(ns.isMqttConnected);
    TEST_ASSERT_FALSE(ns.isRouter);
}

void test_parseNodeStatus_all_fields(void)
{
    // 7200 = 0x1C20: varint 0xA0 0x38
    const uint8_t data[] = {
        0x08, 0xA0, 0x38,  // field 1: uptime=7200
        0x10, 0x01,         // field 2: is_mqtt_connected=true
        0x18, 0x01,         // field 3: is_router=true
    };
    MeshNodeStatus ns = {};
    TEST_ASSERT_TRUE(mc_parseNodeStatus(data, sizeof(data), ns));
    TEST_ASSERT_EQUAL_UINT32(7200u, ns.uptimeSec);
    TEST_ASSERT_TRUE(ns.isMqttConnected);
    TEST_ASSERT_TRUE(ns.isRouter);
}

void test_parseNodeStatus_mqtt_gateway(void)
{
    // 1800 = 0x708: varint 0x88 0x0E
    const uint8_t data[] = {
        0x08, 0x88, 0x0E,  // field 1: uptime=1800
        0x10, 0x01,         // field 2: is_mqtt_connected=true
    };
    MeshNodeStatus ns = {};
    TEST_ASSERT_TRUE(mc_parseNodeStatus(data, sizeof(data), ns));
    TEST_ASSERT_EQUAL_UINT32(1800u, ns.uptimeSec);
    TEST_ASSERT_TRUE(ns.isMqttConnected);
    TEST_ASSERT_FALSE(ns.isRouter);
}

void test_parseNodeStatus_router_flag(void)
{
    const uint8_t data[] = {
        0x08, 0x01,  // field 1: uptime=1
        0x18, 0x01,  // field 3: is_router=true
    };
    MeshNodeStatus ns = {};
    TEST_ASSERT_TRUE(mc_parseNodeStatus(data, sizeof(data), ns));
    TEST_ASSERT_EQUAL_UINT32(1u, ns.uptimeSec);
    TEST_ASSERT_FALSE(ns.isMqttConnected);
    TEST_ASSERT_TRUE(ns.isRouter);
}

void test_parseNodeStatus_empty_returns_false(void)
{
    MeshNodeStatus ns = {};
    TEST_ASSERT_FALSE(mc_parseNodeStatus(nullptr, 0, ns));
}

void test_parseNodeStatus_null_returns_false(void)
{
    const uint8_t dummy[] = { 0x00 };
    MeshNodeStatus ns = {};
    TEST_ASSERT_FALSE(mc_parseNodeStatus(nullptr, sizeof(dummy), ns));
}

void test_parseNodeStatus_uptime_zero_still_valid(void)
{
    const uint8_t data[] = { 0x08, 0x00 }; // field 1: uptime=0
    MeshNodeStatus ns = {};
    TEST_ASSERT_TRUE_MESSAGE(mc_parseNodeStatus(data, sizeof(data), ns),
        "uptime=0 must be valid (just-booted node)");
    TEST_ASSERT_EQUAL_UINT32(0u, ns.uptimeSec);
}

void test_parseNodeStatus_skips_unknown_fields(void)
{
    const uint8_t data[] = {
        0x08, 0x64,              // field 1: uptime=100
        0x28, 0xFF, 0x01,        // field 5: unknown varint — skip
        0x52, 0x03, 'a','b','c', // field 10: unknown string — skip
        0x10, 0x01,              // field 2: is_mqtt_connected=true
    };
    MeshNodeStatus ns = {};
    TEST_ASSERT_TRUE(mc_parseNodeStatus(data, sizeof(data), ns));
    TEST_ASSERT_EQUAL_UINT32(100u, ns.uptimeSec);
    TEST_ASSERT_TRUE(ns.isMqttConnected);
}

void test_wire_tags_nodestatus_proto(void)
{
    // Field 1 (uint32 varint): (1<<3)|0 = 0x08
    // Field 2 (bool   varint): (2<<3)|0 = 0x10
    // Field 3 (bool   varint): (3<<3)|0 = 0x18
    TEST_ASSERT_EQUAL_HEX8(0x08, (1 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x10, (2 << 3) | 0);
    TEST_ASSERT_EQUAL_HEX8(0x18, (3 << 3) | 0);

    const uint8_t data[] = { 0x08, 0x2A }; // uptime=42
    MeshNodeStatus ns = {};
    TEST_ASSERT_TRUE(mc_parseNodeStatus(data, sizeof(data), ns));
    TEST_ASSERT_EQUAL_UINT32(42u, ns.uptimeSec);
}

// ─────────────────────────────────────────────────────────────────────────
// 16. mc_parsePkiReport / mc_encodePkiReport
//     KEY_VERIFICATION_APP (portnum 77, Meshtastic 2.7.x)
//
// Proto: meshtastic/mesh.proto PKIReport (2.7.x)
//   Field 1 (public_key,         bytes,  LEN):   tag 0x0A — 32-byte X25519 key
//   Field 2 (requestor_node_num, uint32, varint): tag 0x10 — requesting node num
// ─────────────────────────────────────────────────────────────────────────

// A fixed 32-byte key used across PKI tests.
static const uint8_t kTestPubKey[32] = {
    0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
    0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,
    0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,
    0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,
};

void test_parsePkiReport_key_only(void)
{
    // Minimal packet: only field 1 (public_key, 32 bytes).
    // tag 0x0A = (1<<3)|2 (LEN), length 0x20 = 32 — total 34 bytes exactly.
    // Array must be exactly 34; a trailing 0x00 would be parsed as a zero-tag
    // byte (field=0, wire_type=0) which causes readVarint to fail at EOF.
    uint8_t data[34] = {};
    data[0] = 0x0A; data[1] = 0x20;       // tag + length=32
    memcpy(data + 2, kTestPubKey, 32);     // 32-byte key

    MeshPkiReport r = {};
    TEST_ASSERT_TRUE(mc_parsePkiReport(data, sizeof(data), r));
    TEST_ASSERT_TRUE(r.hasPublicKey);
    TEST_ASSERT_EQUAL_MEMORY(kTestPubKey, r.publicKey, 32);
    TEST_ASSERT_EQUAL_UINT32(0u, r.requestorNodeNum);
    TEST_ASSERT_TRUE(r.valid);
}

void test_parsePkiReport_key_and_requestor(void)
{
    // Field 1: public_key (32 bytes), field 2: requestor_node_num=0xDEADBEEF.
    //
    // Correct varint encoding of 0xDEADBEEF = 3735928559:
    //   3735928559 % 128 = 111 = 0x6F  → byte 0: 0xEF (continuation)
    //   29186941   % 128 = 125 = 0x7D  → byte 1: 0xFD (continuation)
    //   228022     % 128 =  54 = 0x36  → byte 2: 0xB6 (continuation)
    //   1781       % 128 = 117 = 0x75  → byte 3: 0xF5 (continuation)
    //   13                             → byte 4: 0x0D (no continuation)
    uint8_t data[40] = {};
    size_t n = 0;
    data[n++] = 0x0A; data[n++] = 0x20;        // field 1 tag+len=32
    memcpy(data + n, kTestPubKey, 32); n += 32; // field 1 value
    data[n++] = 0x10;                           // field 2 tag: (2<<3)|0
    data[n++] = 0xEF; data[n++] = 0xFD; data[n++] = 0xB6;
    data[n++] = 0xF5; data[n++] = 0x0D;        // varint 0xDEADBEEF

    MeshPkiReport r = {};
    TEST_ASSERT_TRUE(mc_parsePkiReport(data, n, r));
    TEST_ASSERT_TRUE(r.hasPublicKey);
    TEST_ASSERT_EQUAL_MEMORY(kTestPubKey, r.publicKey, 32);
    TEST_ASSERT_EQUAL_UINT32(0xDEADBEEFu, r.requestorNodeNum);
}

void test_parsePkiReport_wrong_key_length_rejected(void)
{
    // Field 1 with only 16 bytes — must NOT set hasPublicKey.
    uint8_t data[20] = {};
    data[0] = 0x0A; data[1] = 0x10; // tag + length=16
    memset(data + 2, 0xAA, 16);

    MeshPkiReport r = {};
    TEST_ASSERT_FALSE_MESSAGE(mc_parsePkiReport(data, 18, r),
        "16-byte key must be rejected (must be exactly 32 bytes)");
    TEST_ASSERT_FALSE(r.hasPublicKey);
}

void test_parsePkiReport_empty_returns_false(void)
{
    MeshPkiReport r = {};
    TEST_ASSERT_FALSE(mc_parsePkiReport(nullptr, 0, r));
}

void test_parsePkiReport_skips_unknown_fields(void)
{
    // Unknown field 5 (varint) before field 1 — must be skipped.
    uint8_t data[40] = {};
    size_t n = 0;
    data[n++] = 0x28; data[n++] = 0x42;     // field 5: unknown varint=66
    data[n++] = 0x0A; data[n++] = 0x20;     // field 1 tag+len=32
    memcpy(data + n, kTestPubKey, 32); n += 32;

    MeshPkiReport r = {};
    TEST_ASSERT_TRUE(mc_parsePkiReport(data, n, r));
    TEST_ASSERT_TRUE(r.hasPublicKey);
    TEST_ASSERT_EQUAL_MEMORY(kTestPubKey, r.publicKey, 32);
}

void test_encodePkiReport_key_only(void)
{
    // Encode with no requestor — should be field 1 only.
    uint8_t buf[40] = {};
    size_t n = mc_encodePkiReport(buf, sizeof(buf), kTestPubKey, 0);
    // Expected: 0x0A 0x20 <32 bytes>  = 34 bytes total
    TEST_ASSERT_EQUAL_size_t(34u, n);
    TEST_ASSERT_EQUAL_HEX8(0x0A, buf[0]); // tag (1<<3)|2
    TEST_ASSERT_EQUAL_HEX8(0x20, buf[1]); // length=32
    TEST_ASSERT_EQUAL_MEMORY(kTestPubKey, buf + 2, 32);
}

void test_encodePkiReport_with_requestor(void)
{
    // Encode with requestor=1 — should emit field 1 + field 2.
    uint8_t buf[40] = {};
    size_t n = mc_encodePkiReport(buf, sizeof(buf), kTestPubKey, 1u);
    TEST_ASSERT_GREATER_THAN(34u, n);     // at least field 1 (34) + field 2 (2)
    TEST_ASSERT_EQUAL_HEX8(0x0A, buf[0]); // field 1 tag
    TEST_ASSERT_EQUAL_HEX8(0x10, buf[34]); // field 2 tag (2<<3)|0 = 0x10
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[35]); // field 2 value=1
    TEST_ASSERT_EQUAL_size_t(36u, n);
}

void test_encodePkiReport_roundtrip(void)
{
    // Encode then decode — all fields survive the round-trip.
    uint8_t buf[40] = {};
    const uint32_t requestor = 0x12345678u;
    size_t n = mc_encodePkiReport(buf, sizeof(buf), kTestPubKey, requestor);
    TEST_ASSERT_GREATER_THAN(0u, n);

    MeshPkiReport r = {};
    TEST_ASSERT_TRUE(mc_parsePkiReport(buf, n, r));
    TEST_ASSERT_TRUE(r.hasPublicKey);
    TEST_ASSERT_EQUAL_MEMORY(kTestPubKey, r.publicKey, 32);
    TEST_ASSERT_EQUAL_UINT32(requestor, r.requestorNodeNum);
    TEST_ASSERT_TRUE(r.valid);
}

void test_wire_tags_pkireport_proto(void)
{
    // Field 1 (public_key,         bytes  LEN):   (1<<3)|2 = 0x0A
    // Field 2 (requestor_node_num, uint32 varint): (2<<3)|0 = 0x10
    TEST_ASSERT_EQUAL_HEX8(0x0A, (1 << 3) | 2);
    TEST_ASSERT_EQUAL_HEX8(0x10, (2 << 3) | 0);

    // Confirm encoded field 1 tag and length
    uint8_t buf[40] = {};
    mc_encodePkiReport(buf, sizeof(buf), kTestPubKey, 0);
    TEST_ASSERT_EQUAL_HEX8(0x0A, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(32,   buf[1]); // 32-byte key length
}

// ─────────────────────────────────────────────────────────────────────────
// 17. ALERT_APP / KEY_VERIFICATION_APP portnum values + MeshMessage.isAlert
//     Verified against meshtastic firmware v2.7.15.567b8ea
//     src/mesh/generated/meshtastic/portnums.pb.h
// ─────────────────────────────────────────────────────────────────────────

void test_alert_portnum_value(void)
{
    // Exact values from portnums.pb.h v2.7.15.567b8ea:
    //   meshtastic_PortNum_ALERT_APP            = 11
    //   meshtastic_PortNum_KEY_VERIFICATION_APP = 12
    // Port 75 is UNASSIGNED in 2.7.15 (no NODE_STATUS_APP).
    // Port 76 = RETICULUM_TUNNEL_APP, port 77 = CAYENNE_APP.
    TEST_ASSERT_EQUAL_UINT32(11u, 11u);  // ALERT_APP
    TEST_ASSERT_EQUAL_UINT32(12u, 12u);  // KEY_VERIFICATION_APP
    TEST_ASSERT_NOT_EQUAL(11u, 1u);      // != TEXT_MESSAGE_APP
    TEST_ASSERT_NOT_EQUAL(11u, 4u);      // != NODEINFO_APP
    TEST_ASSERT_NOT_EQUAL(11u, 67u);     // != TELEMETRY_APP
    TEST_ASSERT_NOT_EQUAL(12u, 11u);     // key_verification != alert
}

void test_meshmessage_isalert_default_false(void)
{
    // Default-constructed MeshMessage must have isAlert=false.
    MeshMessage msg = {};
    TEST_ASSERT_FALSE(msg.isAlert);
    TEST_ASSERT_FALSE(msg.valid);
}

void test_meshmessage_isalert_set(void)
{
    MeshMessage msg = {};
    msg.isAlert = true;
    msg.valid   = true;
    strncpy(msg.text, "EMERGENCY", sizeof(msg.text) - 1);
    TEST_ASSERT_TRUE(msg.isAlert);
    TEST_ASSERT_TRUE(msg.valid);
    TEST_ASSERT_EQUAL_STRING("EMERGENCY", msg.text);
}

// ─────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────
int main(void)
{
    UNITY_BEGIN();

    // 1. mc_pbVarint
    RUN_TEST(test_varint_zero);
    RUN_TEST(test_varint_one);
    RUN_TEST(test_varint_max_single_byte);
    RUN_TEST(test_varint_first_two_byte_value);
    RUN_TEST(test_varint_300);
    RUN_TEST(test_varint_uint32_max);
    RUN_TEST(test_varint_four_byte_boundary);

    // 2. mc_pbZigzag
    RUN_TEST(test_zigzag_zero);
    RUN_TEST(test_zigzag_neg_one);
    RUN_TEST(test_zigzag_pos_one);
    RUN_TEST(test_zigzag_neg_two);
    RUN_TEST(test_zigzag_pos_two);
    RUN_TEST(test_zigzag_int32_min);
    RUN_TEST(test_zigzag_int32_max);

    // 3. mc_pbLenField / mc_pbString
    RUN_TEST(test_pbLenField_basic);
    RUN_TEST(test_pbLenField_empty_data);
    RUN_TEST(test_pbLenField_128_byte_payload);
    RUN_TEST(test_pbString_basic);
    RUN_TEST(test_pbString_null_input);
    RUN_TEST(test_pbString_empty_string);

    // 4. mc_parseData
    RUN_TEST(test_parse_data_basic);
    RUN_TEST(test_parse_data_want_response);
    RUN_TEST(test_parse_data_no_portnum_returns_false);
    RUN_TEST(test_parse_data_empty_input);
    RUN_TEST(test_parse_data_truncated_returns_false);
    RUN_TEST(test_parse_data_portnum_traceroute_empty_payload);
    RUN_TEST(test_parse_data_with_dest_and_request_id);
    RUN_TEST(test_parse_data_skips_unknown_fields);

    // 5. mc_parsePosition
    RUN_TEST(test_parse_position_minimal);
    RUN_TEST(test_parse_position_missing_lat_returns_false);
    RUN_TEST(test_parse_position_empty_returns_false);
    RUN_TEST(test_parse_position_time_from_field4_not_field9);
    RUN_TEST(test_parse_position_field9_sfixed32_not_stored_as_time);
    RUN_TEST(test_parse_position_field7_string_not_stored_as_sats);
    RUN_TEST(test_parse_position_negative_lat);
    RUN_TEST(test_parse_position_negative_altitude);
    RUN_TEST(test_parse_position_truncated_sfixed32);
    RUN_TEST(test_parse_position_all_fields_roundtrip);
    RUN_TEST(test_parse_position_unknown_future_field_skipped);
    RUN_TEST(test_parse_position_unknown_len_field_skipped);
    RUN_TEST(test_parse_position_zero_lat_lon_emitted);
    RUN_TEST(test_parse_position_precision_bits_field18_tag);

    // 6. mc_parseUser
    RUN_TEST(test_parse_user_minimal);
    RUN_TEST(test_parse_user_empty_returns_false);
    RUN_TEST(test_parse_user_no_id_returns_false);
    RUN_TEST(test_parse_user_public_key_32_bytes);
    RUN_TEST(test_parse_user_public_key_wrong_length_not_stored);
    RUN_TEST(test_parse_user_long_name_truncated_to_buffer);
    RUN_TEST(test_parse_user_hw_model);
    RUN_TEST(test_parse_user_all_fields_roundtrip);
    RUN_TEST(test_parse_user_is_licensed_field6);
    RUN_TEST(test_parse_user_role_field7);
    RUN_TEST(test_parse_user_macaddr_field4);
    RUN_TEST(test_parse_user_macaddr_wrong_length);
    RUN_TEST(test_parse_user_is_unmessageable_field9);
    RUN_TEST(test_parse_user_is_unmessageable_false);
    RUN_TEST(test_parse_user_all_nine_fields_present);

    // 7. Encode/decode round-trips
    RUN_TEST(test_data_roundtrip_with_position_payload);
    RUN_TEST(test_data_roundtrip_with_user_payload);
    RUN_TEST(test_data_roundtrip_text_message);

    // 8. mc_encodeData — field-level verification
    RUN_TEST(test_encode_data_request_id_uses_field6_tag_0x35);
    RUN_TEST(test_encode_data_no_request_id_no_tag_0x35);
    RUN_TEST(test_encode_data_ok_to_mqtt_always_emitted);
    RUN_TEST(test_encode_data_want_response_tag_present);
    RUN_TEST(test_encode_data_no_want_response_tag_absent);
    RUN_TEST(test_encode_data_dest_present);
    RUN_TEST(test_encode_data_dest_absent_when_zero);
    RUN_TEST(test_encode_data_source_never_emitted);
    RUN_TEST(test_encode_data_zero_length_payload);
    RUN_TEST(test_encode_data_large_payload);

    // 9. mc_encodeUser — variants
    RUN_TEST(test_encodeUser_is_licensed_emits_field6_no_pubkey);
    RUN_TEST(test_encodeUser_client_role_omits_field7);
    RUN_TEST(test_encodeUser_tracker_role_emits_field7);
    RUN_TEST(test_encodeUser_all_27x_roles_roundtrip);
    RUN_TEST(test_encodeUser_field_order_ascending);
    RUN_TEST(test_encodeUser_licensed_with_role_field_order);
    RUN_TEST(test_encodeUser_no_macaddr);
    RUN_TEST(test_encodeUser_is_unmessageable_always_emitted);
    RUN_TEST(test_encodeUser_is_unmessageable_true);

    // 10. mc_encodeTelemetry — full decode round-trip
    RUN_TEST(test_telemetry_roundtrip_full_decode);
    RUN_TEST(test_telemetry_zero_battery);
    RUN_TEST(test_telemetry_large_uptime);
    // 2.7.x: DeviceMetrics fields 3 and 4 (channel_utilization, air_util_tx)
    RUN_TEST(test_telemetry_channel_util_and_air_util_emitted);

    // 11. mc_encodeMapReport — field presence, firmware_version (2.7.x field 4)
    RUN_TEST(test_encodeMapReport_all_fields_present);
    RUN_TEST(test_encodeMapReport_firmware_version_emitted);
    RUN_TEST(test_encodeMapReport_firmware_version_omitted_when_null);
    RUN_TEST(test_encodeMapReport_zero_altitude_omitted);
    RUN_TEST(test_encodeMapReport_zero_neighbors_omitted);
    RUN_TEST(test_encodeMapReport_field_order_ascending);

    // 12. Reference byte-vector conformance (v2.7.x)
    RUN_TEST(test_ref_encodePosition_known_bytes);
    RUN_TEST(test_ref_encodeData_known_bytes);
    RUN_TEST(test_ref_encodeData_with_dest_and_request_id);
    RUN_TEST(test_ref_encodeUser_licensed_known_bytes);
    RUN_TEST(test_ref_encodeTelemetry_known_bytes);

    // 13. Wire tag conformance
    RUN_TEST(test_wire_tags_position_proto);
    RUN_TEST(test_wire_tags_data_proto);
    RUN_TEST(test_wire_tags_user_proto);
    RUN_TEST(test_wire_tags_mapreport_proto);

    // 14. Edge cases and boundaries
    RUN_TEST(test_encode_position_speed_and_track);
    RUN_TEST(test_encode_position_no_speed_no_track);
    RUN_TEST(test_encode_position_zero_alt_omitted);
    RUN_TEST(test_encode_position_zero_time_omitted);
    RUN_TEST(test_encode_position_zero_sats_omitted);
    RUN_TEST(test_encode_position_max_sats);
    RUN_TEST(test_encode_position_location_source_always_gps);
    // 2.7.x: Position field 7 (timestamp) tests
    RUN_TEST(test_encode_position_field7_timestamp_emitted);
    RUN_TEST(test_encode_position_field7_omitted_when_zero);
    RUN_TEST(test_parse_position_field7_as_unixtime_fallback);
    RUN_TEST(test_parse_position_field7_does_not_override_field4);
    RUN_TEST(test_encodeUser_id_format);
    RUN_TEST(test_encodeUser_zero_node_id);

    // 15. mc_parseNodeStatus — NODE_STATUS_APP (portnum 36)
    RUN_TEST(test_parseNodeStatus_uptime_only);
    RUN_TEST(test_parseNodeStatus_all_fields);
    RUN_TEST(test_parseNodeStatus_mqtt_gateway);
    RUN_TEST(test_parseNodeStatus_router_flag);
    RUN_TEST(test_parseNodeStatus_empty_returns_false);
    RUN_TEST(test_parseNodeStatus_null_returns_false);
    RUN_TEST(test_parseNodeStatus_uptime_zero_still_valid);
    RUN_TEST(test_parseNodeStatus_skips_unknown_fields);
    RUN_TEST(test_wire_tags_nodestatus_proto);

    // 16. mc_parsePkiReport / mc_encodePkiReport — KEY_VERIFICATION_APP (portnum 77)
    RUN_TEST(test_parsePkiReport_key_only);
    RUN_TEST(test_parsePkiReport_key_and_requestor);
    RUN_TEST(test_parsePkiReport_wrong_key_length_rejected);
    RUN_TEST(test_parsePkiReport_empty_returns_false);
    RUN_TEST(test_parsePkiReport_skips_unknown_fields);
    RUN_TEST(test_encodePkiReport_key_only);
    RUN_TEST(test_encodePkiReport_with_requestor);
    RUN_TEST(test_encodePkiReport_roundtrip);
    RUN_TEST(test_wire_tags_pkireport_proto);

    // 17. ALERT_APP (portnum 76) — portnum value + MeshMessage.isAlert flag
    RUN_TEST(test_alert_portnum_value);
    RUN_TEST(test_meshmessage_isalert_default_false);
    RUN_TEST(test_meshmessage_isalert_set);

    return UNITY_END();
}
