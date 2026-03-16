/**
 * test_mesh_codec.cxx — Unity tests for the platform-free Meshtastic codec.
 *
 * Run with: cmake -B build && cmake --build build && ctest --test-dir build -V
 *
 * Test groups
 * ───────────
 *  1. mc_pbVarint           — base-128 varint encoder
 *  2. mc_pbZigzag           — zigzag sint32 encoder
 *  3. mc_parseData          — Data proto decoder (portnum, payload, want_response)
 *  4. mc_parsePosition      — Position proto decoder (field-number regression tests)
 *  5. mc_parseUser          — User (NodeInfo) proto decoder
 *  6. Encode/decode round-trips
 *  7. mc_encodeData         — request_id field-6 regression (old code used field 7)
 */

#include "unity.h"
#include "mesh_codec.h"
#include <cstring>
#include <cstdint>
#include <climits>

// ── Unity required entry points ───────────────────────────────────────────
void setUp(void)    {}
void tearDown(void) {}

// ─────────────────────────────────────────────────────────────────────────
// Helper: encode a varint value and return the byte count while also letting
// us inspect the output bytes directly in test assertions.
// ─────────────────────────────────────────────────────────────────────────
static size_t encode_varint(uint64_t val, uint8_t* out)
{
    return mc_pbVarint(out, val);
}

// ─────────────────────────────────────────────────────────────────────────
// 1. mc_pbVarint
// ─────────────────────────────────────────────────────────────────────────

void test_varint_zero(void)
{
    uint8_t buf[2] = {};
    size_t n = encode_varint(0, buf);
    TEST_ASSERT_EQUAL_size_t(1, n);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[0]);
}

void test_varint_one(void)
{
    uint8_t buf[2] = {};
    size_t n = encode_varint(1, buf);
    TEST_ASSERT_EQUAL_size_t(1, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[0]);
}

void test_varint_max_single_byte(void)
{
    uint8_t buf[2] = {};
    size_t n = encode_varint(127, buf);
    TEST_ASSERT_EQUAL_size_t(1, n);
    TEST_ASSERT_EQUAL_HEX8(0x7F, buf[0]);
}

void test_varint_first_two_byte_value(void)
{
    // 128 = 0b10000000 → two bytes: 0x80 0x01
    uint8_t buf[3] = {};
    size_t n = encode_varint(128, buf);
    TEST_ASSERT_EQUAL_size_t(2, n);
    TEST_ASSERT_EQUAL_HEX8(0x80, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[1]);
}

void test_varint_300(void)
{
    // 300 = 0b100101100 → two bytes: 0xAC 0x02
    uint8_t buf[3] = {};
    size_t n = encode_varint(300, buf);
    TEST_ASSERT_EQUAL_size_t(2, n);
    TEST_ASSERT_EQUAL_HEX8(0xAC, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, buf[1]);
}

void test_varint_uint32_max(void)
{
    // 0xFFFFFFFF → five bytes
    uint8_t buf[6] = {};
    size_t n = encode_varint(0xFFFFFFFFu, buf);
    TEST_ASSERT_EQUAL_size_t(5, n);
    // MSB continuation bits set in first four bytes, clean final byte
    TEST_ASSERT_TRUE((buf[0] & 0x80) != 0);
    TEST_ASSERT_TRUE((buf[1] & 0x80) != 0);
    TEST_ASSERT_TRUE((buf[2] & 0x80) != 0);
    TEST_ASSERT_TRUE((buf[3] & 0x80) != 0);
    TEST_ASSERT_TRUE((buf[4] & 0x80) == 0);
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
    // INT32_MIN → UINT32_MAX (the largest zigzag value)
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, mc_pbZigzag(INT32_MIN));
}

void test_zigzag_int32_max(void)
{
    // INT32_MAX → (INT32_MAX * 2) = UINT32_MAX - 1
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFEu, mc_pbZigzag(INT32_MAX));
}

// ─────────────────────────────────────────────────────────────────────────
// 3. mc_parseData
// ─────────────────────────────────────────────────────────────────────────

// Hand-crafted minimal Data proto: portnum=1 TEXT_MESSAGE_APP, payload "hi"
// 08 01     field 1 (portnum)  varint 1
// 12 02 68 69   field 2 (payload)  2 bytes "hi"
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
    TEST_ASSERT_EQUAL_UINT32(1u, portnum);             // TEXT_MESSAGE_APP = 1
    TEST_ASSERT_NOT_NULL(payload);
    TEST_ASSERT_EQUAL_size_t(2u, payloadLen);
    TEST_ASSERT_EQUAL_UINT8('h', payload[0]);
    TEST_ASSERT_EQUAL_UINT8('i', payload[1]);
    TEST_ASSERT_FALSE(wantResp);
}

void test_parse_data_want_response(void)
{
    // portnum=67 NODEINFO_APP, payload "x", want_response=true
    // 08 43  12 01 78  18 01
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
    // Only a payload field, no portnum field 1
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
    // Valid start: portnum varint, then truncated length-delimited payload
    const uint8_t data[] = { 0x08, 0x01, 0x12, 0x10 };  // says 16 bytes follow but none do
    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;
    TEST_ASSERT_FALSE(mc_parseData(data, sizeof(data),
                                   portnum, payload, payloadLen, wantResp));
}

void test_parse_data_portnum_traceroute_empty_payload(void)
{
    // TRACEROUTE_APP (portnum=70) with empty payload is valid — empty RouteDiscovery.
    // Only portnum field, no field 2.
    const uint8_t data[] = { 0x08, 0x46 }; // varint 70
    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;

    TEST_ASSERT_TRUE(mc_parseData(data, sizeof(data),
                                  portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(70u, portnum);
    TEST_ASSERT_NULL(payload);
    TEST_ASSERT_EQUAL_size_t(0u, payloadLen);
}

// ─────────────────────────────────────────────────────────────────────────
// 4. mc_parsePosition
// ─────────────────────────────────────────────────────────────────────────

// Minimal correct Position proto:
//   lat_i=1 (field 1 sfixed32)   0D 01 00 00 00
//   lon_i=2 (field 2 sfixed32)   15 02 00 00 00
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
    // Only lon — gotLatLon never set
    const uint8_t data[] = { 0x15, 0x02, 0x00, 0x00, 0x00 };
    MeshPosition pos = {};
    TEST_ASSERT_FALSE(mc_parsePosition(data, sizeof(data), pos));
}

void test_parse_position_empty_returns_false(void)
{
    MeshPosition pos = {};
    TEST_ASSERT_FALSE(mc_parsePosition(nullptr, 0, pos));
}

// ── Field-number regression tests ────────────────────────────────────────
//
// TIME: must come from field 4 (sfixed32, tag 0x25), NOT field 9.
//   Field 9 varint (pos_flags): tag = (9<<3)|0 = 0x48
//   Field 9 sfixed32 (the OLD wrong location): tag = (9<<3)|5 = 0x4D
//
// SATS: must come from field 14 (varint, tag 0x70), NOT field 7.
//   Field 7 string (google_plus_code): tag = (7<<3)|2 = 0x3A
//
// This proto exercises ALL the previously-broken field numbers.

// Correct proto: time at field 4, sats at field 14
static const uint8_t kPosCorrectFields[] = {
    0x0D, 0x01, 0x00, 0x00, 0x00,         // field 1: lat_i = 1
    0x15, 0x02, 0x00, 0x00, 0x00,         // field 2: lon_i = 2
    0x18, 0x64,                            // field 3: alt_m = 100 (varint)
    0x25, 0x78, 0x56, 0x34, 0x12,         // field 4: time = 0x12345678 ← CORRECT
    0x3A, 0x05, 'h','e','l','l','o',      // field 7: google_plus_code (string) — must be SKIPPED
    0x48, 0x05,                            // field 9: pos_flags = 5 (varint) — must NOT go to time
    0x70, 0x08,                            // field 14: sats_in_view = 8 ← CORRECT
};

void test_parse_position_time_from_field4_not_field9(void)
{
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(kPosCorrectFields, sizeof(kPosCorrectFields), pos));
    TEST_ASSERT_EQUAL_INT32(1, pos.lat_i);
    TEST_ASSERT_EQUAL_INT32(2, pos.lon_i);
    TEST_ASSERT_EQUAL_INT32(100, pos.alt_m);
    // time must be 0x12345678 from field 4, not 5 from field 9 pos_flags
    TEST_ASSERT_EQUAL_UINT32(0x12345678u, pos.unixTime);
    // sats must be 8 from field 14, not anything from field 7 string
    TEST_ASSERT_EQUAL_UINT32(8u, pos.sats);
}

// Proto with time encoded at the OLD wrong location (field 9 sfixed32, tag 0x4D).
// The decoder must NOT store this into pos.unixTime.
static const uint8_t kPosOldWrongTimeField[] = {
    0x0D, 0x01, 0x00, 0x00, 0x00,         // field 1: lat_i = 1
    0x15, 0x02, 0x00, 0x00, 0x00,         // field 2: lon_i = 2
    0x4D, 0x78, 0x56, 0x34, 0x12,         // field 9 sfixed32 (tag 0x4D) — the OLD encoder bug
};

void test_parse_position_field9_sfixed32_not_stored_as_time(void)
{
    // The old firmware encoded time at field 9. The decoder must skip it.
    // unixTime should remain 0 since field 4 is absent.
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(kPosOldWrongTimeField, sizeof(kPosOldWrongTimeField), pos));
    TEST_ASSERT_EQUAL_UINT32(0u, pos.unixTime);
}

// Proto where field 7 is a varint (wrong wire type for google_plus_code = string).
// In practice Meshtastic sends field 7 as a string, but also verify we handle
// any unexpected wire types on unknown fields gracefully.
void test_parse_position_field7_string_not_stored_as_sats(void)
{
    // Field 7 is a length-delimited string. The decoder must skip it entirely.
    // No field 14, so pos.sats should remain 0.
    const uint8_t data[] = {
        0x0D, 0x01, 0x00, 0x00, 0x00,     // lat_i = 1
        0x15, 0x02, 0x00, 0x00, 0x00,     // lon_i = 2
        0x3A, 0x03, 0x61, 0x62, 0x63,     // field 7 string "abc" — must be SKIPPED
    };
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(data, sizeof(data), pos));
    TEST_ASSERT_EQUAL_UINT32(0u, pos.sats);
}

void test_parse_position_negative_lat(void)
{
    // lat_i = -296813520 (southern hemisphere: -29.6813520°)
    // Two's complement of 296813520 = 0xEE510F70
    const int32_t lat = -296813520;
    const uint32_t latBits = static_cast<uint32_t>(lat);
    const uint8_t data[] = {
        0x0D,
        (uint8_t)(latBits),
        (uint8_t)(latBits >> 8),
        (uint8_t)(latBits >> 16),
        (uint8_t)(latBits >> 24),
        0x15, 0x00, 0x00, 0x00, 0x00,     // lon_i = 0
    };
    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(data, sizeof(data), pos));
    TEST_ASSERT_EQUAL_INT32(lat, pos.lat_i);
}

void test_parse_position_negative_altitude(void)
{
    // alt_m = -10 (below sea level). Protobuf encodes negative int32 as 10-byte varint.
    // zigzag NOT used for int32, so -10 encodes as large positive via varint:
    // -10 as int64 = 0xFFFFFFFFFFFFFFF6 → 10 bytes in varint.
    // We use the encoder to build the bytes, then verify the decoder round-trips it.
    uint8_t buf[80] = {};
    size_t n = mc_encodePosition(buf, sizeof(buf),
                                 1,    // lat_i
                                 2,    // lon_i
                                 -10,  // alt_m
                                 3,    // sats
                                 0);   // unixTime
    TEST_ASSERT_GREATER_THAN(0u, n);

    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(buf, n, pos));
    TEST_ASSERT_EQUAL_INT32(-10, pos.alt_m);
}

void test_parse_position_truncated_sfixed32(void)
{
    // lat_i tag present but only 2 bytes of the 4-byte fixed32 follow
    const uint8_t data[] = { 0x0D, 0x01, 0x00 };
    MeshPosition pos = {};
    TEST_ASSERT_FALSE(mc_parsePosition(data, sizeof(data), pos));
}

void test_parse_position_all_fields_roundtrip(void)
{
    uint8_t buf[100] = {};
    const int32_t  lat       = 296813520;  // 29.681352 N
    const int32_t  lon       = -952347811; // 95.2347811 W
    const int32_t  alt       = 427;        // 427 m
    const uint32_t sats      = 9;
    const uint32_t ts        = 1741046400; // 2025-03-04 00:00:00 UTC
    const uint32_t speed     = 2778;       // ~100 km/h in cm/s
    const uint32_t track     = 18000;      // 180.00°

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

// ─────────────────────────────────────────────────────────────────────────
// 5. mc_parseUser
// ─────────────────────────────────────────────────────────────────────────

// Minimal User proto: only id field (required)
// 0A 0A 21 38 36 62 63 32 31 63 34 32  → field 1 string "!86bc21c4" (10 bytes)
static const uint8_t kUserMinimal[] = {
    0x0A, 0x09,
    '!','8','6','b','c','2','1','c','4'
};

void test_parse_user_minimal(void)
{
    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(kUserMinimal, sizeof(kUserMinimal), user));
    TEST_ASSERT_EQUAL_STRING("!86bc21c4", user.id);
    // Other fields default
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
    // Only hw_model field (field 5), no id (field 1)
    const uint8_t data[] = { 0x28, 0x30 }; // field 5 varint = 48 (HELTEC_WIRELESS_TRACKER)
    MeshUser user = {};
    TEST_ASSERT_FALSE(mc_parseUser(data, sizeof(data), user));
}

void test_parse_user_public_key_32_bytes(void)
{
    // Build a User proto with a 32-byte public key (field 8, tag = (8<<3)|2 = 0x42)
    uint8_t data[80] = {};
    size_t n = 0;
    // field 1: id
    const char* id = "!deadbeef";
    data[n++] = 0x0A;
    data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    // field 8: public_key (32 bytes)
    data[n++] = 0x42; // tag (8<<3)|2
    data[n++] = 32;   // length
    for (int i = 0; i < 32; i++) data[n++] = (uint8_t)(i + 1);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_EQUAL_STRING("!deadbeef", user.id);
    TEST_ASSERT_TRUE(user.hasPublicKey);
    // Verify first and last byte of public key
    TEST_ASSERT_EQUAL_UINT8(1u,  user.publicKey[0]);
    TEST_ASSERT_EQUAL_UINT8(32u, user.publicKey[31]);
}

void test_parse_user_public_key_wrong_length_not_stored(void)
{
    // 31-byte key — must be rejected (only 32-byte keys accepted)
    uint8_t data[80] = {};
    size_t n = 0;
    const char* id = "!deadbeef";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    data[n++] = 0x42; data[n++] = 31; // WRONG length
    for (int i = 0; i < 31; i++) data[n++] = (uint8_t)(i + 1);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    TEST_ASSERT_FALSE(user.hasPublicKey);   // must NOT be set
}

void test_parse_user_long_name_truncated_to_buffer(void)
{
    // long_name > 32 chars — must be truncated, no buffer overrun
    uint8_t data[200] = {};
    size_t n = 0;
    const char* id = "!aabbccdd";
    data[n++] = 0x0A; data[n++] = (uint8_t)strlen(id);
    memcpy(data + n, id, strlen(id)); n += strlen(id);
    // field 2 (long_name): 50-char string
    const char* longName = "AAAAAAAAAAAABBBBBBBBBBBBCCCCCCCCCCCCDDDDDDDDDDDDEE";
    data[n++] = 0x12; data[n++] = (uint8_t)strlen(longName);
    memcpy(data + n, longName, strlen(longName)); n += strlen(longName);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(data, n, user));
    // longName buffer is 33 chars — must be null-terminated within that
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
    // field 5: hw_model = 48 (HELTEC_WIRELESS_TRACKER)
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
                              "Long Device Name",
                              "LONG",
                              mac,
                              48,   // hw_model = HELTEC_WIRELESS_TRACKER
                              5,    // role = TRACKER
                              pubkey,
                              false);
    TEST_ASSERT_GREATER_THAN(0u, n);

    MeshUser user = {};
    TEST_ASSERT_TRUE(mc_parseUser(buf, n, user));
    TEST_ASSERT_EQUAL_STRING("!deadbeef", user.id);
    TEST_ASSERT_EQUAL_STRING("Long Device Name", user.longName);
    TEST_ASSERT_EQUAL_STRING("LONG", user.shortName);
    TEST_ASSERT_EQUAL_UINT32(48u, user.hwModel);
    TEST_ASSERT_TRUE(user.hasPublicKey);
    TEST_ASSERT_EQUAL_MEMORY(pubkey, user.publicKey, 32);
}

// ─────────────────────────────────────────────────────────────────────────
// 6. Encode/decode round-trips
// ─────────────────────────────────────────────────────────────────────────

void test_data_roundtrip_with_position_payload(void)
{
    // Encode a Position payload, wrap it in a Data proto, then decode.
    uint8_t posBuf[80] = {};
    size_t posLen = mc_encodePosition(posBuf, sizeof(posBuf),
                                      100000000, -800000000, 250,
                                      7, 1710000000u, 0, 0);
    TEST_ASSERT_GREATER_THAN(0u, posLen);

    uint8_t dataBuf[120] = {};
    size_t dataLen = mc_encodeData(dataBuf, sizeof(dataBuf),
                                   3,      // PORT_POSITION = 3
                                   posBuf, posLen,
                                   false,  // want_response
                                   0,      // dest (broadcast)
                                   0);     // requestId
    TEST_ASSERT_GREATER_THAN(0u, dataLen);

    uint32_t portnum = 0; const uint8_t* payload = nullptr;
    size_t payloadLen = 0; bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(dataBuf, dataLen,
                                  portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(3u, portnum);
    TEST_ASSERT_NOT_NULL(payload);
    TEST_ASSERT_EQUAL_size_t(posLen, payloadLen);
    TEST_ASSERT_EQUAL_MEMORY(posBuf, payload, posLen);

    MeshPosition pos = {};
    TEST_ASSERT_TRUE(mc_parsePosition(payload, payloadLen, pos));
    TEST_ASSERT_EQUAL_INT32(100000000,  pos.lat_i);
    TEST_ASSERT_EQUAL_INT32(-800000000, pos.lon_i);
    TEST_ASSERT_EQUAL_INT32(250,        pos.alt_m);
    TEST_ASSERT_EQUAL_UINT32(7u,        pos.sats);
    TEST_ASSERT_EQUAL_UINT32(1710000000u, pos.unixTime);
}

void test_telemetry_encode_contains_battery_fields(void)
{
    uint8_t buf[64] = {};
    const uint32_t now    = 1741046400u;
    const uint32_t uptime = 3600u;
    const uint8_t  level  = 85u;
    const float    volts  = 4.05f;

    size_t n = mc_encodeTelemetry(buf, sizeof(buf), now, uptime, level, volts);
    TEST_ASSERT_GREATER_THAN(0u, n);

    // Field 1 (time) tag = 0x0D (field 1 fixed32)
    TEST_ASSERT_EQUAL_HEX8(0x0D, buf[0]);
    // Field 2 (device_metrics) tag = 0x12 (field 2 LEN), appears at byte 5
    TEST_ASSERT_EQUAL_HEX8(0x12, buf[5]);
    // Inner DeviceMetrics: battery_level tag 0x08 appears right after field 2 length
    // Find 0x08 after the outer telemetry field 1 (5 bytes) + field 2 tag+len:
    TEST_ASSERT_EQUAL_HEX8(0x08, buf[7]);
    TEST_ASSERT_EQUAL_UINT8(level, buf[8]);  // varint 85 fits in 1 byte
}

// ─────────────────────────────────────────────────────────────────────────
// 7. mc_encodeData — request_id field regression
// ─────────────────────────────────────────────────────────────────────────
// BUG HISTORY: The old encoder used tag 0x3D (field 7, wire type 5) for
// request_id.  Meshtastic firmware uses field 6 (tag 0x35).  The fix was
// confirmed via live hex dump: "35 70 bc 3d 7d" = tag 0x35.
// These tests ensure the encoding never regresses back to 0x3D.

void test_encode_data_request_id_uses_field6_tag_0x35(void)
{
    const uint8_t payload[] = { 0x01, 0x02, 0x03 };
    uint8_t buf[64] = {};
    size_t n = mc_encodeData(buf, sizeof(buf),
                              67,       // PORT_NODEINFO
                              payload, sizeof(payload),
                              false,    // want_response
                              0xABCD1234u, // dest
                              0x12345678u); // requestId

    TEST_ASSERT_GREATER_THAN(0u, n);

    // Search for tag 0x35 (correct: field 6 fixed32) in encoded bytes
    bool found_0x35 = false;
    bool found_0x3D = false;
    for (size_t i = 0; i < n; i++) {
        if (buf[i] == 0x35) found_0x35 = true;
        if (buf[i] == 0x3D) found_0x3D = true;
    }

    TEST_ASSERT_TRUE_MESSAGE(found_0x35,  "request_id must use tag 0x35 (field 6)");
    TEST_ASSERT_FALSE_MESSAGE(found_0x3D, "tag 0x3D (field 7) must NOT appear in encoded Data");
}

void test_encode_data_no_request_id_no_tag_0x35(void)
{
    const uint8_t payload[] = { 0x01 };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 1, payload, 1,
                              false, 0, 0); // requestId = 0

    bool found_0x35 = false;
    for (size_t i = 0; i < n; i++)
        if (buf[i] == 0x35) found_0x35 = true;

    TEST_ASSERT_FALSE_MESSAGE(found_0x35,
        "tag 0x35 must not appear when requestId==0");
}

void test_encode_data_ok_to_mqtt_false_emitted_with_request_id(void)
{
    // When requestId is set, ok_to_mqtt=false (tag 0x48 0x00) must also be emitted.
    const uint8_t payload[] = { 0x01 };
    uint8_t buf[48] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 67, payload, 1,
                              false, 0xDEADBEEFu, 0x11223344u);

    bool found_0x48 = false;
    for (size_t i = 0; i + 1 < n; i++) {
        if (buf[i] == 0x48 && buf[i+1] == 0x00) { found_0x48 = true; break; }
    }
    TEST_ASSERT_TRUE_MESSAGE(found_0x48, "ok_to_mqtt=false (0x48 0x00) must accompany request_id");
}

void test_encode_data_want_response_tag_present(void)
{
    const uint8_t payload[] = { 0xAA };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 67, payload, 1,
                              true, 0, 0); // want_response=true

    bool found = false;
    for (size_t i = 0; i + 1 < n; i++)
        if (buf[i] == 0x18 && buf[i+1] == 0x01) { found = true; break; }
    TEST_ASSERT_TRUE_MESSAGE(found, "want_response=true must emit tag 0x18 value 0x01");
}

void test_encode_data_no_want_response_tag_absent(void)
{
    const uint8_t payload[] = { 0xAA };
    uint8_t buf[32] = {};
    size_t n = mc_encodeData(buf, sizeof(buf), 67, payload, 1,
                              false, 0, 0);

    for (size_t i = 0; i < n; i++)
        TEST_ASSERT_NOT_EQUAL_HEX8_MESSAGE(0x18, buf[i],
            "tag 0x18 (want_response) must not appear when want_response=false");
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

    // 2. mc_pbZigzag
    RUN_TEST(test_zigzag_zero);
    RUN_TEST(test_zigzag_neg_one);
    RUN_TEST(test_zigzag_pos_one);
    RUN_TEST(test_zigzag_neg_two);
    RUN_TEST(test_zigzag_pos_two);
    RUN_TEST(test_zigzag_int32_min);
    RUN_TEST(test_zigzag_int32_max);

    // 3. mc_parseData
    RUN_TEST(test_parse_data_basic);
    RUN_TEST(test_parse_data_want_response);
    RUN_TEST(test_parse_data_no_portnum_returns_false);
    RUN_TEST(test_parse_data_empty_input);
    RUN_TEST(test_parse_data_truncated_returns_false);
    RUN_TEST(test_parse_data_portnum_traceroute_empty_payload);

    // 4. mc_parsePosition — field-number regression tests
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

    // 5. mc_parseUser
    RUN_TEST(test_parse_user_minimal);
    RUN_TEST(test_parse_user_empty_returns_false);
    RUN_TEST(test_parse_user_no_id_returns_false);
    RUN_TEST(test_parse_user_public_key_32_bytes);
    RUN_TEST(test_parse_user_public_key_wrong_length_not_stored);
    RUN_TEST(test_parse_user_long_name_truncated_to_buffer);
    RUN_TEST(test_parse_user_hw_model);
    RUN_TEST(test_parse_user_all_fields_roundtrip);

    // 6. Encode/decode round-trips
    RUN_TEST(test_data_roundtrip_with_position_payload);
    RUN_TEST(test_telemetry_encode_contains_battery_fields);

    // 7. mc_encodeData — request_id field regression
    RUN_TEST(test_encode_data_request_id_uses_field6_tag_0x35);
    RUN_TEST(test_encode_data_no_request_id_no_tag_0x35);
    RUN_TEST(test_encode_data_ok_to_mqtt_false_emitted_with_request_id);
    RUN_TEST(test_encode_data_want_response_tag_present);
    RUN_TEST(test_encode_data_no_want_response_tag_absent);

    return UNITY_END();
}
