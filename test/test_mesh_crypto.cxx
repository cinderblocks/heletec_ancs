/**
 * test_mesh_crypto.cxx — Unity tests for Meshtastic crypto primitives.
 *
 * Build and run:
 *   cd test && cmake -B build && cmake --build build -j$(nproc)
 *   ctest --test-dir build --output-on-failure -R test_mesh_crypto
 * Or verbosely:
 *   ./build/test_mesh_crypto
 *
 * ── Reference material ───────────────────────────────────────────────────
 *
 * AES-CTR vectors: NIST SP 800-38A (2001), Appendix F.5
 *   https://csrc.nist.gov/publications/detail/sp/800-38a/final
 *
 * X25519 vectors: RFC 7748 Section 6.1
 *   https://datatracker.ietf.org/doc/html/rfc7748#section-6.1
 *
 * Meshtastic encryption: meshtastic.org/docs/overview/encryption/
 *   - Channel cipher: AES-128-CTR, key=PSK, nonce=[pktId LE|0|fromNode LE|0]
 *   - PKC DM cipher:  AES-256-CTR, key=X25519 ECDH shared secret (LE),
 *                     same nonce layout, chanHash=0x00 on wire.
 *
 * Channel hash computation (DEFAULT_CHAN_HASH = 0x08):
 *   xorHash("LongFast", 8) = 0x0A
 *   xorHash(DEFAULT_PSK,  16) = 0x02    (XOR of all 16 PSK bytes)
 *   hash = 0x0A ^ 0x02 = 0x08
 *
 * ── Test groups ──────────────────────────────────────────────────────────
 *  1. mc_buildNonce          — nonce byte layout
 *  2. mc_aes128ctr           — NIST SP 800-38A F.5.1 vectors
 *  3. mc_aes256ctr           — NIST SP 800-38A F.5.5 vectors
 *  4. mc_channelCrypt        — Meshtastic DEFAULT_PSK round-trips
 *  5. mc_x25519PublicKey     — RFC 7748 known key derivations
 *  6. mc_x25519SharedSecret  — RFC 7748 known shared secrets
 *  7. mc_pkcCrypt            — PKC DM encrypt/decrypt round-trips
 *  8. Full Meshtastic OTA frame — encode→encrypt→decrypt→parse
 *  9. Channel-hash computation — verify DEFAULT_CHAN_HASH = 0x08
 */

#include "unity.h"
#include "mesh_crypto.h"
#include "mesh_codec.h"    // mc_encodeData, mc_parseData (for OTA frame tests)

#include <cstring>
#include <cstdint>

void setUp(void)    {}
void tearDown(void) {}

// ── Meshtastic DEFAULT_PSK ────────────────────────────────────────────────
// The factory LongFast channel AES-128 key used by every unmodified
// Meshtastic device.  Hardcoded here so tests are self-contained.
static const uint8_t DEFAULT_PSK[16] = {
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};

// ─────────────────────────────────────────────────────────────────────────
// 1. mc_buildNonce
// ─────────────────────────────────────────────────────────────────────────

void test_nonce_layout_packetid_le(void)
{
    // packetId = 0x04030201 → bytes [0:3] must be 01 02 03 04 (little-endian)
    uint8_t n[16] = {};
    mc_buildNonce(0x04030201u, 0xDEADBEEFu, n);
    TEST_ASSERT_EQUAL_HEX8(0x01, n[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, n[1]);
    TEST_ASSERT_EQUAL_HEX8(0x03, n[2]);
    TEST_ASSERT_EQUAL_HEX8(0x04, n[3]);
}

void test_nonce_layout_middle_zero(void)
{
    // bytes [4:7] must always be 0x00
    uint8_t n[16] = {};
    mc_buildNonce(0xFFFFFFFFu, 0xFFFFFFFFu, n);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[4]);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[5]);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[7]);
}

void test_nonce_layout_fromnode_le(void)
{
    // fromNode = 0x12345678 → bytes [8:11] must be 78 56 34 12
    uint8_t n[16] = {};
    mc_buildNonce(0x00000001u, 0x12345678u, n);
    TEST_ASSERT_EQUAL_HEX8(0x78, n[8]);
    TEST_ASSERT_EQUAL_HEX8(0x56, n[9]);
    TEST_ASSERT_EQUAL_HEX8(0x34, n[10]);
    TEST_ASSERT_EQUAL_HEX8(0x12, n[11]);
}

void test_nonce_layout_trailing_zero(void)
{
    // bytes [12:15] must always be 0x00
    uint8_t n[16] = {};
    mc_buildNonce(0xFFFFFFFFu, 0xFFFFFFFFu, n);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[12]);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[13]);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[14]);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[15]);
}

void test_nonce_zero_inputs(void)
{
    uint8_t n[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    mc_buildNonce(0, 0, n);
    for (int i = 0; i < 16; i++)
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x00, n[i], "nonce byte must be 0 when both inputs are 0");
}

void test_nonce_packetid_only(void)
{
    // Sanity: fromNode=0, only packetId contributes to [0:3]
    uint8_t n[16] = {};
    mc_buildNonce(0xAABBCCDDu, 0u, n);
    TEST_ASSERT_EQUAL_HEX8(0xDD, n[0]);
    TEST_ASSERT_EQUAL_HEX8(0xCC, n[1]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, n[2]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, n[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, n[8]);
}

// ─────────────────────────────────────────────────────────────────────────
// 2. mc_aes128ctr — NIST SP 800-38A Appendix F.5.1
//
// Key:     2b7e151628aed2a6abf7158809cf4f3c
// Counter: f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff
//
// Block 1: PT  6bc1bee22e409f96e93d7e117393172a
//          CT  874d6191b620e3261bef6864990db6ce
// Block 2: PT  ae2d8a571e03ac9c9eb76fac45af8e51
//          CT  9806f66b7970fdff8617187bb9fffdff
// ─────────────────────────────────────────────────────────────────────────

static const uint8_t NIST_AES128_KEY[16] = {
    0x2b,0x7e,0x15,0x16, 0x28,0xae,0xd2,0xa6,
    0xab,0xf7,0x15,0x88, 0x09,0xcf,0x4f,0x3c
};
static const uint8_t NIST_AES128_CTR[16] = {
    0xf0,0xf1,0xf2,0xf3, 0xf4,0xf5,0xf6,0xf7,
    0xf8,0xf9,0xfa,0xfb, 0xfc,0xfd,0xfe,0xff
};
static const uint8_t NIST_AES128_PT1[16] = {
    0x6b,0xc1,0xbe,0xe2, 0x2e,0x40,0x9f,0x96,
    0xe9,0x3d,0x7e,0x11, 0x73,0x93,0x17,0x2a
};
static const uint8_t NIST_AES128_CT1[16] = {
    0x87,0x4d,0x61,0x91, 0xb6,0x20,0xe3,0x26,
    0x1b,0xef,0x68,0x64, 0x99,0x0d,0xb6,0xce
};
static const uint8_t NIST_AES128_PT2[16] = {
    0xae,0x2d,0x8a,0x57, 0x1e,0x03,0xac,0x9c,
    0x9e,0xb7,0x6f,0xac, 0x45,0xaf,0x8e,0x51
};
static const uint8_t NIST_AES128_CT2[16] = {
    0x98,0x06,0xf6,0x6b, 0x79,0x70,0xfd,0xff,
    0x86,0x17,0x18,0x7b, 0xb9,0xff,0xfd,0xff
};

void test_aes128ctr_nist_block1(void)
{
    uint8_t ct[16] = {};
    TEST_ASSERT_TRUE(mc_aes128ctr(NIST_AES128_KEY, NIST_AES128_CTR,
                                   NIST_AES128_PT1, 16, ct));
    TEST_ASSERT_EQUAL_MEMORY(NIST_AES128_CT1, ct, 16);
}

void test_aes128ctr_nist_block2(void)
{
    // Provide 32 bytes (blocks 1+2) in one call — counter auto-advances
    uint8_t pt32[32], ct32[32], expected[32];
    memcpy(pt32,       NIST_AES128_PT1, 16);
    memcpy(pt32 + 16,  NIST_AES128_PT2, 16);
    memcpy(expected,      NIST_AES128_CT1, 16);
    memcpy(expected + 16, NIST_AES128_CT2, 16);

    TEST_ASSERT_TRUE(mc_aes128ctr(NIST_AES128_KEY, NIST_AES128_CTR, pt32, 32, ct32));
    TEST_ASSERT_EQUAL_MEMORY(expected, ct32, 32);
}

void test_aes128ctr_decrypt_nist_block1(void)
{
    // CTR mode is symmetric: decrypt(CT) must give back PT
    uint8_t pt[16] = {};
    TEST_ASSERT_TRUE(mc_aes128ctr(NIST_AES128_KEY, NIST_AES128_CTR,
                                   NIST_AES128_CT1, 16, pt));
    TEST_ASSERT_EQUAL_MEMORY(NIST_AES128_PT1, pt, 16);
}

void test_aes128ctr_roundtrip(void)
{
    const uint8_t key[16] = {0x01,0x02,0x03,0x04, 0x05,0x06,0x07,0x08,
                              0x09,0x0a,0x0b,0x0c, 0x0d,0x0e,0x0f,0x10};
    const uint8_t nonce[16] = {};
    const char    plaintext[] = "Hello Meshtastic!";
    const size_t  len = sizeof(plaintext) - 1;

    uint8_t ct[64] = {}, pt[64] = {};
    TEST_ASSERT_TRUE(mc_aes128ctr(key, nonce, (const uint8_t*)plaintext, len, ct));
    TEST_ASSERT_TRUE(mc_aes128ctr(key, nonce, ct, len, pt));
    TEST_ASSERT_EQUAL_MEMORY(plaintext, pt, len);
}

void test_aes128ctr_different_nonces_give_different_output(void)
{
    const uint8_t key[16] = {0xAA,0xBB,0xCC,0xDD, 0xAA,0xBB,0xCC,0xDD,
                              0xAA,0xBB,0xCC,0xDD, 0xAA,0xBB,0xCC,0xDD};
    const uint8_t pt[16]  = {0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
                              0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00};
    const uint8_t nonce1[16] = {0x01};
    const uint8_t nonce2[16] = {0x02};

    uint8_t ct1[16] = {}, ct2[16] = {};
    mc_aes128ctr(key, nonce1, pt, 16, ct1);
    mc_aes128ctr(key, nonce2, pt, 16, ct2);
    TEST_ASSERT_FALSE_MESSAGE(memcmp(ct1, ct2, 16) == 0,
        "Different nonces must produce different ciphertext");
}

void test_aes128ctr_zero_len_returns_false(void)
{
    uint8_t ct[1] = {};
    TEST_ASSERT_FALSE(mc_aes128ctr(NIST_AES128_KEY, NIST_AES128_CTR, ct, 0, ct));
}

// ─────────────────────────────────────────────────────────────────────────
// 3. mc_aes256ctr — NIST SP 800-38A Appendix F.5.5
//
// Key:     603deb1015ca71be2b73aef0857d7781
//          1f352c073b6108d72d9810a30914dff4
// Counter: f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff
//
// Block 1: PT  6bc1bee22e409f96e93d7e117393172a
//          CT  601ec313775789a5b7a7f504bbf3d228
// ─────────────────────────────────────────────────────────────────────────

static const uint8_t NIST_AES256_KEY[32] = {
    0x60,0x3d,0xeb,0x10, 0x15,0xca,0x71,0xbe,
    0x2b,0x73,0xae,0xf0, 0x85,0x7d,0x77,0x81,
    0x1f,0x35,0x2c,0x07, 0x3b,0x61,0x08,0xd7,
    0x2d,0x98,0x10,0xa3, 0x09,0x14,0xdf,0xf4
};
static const uint8_t NIST_AES256_CTR[16] = {
    0xf0,0xf1,0xf2,0xf3, 0xf4,0xf5,0xf6,0xf7,
    0xf8,0xf9,0xfa,0xfb, 0xfc,0xfd,0xfe,0xff
};
static const uint8_t NIST_AES256_PT1[16] = {
    0x6b,0xc1,0xbe,0xe2, 0x2e,0x40,0x9f,0x96,
    0xe9,0x3d,0x7e,0x11, 0x73,0x93,0x17,0x2a
};
static const uint8_t NIST_AES256_CT1[16] = {
    0x60,0x1e,0xc3,0x13, 0x77,0x57,0x89,0xa5,
    0xb7,0xa7,0xf5,0x04, 0xbb,0xf3,0xd2,0x28
};

void test_aes256ctr_nist_block1(void)
{
    uint8_t ct[16] = {};
    TEST_ASSERT_TRUE(mc_aes256ctr(NIST_AES256_KEY, NIST_AES256_CTR,
                                   NIST_AES256_PT1, 16, ct));
    TEST_ASSERT_EQUAL_MEMORY(NIST_AES256_CT1, ct, 16);
}

void test_aes256ctr_decrypt_nist_block1(void)
{
    uint8_t pt[16] = {};
    TEST_ASSERT_TRUE(mc_aes256ctr(NIST_AES256_KEY, NIST_AES256_CTR,
                                   NIST_AES256_CT1, 16, pt));
    TEST_ASSERT_EQUAL_MEMORY(NIST_AES256_PT1, pt, 16);
}

void test_aes256ctr_roundtrip_arbitrary(void)
{
    // Use a zero key and a Meshtastic-style nonce to verify round-trip
    const uint8_t key[32] = {};
    uint8_t nonce[16] = {};
    mc_buildNonce(0xBEEFBEEF, 0x12345678, nonce);

    const uint8_t pt[32] = {
        0x08,0x01,0x12,0x1c, 'H','e','l','l','o',' ','f','r','o','m',
        ' ','M','e','s','h','t','a','s','t','i','c','!',0x00,0x00,0x00,0x00,
        0x00,0x00
    };
    uint8_t ct[32] = {}, pt2[32] = {};
    TEST_ASSERT_TRUE(mc_aes256ctr(key, nonce, pt, 32, ct));
    TEST_ASSERT_TRUE(mc_aes256ctr(key, nonce, ct, 32, pt2));
    TEST_ASSERT_EQUAL_MEMORY(pt, pt2, 32);
}

// ─────────────────────────────────────────────────────────────────────────
// 4. mc_channelCrypt — Meshtastic DEFAULT_PSK
// ─────────────────────────────────────────────────────────────────────────

void test_channel_crypt_roundtrip_text_message(void)
{
    // Data proto: portnum=1 (TEXT_MESSAGE_APP), payload="Hello!!!"
    const uint8_t pt[] = { 0x08,0x01, 0x12,0x08,
                            'H','e','l','l','o','!','!','!' };
    const size_t len = sizeof(pt);
    const uint32_t pktId    = 0xDEADBEEF;
    const uint32_t fromNode = 0x12345678;

    uint8_t ct[sizeof(pt)] = {}, pt2[sizeof(pt)] = {};
    TEST_ASSERT_TRUE(mc_channelCrypt(DEFAULT_PSK, pktId, fromNode, pt, len, ct));
    TEST_ASSERT_TRUE(mc_channelCrypt(DEFAULT_PSK, pktId, fromNode, ct, len, pt2));
    TEST_ASSERT_EQUAL_MEMORY(pt, pt2, len);
}

void test_channel_crypt_ciphertext_differs_from_plaintext(void)
{
    const uint8_t pt[16] = "Mesh message!!";
    uint8_t ct[16] = {};
    mc_channelCrypt(DEFAULT_PSK, 1, 1, pt, 16, ct);
    TEST_ASSERT_FALSE_MESSAGE(memcmp(pt, ct, 16) == 0,
        "Ciphertext must differ from plaintext");
}

void test_channel_crypt_different_packetids_different_output(void)
{
    // Two identical plaintexts encrypted with different packetIds must differ
    const uint8_t pt[16] = {};
    uint8_t ct1[16] = {}, ct2[16] = {};
    mc_channelCrypt(DEFAULT_PSK, 0x00000001, 0xDEADBEEF, pt, 16, ct1);
    mc_channelCrypt(DEFAULT_PSK, 0x00000002, 0xDEADBEEF, pt, 16, ct2);
    TEST_ASSERT_FALSE_MESSAGE(memcmp(ct1, ct2, 16) == 0,
        "Different packetIds must change the ciphertext (nonce sensitivity)");
}

void test_channel_crypt_different_fromnodes_different_output(void)
{
    const uint8_t pt[16] = {};
    uint8_t ct1[16] = {}, ct2[16] = {};
    mc_channelCrypt(DEFAULT_PSK, 0xDEADBEEF, 0xAAAAAAAA, pt, 16, ct1);
    mc_channelCrypt(DEFAULT_PSK, 0xDEADBEEF, 0xBBBBBBBB, pt, 16, ct2);
    TEST_ASSERT_FALSE_MESSAGE(memcmp(ct1, ct2, 16) == 0,
        "Different fromNode values must change the ciphertext (nonce sensitivity)");
}

void test_channel_crypt_deterministic(void)
{
    // Same inputs always give the same output (no random state)
    const uint8_t pt[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    uint8_t ct1[16] = {}, ct2[16] = {};
    mc_channelCrypt(DEFAULT_PSK, 0xCAFEBABE, 0x87654321, pt, 16, ct1);
    mc_channelCrypt(DEFAULT_PSK, 0xCAFEBABE, 0x87654321, pt, 16, ct2);
    TEST_ASSERT_EQUAL_MEMORY(ct1, ct2, 16);
}

void test_channel_crypt_symmetric_encrypt_equals_decrypt(void)
{
    // encrypt(encrypt(pt)) == pt  for any input
    const uint8_t pt[20] = {0x08,0x01, 0x12,0x10,
                             'S','e','c','r','e','t',' ','M','e','s','s','a',
                             'g','e','!',0};
    uint8_t ct[20] = {}, pt2[20] = {};
    mc_channelCrypt(DEFAULT_PSK, 0xABCD1234, 0xDEAD0000, pt, 20, ct);
    mc_channelCrypt(DEFAULT_PSK, 0xABCD1234, 0xDEAD0000, ct, 20, pt2);
    TEST_ASSERT_EQUAL_MEMORY(pt, pt2, 20);
}

// ─────────────────────────────────────────────────────────────────────────
// 5. mc_x25519PublicKey — RFC 7748 Section 6.1 Known Key Derivations
//
// RFC 7748 §6.1 provides raw (unclamped) private keys.  The X25519 function
// is defined (RFC 7748 §5) to clamp the scalar before use:
//   k[0]  &= 0xF8  (clear bits 0,1,2)
//   k[31] &= 0x7F  (clear bit 7)
//   k[31] |= 0x40  (set  bit 6)
// mc_x25519PublicKey applies this clamping internally, matching the RFC
// computation.  The expected public keys below are from the RFC and are the
// correct output for X25519(clamp(priv), base_point_9).
//
// Alice private (raw LE, as given in RFC 7748):
//   77076d0a7318a57d3c16c17251b26645df949d789577965b83f63f2e9fc282e5
// Alice public (LE):
//   8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a
//
// Bob private (raw LE, as given in RFC 7748):
//   5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c268a9c6751daae
// Bob public (LE):
//   de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f
// ─────────────────────────────────────────────────────────────────────────

static const uint8_t RFC7748_ALICE_PRIV[32] = {
    0x77,0x07,0x6d,0x0a, 0x73,0x18,0xa5,0x7d,
    0x3c,0x16,0xc1,0x72, 0x51,0xb2,0x66,0x45,
    0xdf,0x94,0x9d,0x78, 0x95,0x77,0x96,0x5b,
    0x83,0xf6,0x3f,0x2e, 0x9f,0xc2,0x82,0xe5
};
static const uint8_t RFC7748_ALICE_PUB[32] = {
    0x85,0x20,0xf0,0x09, 0x89,0x30,0xa7,0x54,
    0x74,0x8b,0x7d,0xdc, 0xb4,0x3e,0xf7,0x5a,
    0x0d,0xbf,0x3a,0x0d, 0x26,0x38,0x1a,0xf4,
    0xeb,0xa4,0xa9,0x8e, 0xaa,0x9b,0x4e,0x6a
};
static const uint8_t RFC7748_BOB_PRIV[32] = {
    0x5d,0xab,0x08,0x7e, 0x62,0x4a,0x8a,0x4b,
    0x79,0xe1,0x7f,0x8b, 0x83,0x80,0x0e,0xe6,
    0x6f,0x3b,0xb1,0x29, 0x26,0x18,0xb6,0xfd,
    0x1c,0x26,0x8a,0x9c, 0x67,0x51,0xda,0xae
};
static const uint8_t RFC7748_BOB_PUB[32] = {
    0xde,0x9e,0xdb,0x7d, 0x7b,0x7d,0xc1,0xb4,
    0xd3,0x5b,0x61,0xc2, 0xec,0xe4,0x35,0x37,
    0x3f,0x83,0x43,0xc8, 0x5b,0x78,0x67,0x4d,
    0xad,0xfc,0x7e,0x14, 0x6f,0x88,0x2b,0x4f
};
static const uint8_t RFC7748_SHARED[32] = {
    0x4a,0x5d,0x9d,0x5b, 0xa4,0xce,0x2d,0xe1,
    0x72,0x8e,0x3b,0xf4, 0x80,0x35,0x0f,0x25,
    0xe0,0x7e,0x21,0xc9, 0x47,0xd1,0x9e,0x33,
    0x76,0xf0,0x9b,0x3c, 0x1e,0x16,0x17,0x42
};

void test_x25519_alice_public_key(void)
{
    uint8_t pub[32] = {};
    TEST_ASSERT_TRUE(mc_x25519PublicKey(RFC7748_ALICE_PRIV, pub));
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(RFC7748_ALICE_PUB, pub, 32,
        "Alice public key must match RFC 7748 test vector");
}

void test_x25519_bob_public_key(void)
{
    uint8_t pub[32] = {};
    TEST_ASSERT_TRUE(mc_x25519PublicKey(RFC7748_BOB_PRIV, pub));
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(RFC7748_BOB_PUB, pub, 32,
        "Bob public key must match RFC 7748 test vector");
}

void test_x25519_public_key_is_deterministic(void)
{
    // Same private key must always produce the same public key
    uint8_t pub1[32] = {}, pub2[32] = {};
    mc_x25519PublicKey(RFC7748_ALICE_PRIV, pub1);
    mc_x25519PublicKey(RFC7748_ALICE_PRIV, pub2);
    TEST_ASSERT_EQUAL_MEMORY(pub1, pub2, 32);
}

void test_x25519_different_privkeys_different_pubkeys(void)
{
    uint8_t pub1[32] = {}, pub2[32] = {};
    mc_x25519PublicKey(RFC7748_ALICE_PRIV, pub1);
    mc_x25519PublicKey(RFC7748_BOB_PRIV,   pub2);
    TEST_ASSERT_FALSE_MESSAGE(memcmp(pub1, pub2, 32) == 0,
        "Different private keys must produce different public keys");
}

// ─────────────────────────────────────────────────────────────────────────
// 6. mc_x25519SharedSecret — RFC 7748 Section 6.1
// ─────────────────────────────────────────────────────────────────────────

void test_x25519_shared_secret_alice_to_bob(void)
{
    // Alice uses her private key + Bob's public key
    uint8_t shared[32] = {};
    TEST_ASSERT_TRUE(mc_x25519SharedSecret(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB, shared));
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(RFC7748_SHARED, shared, 32,
        "Alice→Bob shared secret must match RFC 7748 vector");
}

void test_x25519_shared_secret_bob_to_alice(void)
{
    // Bob uses his private key + Alice's public key — must give same secret
    uint8_t shared[32] = {};
    TEST_ASSERT_TRUE(mc_x25519SharedSecret(RFC7748_BOB_PRIV, RFC7748_ALICE_PUB, shared));
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(RFC7748_SHARED, shared, 32,
        "Bob→Alice shared secret must match RFC 7748 vector (ECDH commutativity)");
}

void test_x25519_shared_secret_commutativity(void)
{
    // ECDH(a, B) == ECDH(b, A) for any valid keypair
    uint8_t s1[32] = {}, s2[32] = {};
    mc_x25519SharedSecret(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB, s1);
    mc_x25519SharedSecret(RFC7748_BOB_PRIV,   RFC7748_ALICE_PUB, s2);
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(s1, s2, 32,
        "ECDH must be commutative: ECDH(a,B) == ECDH(b,A)");
}

void test_x25519_shared_secret_is_deterministic(void)
{
    uint8_t s1[32] = {}, s2[32] = {};
    mc_x25519SharedSecret(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB, s1);
    mc_x25519SharedSecret(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB, s2);
    TEST_ASSERT_EQUAL_MEMORY(s1, s2, 32);
}

void test_x25519_wrong_remote_key_gives_different_secret(void)
{
    // If we use the wrong remote public key, we get a different shared secret
    uint8_t good[32] = {}, bad[32] = {};
    mc_x25519SharedSecret(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB,   good);
    mc_x25519SharedSecret(RFC7748_ALICE_PRIV, RFC7748_ALICE_PUB, bad); // Alice's own pub
    TEST_ASSERT_FALSE_MESSAGE(memcmp(good, bad, 32) == 0,
        "Wrong remote key must produce a different shared secret");
}

// ─────────────────────────────────────────────────────────────────────────
// 7. mc_pkcCrypt — PKC DM encrypt/decrypt round-trips
//
// In Meshtastic PKC DM flow:
//   Sender (Alice) encrypts:  mc_pkcCrypt(alicePriv, bobPub,   pktId, aliceNode, pt → ct)
//   Receiver (Bob) decrypts:  mc_pkcCrypt(bobPriv,  alicePub, pktId, aliceNode, ct → pt)
//
// The fromNode in the nonce is always the SENDER's node ID, even when
// the receiver (Bob) is decrypting.  This matches Meshtastic's
// CryptoEngine::initCounter() where fromNode is the OTA header sender.
// ─────────────────────────────────────────────────────────────────────────

void test_pkc_crypt_roundtrip_basic(void)
{
    const uint8_t pt[] = "Direct message via PKC";
    const size_t  len  = sizeof(pt) - 1;
    const uint32_t pktId    = 0x11223344;
    const uint32_t aliceNode = 0xAAAA0001; // sender node ID

    uint8_t ct[64] = {}, pt2[64] = {};

    // Alice encrypts to Bob
    TEST_ASSERT_TRUE(mc_pkcCrypt(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB,
                                  pktId, aliceNode, pt, len, ct));
    // Bob decrypts from Alice
    TEST_ASSERT_TRUE(mc_pkcCrypt(RFC7748_BOB_PRIV, RFC7748_ALICE_PUB,
                                  pktId, aliceNode, ct, len, pt2));
    TEST_ASSERT_EQUAL_MEMORY(pt, pt2, len);
}

void test_pkc_crypt_ciphertext_differs_from_plaintext(void)
{
    const uint8_t pt[32] = {0x08,0x01,0x12,0x18,'P','K','C',' ','t','e',
                             's','t',' ','m','e','s','s','a','g','e',' ',
                             'h','e','r','e','!',0,0,0,0,0,0};
    uint8_t ct[32] = {};
    mc_pkcCrypt(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB, 0xDEAD, 0xBEEF, pt, 32, ct);
    TEST_ASSERT_FALSE_MESSAGE(memcmp(pt, ct, 32) == 0,
        "PKC ciphertext must differ from plaintext");
}

void test_pkc_crypt_different_packetid_different_ciphertext(void)
{
    // Changing the packetId changes the nonce, changing the ciphertext
    const uint8_t pt[16] = {};
    uint8_t ct1[16] = {}, ct2[16] = {};
    mc_pkcCrypt(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB, 0x00000001, 0xAAAA0001, pt, 16, ct1);
    mc_pkcCrypt(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB, 0x00000002, 0xAAAA0001, pt, 16, ct2);
    TEST_ASSERT_FALSE_MESSAGE(memcmp(ct1, ct2, 16) == 0,
        "Different packetIds must produce different PKC ciphertext");
}

void test_pkc_crypt_wrong_key_fails_decryption(void)
{
    // If Bob uses the wrong private key, the decrypted plaintext must be garbage
    const uint8_t pt[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    uint8_t ct[16] = {}, wrong_pt[16] = {};

    mc_pkcCrypt(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB, 0x1234, 0x5678, pt, 16, ct);

    // "Bob" decrypts with Alice's key (wrong key) instead of his own
    mc_pkcCrypt(RFC7748_ALICE_PRIV, RFC7748_ALICE_PUB, 0x1234, 0x5678, ct, 16, wrong_pt);

    TEST_ASSERT_FALSE_MESSAGE(memcmp(pt, wrong_pt, 16) == 0,
        "Wrong decryption key must not recover the original plaintext");
}

void test_pkc_crypt_roundtrip_data_proto(void)
{
    // Build a real Meshtastic Data proto payload, encrypt as PKC DM, then decrypt
    // Data proto: portnum=1 (TEXT_MESSAGE), payload="PKC DM test"
    const uint8_t pt[] = {
        0x08, 0x01,                          // field 1: portnum = 1
        0x12, 0x0b,                          // field 2: payload (11 bytes)
        'P','K','C',' ','D','M',' ','t','e','s','t'
    };
    const size_t  len      = sizeof(pt);
    const uint32_t pktId   = 0xFEEDFACE;
    const uint32_t senderNode = 0xDEAD0000;

    uint8_t ct[sizeof(pt)] = {};
    uint8_t pt_out[sizeof(pt)] = {};

    TEST_ASSERT_TRUE(mc_pkcCrypt(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB,
                                  pktId, senderNode, pt, len, ct));
    TEST_ASSERT_TRUE(mc_pkcCrypt(RFC7748_BOB_PRIV, RFC7748_ALICE_PUB,
                                  pktId, senderNode, ct, len, pt_out));
    TEST_ASSERT_EQUAL_MEMORY(pt, pt_out, len);

    // Also verify the decrypted bytes parse as a valid Data proto
    uint32_t portnum = 0;
    const uint8_t* payload = nullptr;
    size_t payloadLen = 0;
    bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(pt_out, len, portnum, payload, payloadLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(1u, portnum);
    TEST_ASSERT_EQUAL_size_t(11u, payloadLen);
    TEST_ASSERT_EQUAL_MEMORY("PKC DM test", payload, 11);
}

// ─────────────────────────────────────────────────────────────────────────
// 8. Full Meshtastic OTA frame — encode → channel-encrypt → decrypt → parse
// ─────────────────────────────────────────────────────────────────────────

void test_full_ota_channel_text_message(void)
{
    // Simulate a complete Meshtastic OTA frame for a TEXT_MESSAGE_APP broadcast.
    //
    // OTA layout:
    //   [0:3]   to       = 0xFFFFFFFF (broadcast)
    //   [4:7]   from     = fromNode
    //   [8:11]  id       = packetId
    //   [12]    flags    = 0x63 (broadcast, hop_limit=3, hop_start=3)
    //   [13]    chanHash = 0x08 (DEFAULT_CHAN_HASH for LongFast)
    //   [14:15] pad      = 0x00 0x00
    //   [16:]   AES-128-CTR encrypted Data proto

    const uint32_t fromNode  = 0xDEADBEEF;
    const uint32_t to        = 0xFFFFFFFFu;
    const uint32_t packetId  = 0x01234567;
    const char     text[]    = "Hello, mesh!";

    // 1. Build Data proto: portnum=1, payload=text
    uint8_t dataProto[64] = {};
    size_t  dataLen = mc_encodeData(dataProto, sizeof(dataProto),
                                    1 /*TEXT_MESSAGE_APP*/,
                                    (const uint8_t*)text, sizeof(text) - 1,
                                    false, 0, 0);
    TEST_ASSERT_GREATER_THAN(0u, dataLen);

    // 2. Encrypt the Data proto with the channel PSK
    uint8_t ciphertext[64] = {};
    TEST_ASSERT_TRUE(mc_channelCrypt(DEFAULT_PSK, packetId, fromNode,
                                     dataProto, dataLen, ciphertext));

    // 3. Build the 16-byte OTA header (plaintext)
    uint8_t ota[256] = {};
    ota[0]  = (uint8_t)(to);          ota[1]  = (uint8_t)(to >> 8);
    ota[2]  = (uint8_t)(to >> 16);    ota[3]  = (uint8_t)(to >> 24);
    ota[4]  = (uint8_t)(fromNode);    ota[5]  = (uint8_t)(fromNode >> 8);
    ota[6]  = (uint8_t)(fromNode>>16);ota[7]  = (uint8_t)(fromNode>>24);
    ota[8]  = (uint8_t)(packetId);    ota[9]  = (uint8_t)(packetId >> 8);
    ota[10] = (uint8_t)(packetId>>16);ota[11] = (uint8_t)(packetId>>24);
    ota[12] = 0x63; // FLAGS_BROADCAST
    ota[13] = 0x08; // DEFAULT_CHAN_HASH
    ota[14] = 0x00; ota[15] = 0x00;
    memcpy(ota + 16, ciphertext, dataLen);
    const size_t otaLen = 16 + dataLen;

    // 4. "Receive" the OTA packet: parse header, decrypt, parse Data proto
    const uint32_t rx_to      = ota[0]  | (uint32_t)ota[1]<<8  | (uint32_t)ota[2]<<16 | (uint32_t)ota[3]<<24;
    const uint32_t rx_from    = ota[4]  | (uint32_t)ota[5]<<8  | (uint32_t)ota[6]<<16 | (uint32_t)ota[7]<<24;
    const uint32_t rx_pktId   = ota[8]  | (uint32_t)ota[9]<<8  | (uint32_t)ota[10]<<16| (uint32_t)ota[11]<<24;
    const uint8_t  rx_chanHash = ota[13];
    const uint8_t* rx_payload = ota + 16;
    const size_t   rx_payLen  = otaLen - 16;

    TEST_ASSERT_EQUAL_UINT32(to,       rx_to);
    TEST_ASSERT_EQUAL_UINT32(fromNode, rx_from);
    TEST_ASSERT_EQUAL_UINT32(packetId, rx_pktId);
    TEST_ASSERT_EQUAL_HEX8(0x08,      rx_chanHash);

    uint8_t plain[64] = {};
    TEST_ASSERT_TRUE(mc_channelCrypt(DEFAULT_PSK, rx_pktId, rx_from,
                                     rx_payload, rx_payLen, plain));

    uint32_t portnum = 0;
    const uint8_t* msgPayload = nullptr;
    size_t msgLen = 0; bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(plain, rx_payLen, portnum, msgPayload, msgLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(1u, portnum);
    TEST_ASSERT_EQUAL_size_t(sizeof(text) - 1, msgLen);
    TEST_ASSERT_EQUAL_MEMORY(text, msgPayload, msgLen);
}

void test_full_ota_pkc_direct_message(void)
{
    // Simulate a PKC DM: Alice → Bob (chanHash = 0x00, to = Bob's node ID)
    const uint32_t aliceNode = 0xAAAA0001;
    const uint32_t bobNode   = 0xBBBB0002;
    const uint32_t packetId  = 0xBEEF0001;
    const char     dmText[]  = "Private DM via PKC";

    // 1. Build Data proto
    uint8_t dataProto[64] = {};
    size_t dataLen = mc_encodeData(dataProto, sizeof(dataProto),
                                   1 /*TEXT_MESSAGE_APP*/,
                                   (const uint8_t*)dmText, sizeof(dmText) - 1,
                                   false, bobNode, 0);
    TEST_ASSERT_GREATER_THAN(0u, dataLen);

    // 2. Alice PKC-encrypts the Data proto
    uint8_t ciphertext[64] = {};
    TEST_ASSERT_TRUE(mc_pkcCrypt(RFC7748_ALICE_PRIV, RFC7748_BOB_PUB,
                                  packetId, aliceNode,
                                  dataProto, dataLen, ciphertext));

    // 3. Build OTA header (chanHash=0x00 marks PKC)
    uint8_t ota[256] = {};
    ota[0]  = (uint8_t)(bobNode);     ota[1]  = (uint8_t)(bobNode >> 8);
    ota[2]  = (uint8_t)(bobNode>>16); ota[3]  = (uint8_t)(bobNode>>24);
    ota[4]  = (uint8_t)(aliceNode);   ota[5]  = (uint8_t)(aliceNode >> 8);
    ota[6]  = (uint8_t)(aliceNode>>16);ota[7] = (uint8_t)(aliceNode>>24);
    ota[8]  = (uint8_t)(packetId);    ota[9]  = (uint8_t)(packetId >> 8);
    ota[10] = (uint8_t)(packetId>>16);ota[11] = (uint8_t)(packetId>>24);
    ota[12] = 0x6B; // FLAGS_UNICAST
    ota[13] = 0x00; // PKC marker (chanHash=0x00)
    ota[14] = 0x00; ota[15] = 0x00;
    memcpy(ota + 16, ciphertext, dataLen);
    const size_t otaLen = 16 + dataLen;

    // 4. Bob "receives" the packet
    const uint32_t rx_from   = ota[4] | (uint32_t)ota[5]<<8 | (uint32_t)ota[6]<<16 | (uint32_t)ota[7]<<24;
    const uint32_t rx_pktId  = ota[8] | (uint32_t)ota[9]<<8 | (uint32_t)ota[10]<<16| (uint32_t)ota[11]<<24;
    const uint8_t  rx_chanHash = ota[13]; // must be 0x00 for PKC

    TEST_ASSERT_EQUAL_HEX8(0x00, rx_chanHash);

    // 5. Bob PKC-decrypts (Bob's private key + Alice's public key)
    uint8_t plain[64] = {};
    TEST_ASSERT_TRUE(mc_pkcCrypt(RFC7748_BOB_PRIV, RFC7748_ALICE_PUB,
                                  rx_pktId, rx_from,
                                  ota + 16, otaLen - 16, plain));

    // 6. Parse the Data proto
    uint32_t portnum = 0;
    const uint8_t* msgPayload = nullptr;
    size_t msgLen = 0; bool wantResp = false;
    TEST_ASSERT_TRUE(mc_parseData(plain, otaLen - 16, portnum, msgPayload, msgLen, wantResp));
    TEST_ASSERT_EQUAL_UINT32(1u, portnum);
    TEST_ASSERT_EQUAL_MEMORY(dmText, msgPayload, sizeof(dmText) - 1);
}

// ─────────────────────────────────────────────────────────────────────────
// 9. Channel hash computation — verify DEFAULT_CHAN_HASH = 0x08
//
// Meshtastic channel hash = xorHash(name) ^ xorHash(PSK)
//   xorHash("LongFast", 8) = 0x4c^0x6f^0x6e^0x67^0x46^0x61^0x73^0x74 = 0x0A
//   xorHash(DEFAULT_PSK, 16) = XOR of all 16 PSK bytes = 0x02
//   hash = 0x0A ^ 0x02 = 0x08
// ─────────────────────────────────────────────────────────────────────────

void test_channel_hash_longfast_name_xor(void)
{
    const char name[] = "LongFast";
    uint8_t xr = 0;
    for (int i = 0; name[i]; i++) xr ^= (uint8_t)name[i];
    TEST_ASSERT_EQUAL_HEX8(0x0A, xr);
}

void test_channel_hash_default_psk_xor(void)
{
    uint8_t xr = 0;
    for (int i = 0; i < 16; i++) xr ^= DEFAULT_PSK[i];
    TEST_ASSERT_EQUAL_HEX8(0x02, xr);
}

void test_channel_hash_combined(void)
{
    const char name[] = "LongFast";
    uint8_t nameXor = 0, pskXor = 0;
    for (int i = 0; name[i]; i++) nameXor ^= (uint8_t)name[i];
    for (int i = 0; i < 16; i++) pskXor  ^= DEFAULT_PSK[i];
    TEST_ASSERT_EQUAL_HEX8(0x08, nameXor ^ pskXor);
}

// ─────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────

int main(void)
{
    UNITY_BEGIN();

    // 1. mc_buildNonce
    RUN_TEST(test_nonce_layout_packetid_le);
    RUN_TEST(test_nonce_layout_middle_zero);
    RUN_TEST(test_nonce_layout_fromnode_le);
    RUN_TEST(test_nonce_layout_trailing_zero);
    RUN_TEST(test_nonce_zero_inputs);
    RUN_TEST(test_nonce_packetid_only);

    // 2. mc_aes128ctr — NIST SP 800-38A
    RUN_TEST(test_aes128ctr_nist_block1);
    RUN_TEST(test_aes128ctr_nist_block2);
    RUN_TEST(test_aes128ctr_decrypt_nist_block1);
    RUN_TEST(test_aes128ctr_roundtrip);
    RUN_TEST(test_aes128ctr_different_nonces_give_different_output);
    RUN_TEST(test_aes128ctr_zero_len_returns_false);

    // 3. mc_aes256ctr — NIST SP 800-38A
    RUN_TEST(test_aes256ctr_nist_block1);
    RUN_TEST(test_aes256ctr_decrypt_nist_block1);
    RUN_TEST(test_aes256ctr_roundtrip_arbitrary);

    // 4. mc_channelCrypt — Meshtastic DEFAULT_PSK
    RUN_TEST(test_channel_crypt_roundtrip_text_message);
    RUN_TEST(test_channel_crypt_ciphertext_differs_from_plaintext);
    RUN_TEST(test_channel_crypt_different_packetids_different_output);
    RUN_TEST(test_channel_crypt_different_fromnodes_different_output);
    RUN_TEST(test_channel_crypt_deterministic);
    RUN_TEST(test_channel_crypt_symmetric_encrypt_equals_decrypt);

    // 5. mc_x25519PublicKey — RFC 7748
    RUN_TEST(test_x25519_alice_public_key);
    RUN_TEST(test_x25519_bob_public_key);
    RUN_TEST(test_x25519_public_key_is_deterministic);
    RUN_TEST(test_x25519_different_privkeys_different_pubkeys);

    // 6. mc_x25519SharedSecret — RFC 7748
    RUN_TEST(test_x25519_shared_secret_alice_to_bob);
    RUN_TEST(test_x25519_shared_secret_bob_to_alice);
    RUN_TEST(test_x25519_shared_secret_commutativity);
    RUN_TEST(test_x25519_shared_secret_is_deterministic);
    RUN_TEST(test_x25519_wrong_remote_key_gives_different_secret);

    // 7. mc_pkcCrypt
    RUN_TEST(test_pkc_crypt_roundtrip_basic);
    RUN_TEST(test_pkc_crypt_ciphertext_differs_from_plaintext);
    RUN_TEST(test_pkc_crypt_different_packetid_different_ciphertext);
    RUN_TEST(test_pkc_crypt_wrong_key_fails_decryption);
    RUN_TEST(test_pkc_crypt_roundtrip_data_proto);

    // 8. Full OTA frame
    RUN_TEST(test_full_ota_channel_text_message);
    RUN_TEST(test_full_ota_pkc_direct_message);

    // 9. Channel hash
    RUN_TEST(test_channel_hash_longfast_name_xor);
    RUN_TEST(test_channel_hash_default_psk_xor);
    RUN_TEST(test_channel_hash_combined);

    return UNITY_END();
}
