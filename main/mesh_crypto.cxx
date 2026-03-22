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
 * mesh_crypto.cxx — Meshtastic cryptographic primitive implementations.
 *
 * PLATFORM-FREE: only mbedtls (AES) + standard C/C++ headers.
 * No ESP-IDF, FreeRTOS, or hardware-specific functions.
 *
 * X25519 (PKC key exchange) uses a self-contained implementation derived
 * from TweetNaCl by D.J. Bernstein et al. (public domain).  This avoids
 * platform-specific quirks in various mbedTLS builds.
 *
 * AES-CTR (channel cipher) uses mbedtls_aes.
 * AES-CCM (PKC cipher) uses mbedtls_ccm.
 * SHA-256 (key derivation) uses mbedtls_sha256.
 * All are hardware-accelerated on ESP32-S3.
 *
 * On device:  linked against ESP-IDF's mbedtls component.
 * On host:    linked against system mbedtls (brew install mbedtls).
 */

#include "mesh_crypto.h"

#include <mbedtls/aes.h>
#include <mbedtls/ccm.h>
#include <mbedtls/gcm.h>
#include <mbedtls/sha256.h>
#include <mbedtls/platform_util.h>
#include <mbedtls/ecp.h>
#include <mbedtls/bignum.h>
#include <cstring>
#include <cstdint>

// ═════════════════════════════════════════════════════════════════════════
// Standalone X25519 — derived from TweetNaCl (public domain)
//
// Field element: 16 limbs of int64_t, each holding ≤ 16 bits of value.
// All arithmetic is in GF(2^255 − 19).
//
// This implementation is constant-time and produces correct results for
// all RFC 7748 test vectors on every platform.
// ═════════════════════════════════════════════════════════════════════════

typedef int64_t gf[16];

static void _car25519(gf o)
{
    for (int i = 0; i < 16; i++) {
        o[(i + 1) % 16] += (i < 15 ? 1 : 38) * (o[i] >> 16);
        o[i] &= 0xffff;
    }
}

static void _sel25519(gf p, gf q, int b)
{
    int64_t c = ~(static_cast<int64_t>(b) - 1);
    for (int i = 0; i < 16; i++) {
        int64_t t = c & (p[i] ^ q[i]);
        p[i] ^= t;
        q[i] ^= t;
    }
}

static void _pack25519(uint8_t o[32], const gf n)
{
    gf m, t;
    for (int i = 0; i < 16; i++) t[i] = n[i];
    _car25519(t); _car25519(t); _car25519(t);
    for (int j = 0; j < 2; j++) {
        m[0] = t[0] - 0xffed;
        for (int i = 1; i < 15; i++) {
            m[i] = t[i] - 0xffff - ((m[i - 1] >> 16) & 1);
            m[i - 1] &= 0xffff;
        }
        m[15] = t[15] - 0x7fff - ((m[14] >> 16) & 1);
        int64_t b = (m[15] >> 16) & 1;
        m[14] &= 0xffff;
        _sel25519(t, m, 1 - static_cast<int>(b));
    }
    for (int i = 0; i < 16; i++) {
        o[2 * i]     = static_cast<uint8_t>(t[i] & 0xff);
        o[2 * i + 1] = static_cast<uint8_t>(t[i] >> 8);
    }
}

static void _unpack25519(gf o, const uint8_t n[32])
{
    for (int i = 0; i < 16; i++)
        o[i] = n[2 * i] + (static_cast<int64_t>(n[2 * i + 1]) << 8);
    o[15] &= 0x7fff;
}

static void _fadd(gf o, const gf a, const gf b)
{
    for (int i = 0; i < 16; i++) o[i] = a[i] + b[i];
}

static void _fsub(gf o, const gf a, const gf b)
{
    for (int i = 0; i < 16; i++) o[i] = a[i] - b[i];
}

static void _fmul(gf o, const gf a, const gf b)
{
    int64_t t[31] = {};
    for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
            t[i + j] += a[i] * b[j];
    for (int i = 16; i < 31; i++)
        t[i - 16] += 38 * t[i];
    for (int i = 0; i < 16; i++) o[i] = t[i];
    _car25519(o);
    _car25519(o);
}

static void _fsq(gf o, const gf a) { _fmul(o, a, a); }

static void _finv(gf o, const gf inp)
{
    gf c;
    for (int i = 0; i < 16; i++) c[i] = inp[i];
    for (int a = 253; a >= 0; a--) {
        _fsq(c, c);
        if (a != 2 && a != 4) _fmul(c, c, inp);
    }
    for (int i = 0; i < 16; i++) o[i] = c[i];
}

// RFC 7748 X25519 scalar multiplication: q = clamp(n) * p
static void _scalarmult(uint8_t q[32], const uint8_t n[32], const uint8_t p[32])
{
    uint8_t z[32];
    memcpy(z, n, 32);
    z[0]  &= 248;
    z[31] &= 127;
    z[31] |= 64;

    gf x, a, b, c, d, e, f;
    _unpack25519(x, p);
    for (int i = 0; i < 16; i++) {
        b[i] = x[i];
        a[i] = c[i] = d[i] = 0;
    }
    a[0] = d[0] = 1;

    static const gf _121665 = {0xDB41, 1};

    for (int i = 254; i >= 0; --i) {
        int64_t r = (z[i >> 3] >> (i & 7)) & 1;
        _sel25519(a, b, static_cast<int>(r));
        _sel25519(c, d, static_cast<int>(r));
        _fadd(e, a, c);
        _fsub(a, a, c);
        _fadd(c, b, d);
        _fsub(b, b, d);
        _fsq(d, e);
        _fsq(f, a);
        _fmul(a, c, a);
        _fmul(c, b, e);
        _fadd(e, a, c);
        _fsub(a, a, c);
        _fsq(b, a);
        _fsub(c, d, f);
        _fmul(a, c, _121665);
        _fadd(a, a, d);
        _fmul(c, c, a);
        _fmul(a, d, f);
        _fmul(d, b, x);
        _fsq(b, e);
        _sel25519(a, b, static_cast<int>(r));
        _sel25519(c, d, static_cast<int>(r));
    }

    _finv(c, c);
    _fmul(a, a, c);
    _pack25519(q, a);
}

// ── mc_buildNonce ─────────────────────────────────────────────────────────
void mc_buildNonce(uint32_t packetId, uint32_t fromNode, uint8_t nonce[16])
{
    memset(nonce, 0, 16);
    nonce[0]  = static_cast<uint8_t>(packetId);
    nonce[1]  = static_cast<uint8_t>(packetId >>  8);
    nonce[2]  = static_cast<uint8_t>(packetId >> 16);
    nonce[3]  = static_cast<uint8_t>(packetId >> 24);
    // [4:7] = 0
    nonce[8]  = static_cast<uint8_t>(fromNode);
    nonce[9]  = static_cast<uint8_t>(fromNode >>  8);
    nonce[10] = static_cast<uint8_t>(fromNode >> 16);
    nonce[11] = static_cast<uint8_t>(fromNode >> 24);
    // [12:15] = 0
}

// ── mc_aes128ctr ──────────────────────────────────────────────────────────
bool mc_aes128ctr(const uint8_t key[16], const uint8_t nonce_in[16],
                  const uint8_t* in, size_t len, uint8_t* out)
{
    if (len == 0) return false;

    uint8_t nonce[16];
    memcpy(nonce, nonce_in, 16);

    uint8_t stream_block[16] = {};
    size_t  nc_off = 0;

    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    bool ok = false;
    if (mbedtls_aes_setkey_enc(&ctx, key, 128) == 0)
        ok = (mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce, stream_block, in, out) == 0);
    mbedtls_aes_free(&ctx);
    mbedtls_platform_zeroize(stream_block, sizeof(stream_block));
    return ok;
}

// ── mc_aes256ctr ──────────────────────────────────────────────────────────
bool mc_aes256ctr(const uint8_t key[32], const uint8_t nonce_in[16],
                  const uint8_t* in, size_t len, uint8_t* out)
{
    if (len == 0) return false;

    uint8_t nonce[16];
    memcpy(nonce, nonce_in, 16);

    uint8_t stream_block[16] = {};
    size_t  nc_off = 0;

    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    bool ok = false;
    if (mbedtls_aes_setkey_enc(&ctx, key, 256) == 0)
        ok = (mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce, stream_block, in, out) == 0);
    mbedtls_aes_free(&ctx);
    mbedtls_platform_zeroize(stream_block, sizeof(stream_block));
    return ok;
}

// ── mc_channelCrypt ───────────────────────────────────────────────────────
bool mc_channelCrypt(const uint8_t psk[16],
                     uint32_t packetId, uint32_t fromNode,
                     const uint8_t* in, size_t len, uint8_t* out)
{
    uint8_t nonce[16];
    mc_buildNonce(packetId, fromNode, nonce);
    return mc_aes128ctr(psk, nonce, in, len, out);
}

// ── mc_x25519PublicKey ────────────────────────────────────────────────────
bool mc_x25519PublicKey(const uint8_t privateKey[32], uint8_t publicKeyOut[32])
{
    static const uint8_t basepoint[32] = {9};
    _scalarmult(publicKeyOut, privateKey, basepoint);
    return true;
}

// ── mc_x25519SharedSecret ─────────────────────────────────────────────────
bool mc_x25519SharedSecret(const uint8_t ourPrivKey[32],
                           const uint8_t remotePubKey[32],
                           uint8_t sharedOut[32])
{
    _scalarmult(sharedOut, ourPrivKey, remotePubKey);
    // Reject the all-zero shared secret (low-order point input)
    uint8_t z = 0;
    for (int i = 0; i < 32; i++) z |= sharedOut[i];
    return z != 0;
}

// ── mc_x25519SharedSecret_alt — mbedtls ECP Curve25519 cross-check ────────
// Returns 0 on success, or negative mbedtls error code.
// Montgomery curves in mbedtls use raw LE coordinates (no 0x04 prefix).
// f_rng is needed by mbedtls_ecp_mul for side-channel blinding on ESP32.
int mc_x25519SharedSecret_alt(const uint8_t ourPrivKey[32],
                              const uint8_t remotePubKey[32],
                              uint8_t sharedOut[32],
                              int (*f_rng)(void*, unsigned char*, size_t),
                              void* p_rng)
{
    // Clamp private key per RFC 7748
    uint8_t clamped[32];
    memcpy(clamped, ourPrivKey, 32);
    clamped[0]  &= 248;
    clamped[31] &= 127;
    clamped[31] |= 64;

    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q, result;
    mbedtls_mpi d;

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&Q);
    mbedtls_ecp_point_init(&result);
    mbedtls_mpi_init(&d);

    int ret;

    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519);
    if (ret != 0) { ret = ret - 1000000; goto cleanup; }  // tag: -100xxxx

    // Read remote public key — Montgomery: raw 32-byte LE
    ret = mbedtls_ecp_point_read_binary(&grp, &Q, remotePubKey, 32);
    if (ret != 0) { ret = ret - 2000000; goto cleanup; }  // tag: -200xxxx

    // Read clamped private key scalar (LE)
    ret = mbedtls_mpi_read_binary_le(&d, clamped, 32);
    if (ret != 0) { ret = ret - 3000000; goto cleanup; }  // tag: -300xxxx

    // Compute: result = d * Q on Curve25519
    ret = mbedtls_ecp_mul(&grp, &result, &d, &Q, f_rng, p_rng);
    if (ret != 0) { ret = ret - 4000000; goto cleanup; }  // tag: -400xxxx

    // Write result X-coordinate in LE
    {
        uint8_t resBuf[32];
        size_t olen = 0;
        ret = mbedtls_ecp_point_write_binary(&grp, &result,
                MBEDTLS_ECP_PF_COMPRESSED, &olen, resBuf, sizeof(resBuf));
        if (ret != 0) { ret = ret - 5000000; goto cleanup; }  // tag: -500xxxx
        if (olen != 32) { ret = -6000000; goto cleanup; }     // tag: -6000000
        memcpy(sharedOut, resBuf, 32);
    }

    ret = 0;

cleanup:
    mbedtls_platform_zeroize(clamped, sizeof(clamped));
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&result);
    mbedtls_ecp_point_free(&Q);
    mbedtls_ecp_group_free(&grp);
    return ret;
}

// ── mc_sha256 ─────────────────────────────────────────────────────────────
bool mc_sha256(const uint8_t* data, size_t len, uint8_t hash[32])
{
    return mbedtls_sha256(data, len, hash, 0 /*not SHA-224*/) == 0;
}

// ── PKC helpers ───────────────────────────────────────────────────────────
// Build the 13-byte CCM nonce from the 16-byte Meshtastic nonce layout.
// CCM with L=2 uses nonce of 15−L = 13 bytes.
static void _buildPkcNonce(uint32_t packetId, uint32_t fromNode,
                           uint32_t extraNonce, uint8_t ccmNonce[13])
{
    uint8_t full[16];
    mc_buildNonce(packetId, fromNode, full);
    // Set extraNonce in bytes [12:15] of the full nonce
    full[12] = static_cast<uint8_t>(extraNonce);
    full[13] = static_cast<uint8_t>(extraNonce >>  8);
    full[14] = static_cast<uint8_t>(extraNonce >> 16);
    full[15] = static_cast<uint8_t>(extraNonce >> 24);
    // CCM L=2 → nonce is first 13 bytes
    memcpy(ccmNonce, full, 13);
}

// Derive AES-256 key: ECDH → SHA-256(shared_secret)
static bool _derivePkcKey(const uint8_t ourPrivKey[32],
                          const uint8_t remotePubKey[32],
                          uint8_t keyOut[32])
{
    uint8_t shared[32] = {};
    if (!mc_x25519SharedSecret(ourPrivKey, remotePubKey, shared))
    {
        mbedtls_platform_zeroize(shared, 32);
        return false;
    }
    // Meshtastic: crypto->hash(shared_key, 32) — SHA-256 of raw ECDH output
    const bool ok = mc_sha256(shared, 32, keyOut);
    mbedtls_platform_zeroize(shared, 32);
    return ok;
}

// ── mc_pkcEncrypt — AES-256-CCM (M=8, L=2) ──────────────────────────────
// Meshtastic-standard PKC wire format:
//   Wire = [ciphertext(N)] [8-byte CCM tag]
//   Nonce = [packetId(8) | fromNode(4) | 0x00] — 13 bytes, byte 12 = 0
//   Overhead = 8 bytes (MC_PKC_OVERHEAD)
//
// NOTE: The toNode parameter is currently unused for the nonce (stock
// Meshtastic firmware uses nonce[12]=0).  Retained in the API for
// potential future firmware variants that may use toNode in the nonce.
bool mc_pkcEncrypt(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                   uint32_t packetId, uint32_t fromNode, uint32_t toNode,
                   const uint8_t* in, size_t len, uint8_t* out)
{
    (void)toNode;  // Stock Meshtastic uses nonce[12]=0

    uint8_t key[32] = {};
    if (!_derivePkcKey(ourPrivKey, remotePubKey, key))
        return false;

    uint8_t ccmNonce[13];
    _buildPkcNonce(packetId, fromNode, /*extraNonce=*/0, ccmNonce);

    // out layout: [ciphertext (len)] [8-byte CCM tag]
    uint8_t tag[8] = {};

    mbedtls_ccm_context ccm;
    mbedtls_ccm_init(&ccm);
    bool ok = false;
    if (mbedtls_ccm_setkey(&ccm, MBEDTLS_CIPHER_ID_AES, key, 256) == 0)
    {
        ok = (mbedtls_ccm_encrypt_and_tag(&ccm,
                len,                    // plaintext length
                ccmNonce, 13,           // nonce (13 bytes → L=2)
                nullptr, 0,             // no AAD
                in, out,                // plaintext → ciphertext
                tag, 8) == 0);         // M=8 tag
    }
    mbedtls_ccm_free(&ccm);
    mbedtls_platform_zeroize(key, 32);

    if (ok)
    {
        // Append tag after ciphertext (no extraNonce — Meshtastic standard)
        memcpy(out + len, tag, 8);
    }
    return ok;
}

// ── mc_pkcDecrypt — AES-256-CCM (M=8, L=2) ──────────────────────────────
// Meshtastic-standard PKC wire format:
//   Wire = [ciphertext(N)] [8-byte CCM tag]
//   Nonce = [packetId(8) | fromNode(4) | 0x00] — 13 bytes, byte 12 = 0
//   Overhead = 8 bytes
bool mc_pkcDecrypt(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                   uint32_t packetId, uint32_t fromNode,
                   const uint8_t* in, size_t len, uint8_t* out)
{
    static constexpr size_t TAG_SIZE = 8;
    if (len <= TAG_SIZE) return false;

    const size_t cipherLen = len - TAG_SIZE;
    const uint8_t* tag     = in + cipherLen;

    uint8_t key[32] = {};
    if (!_derivePkcKey(ourPrivKey, remotePubKey, key))
        return false;

    uint8_t ccmNonce[13];
    _buildPkcNonce(packetId, fromNode, /*extraNonce=*/0, ccmNonce);

    mbedtls_ccm_context ccm;
    mbedtls_ccm_init(&ccm);
    bool ok = false;
    if (mbedtls_ccm_setkey(&ccm, MBEDTLS_CIPHER_ID_AES, key, 256) == 0)
    {
        ok = (mbedtls_ccm_auth_decrypt(&ccm,
                cipherLen,              // ciphertext length (without tag)
                ccmNonce, 13,           // nonce (13 bytes → L=2)
                nullptr, 0,             // no AAD
                in, out,                // ciphertext → plaintext
                tag, TAG_SIZE) == 0);   // verify 8-byte tag
    }
    mbedtls_ccm_free(&ccm);
    mbedtls_platform_zeroize(key, 32);
    return ok;
}

// ── mc_pkcDecryptCcmEx — flexible AES-256-CCM decrypt ─────────────────────
// Wire layout: [ciphertext(wireLen - tagSize)] [tag(tagSize)].
// ExtraNonce is provided explicitly (not extracted from wire).
bool mc_pkcDecryptCcmEx(const uint8_t key[32],
                        uint32_t packetId, uint32_t fromNode,
                        uint32_t extraNonce,
                        const uint8_t* in, size_t wireLen,
                        uint8_t* out, size_t tagSize)
{
    if (wireLen <= tagSize) return false;
    if (tagSize != 4 && tagSize != 6 && tagSize != 8 &&
        tagSize != 10 && tagSize != 12 && tagSize != 14 && tagSize != 16)
        return false;

    const size_t cipherLen = wireLen - tagSize;
    const uint8_t* tag     = in + cipherLen;

    uint8_t ccmNonce[13];
    _buildPkcNonce(packetId, fromNode, extraNonce, ccmNonce);

    mbedtls_ccm_context ccm;
    mbedtls_ccm_init(&ccm);
    bool ok = false;
    if (mbedtls_ccm_setkey(&ccm, MBEDTLS_CIPHER_ID_AES, key, 256) == 0)
    {
        ok = (mbedtls_ccm_auth_decrypt(&ccm,
                cipherLen,
                ccmNonce, 13,
                nullptr, 0,
                in, out,
                tag, tagSize) == 0);
    }
    mbedtls_ccm_free(&ccm);
    return ok;
}

// ── mc_pkcDecryptCcmFlexAad — CCM with variable nonce length + optional AAD ─
// Accepts pre-built nonce of any valid CCM length (7-13 bytes).
// Wire = [ciphertext(wireLen - tagSize)] [tag(tagSize)].
bool mc_pkcDecryptCcmFlexAad(const uint8_t key[32],
                             const uint8_t* nonce, size_t nonceLen,
                             const uint8_t* aad, size_t aadLen,
                             const uint8_t* in, size_t wireLen,
                             uint8_t* out, size_t tagSize)
{
    if (wireLen <= tagSize) return false;
    if (nonceLen < 7 || nonceLen > 13) return false;
    if (aadLen != 0 && aad == nullptr) return false;

    const size_t cipherLen = wireLen - tagSize;
    const uint8_t* tag     = in + cipherLen;

    mbedtls_ccm_context ccm;
    mbedtls_ccm_init(&ccm);
    bool ok = false;
    if (mbedtls_ccm_setkey(&ccm, MBEDTLS_CIPHER_ID_AES, key, 256) == 0)
    {
        ok = (mbedtls_ccm_auth_decrypt(&ccm,
                cipherLen,
                nonce, nonceLen,
                aad, aadLen,
                in, out,
                tag, tagSize) == 0);
    }
    mbedtls_ccm_free(&ccm);
    return ok;
}

// ── mc_pkcDecryptCcmFlex — CCM with variable nonce length ─────────────────
// Accepts pre-built nonce of any valid CCM length (7-13 bytes).
// Wire = [ciphertext(wireLen - tagSize)] [tag(tagSize)].
bool mc_pkcDecryptCcmFlex(const uint8_t key[32],
                          const uint8_t* nonce, size_t nonceLen,
                          const uint8_t* in, size_t wireLen,
                          uint8_t* out, size_t tagSize)
{
    return mc_pkcDecryptCcmFlexAad(key, nonce, nonceLen,
                                   nullptr, 0,
                                   in, wireLen, out, tagSize);
}

// ── mc_pkcDecryptGcm — AES-256-GCM decrypt ───────────────────────────────
// Wire = [ciphertext(wireLen - tagSize)] [tag(tagSize)].
// IV is typically 12 bytes for GCM.
bool mc_pkcDecryptGcm(const uint8_t key[32],
                      const uint8_t* iv, size_t ivLen,
                      const uint8_t* in, size_t wireLen,
                      uint8_t* out, size_t tagSize)
{
    if (wireLen <= tagSize) return false;

    const size_t cipherLen = wireLen - tagSize;
    const uint8_t* tag     = in + cipherLen;

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    bool ok = false;
    if (mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, 256) == 0)
    {
        ok = (mbedtls_gcm_auth_decrypt(&gcm,
                cipherLen,
                iv, ivLen,
                nullptr, 0,     // no AAD
                tag, tagSize,
                in, out) == 0);
    }
    mbedtls_gcm_free(&gcm);
    return ok;
}

// ── mc_pkcDecryptCtr — AES-256-CTR with 4-byte extraNonce ────────────────
// Fallback for firmware that uses CTR instead of CCM for PKC.
// Wire layout: [ciphertext(len-4)] [4-byte extraNonce].
// No authentication tag — caller MUST validate decrypted output.
bool mc_pkcDecryptCtr(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                      uint32_t packetId, uint32_t fromNode,
                      const uint8_t* in, size_t len, uint8_t* out)
{
    if (len <= 4) return false;

    const size_t plainLen = len - 4;
    const uint8_t* extraBytes = in + plainLen;
    const uint32_t extraNonce = (uint32_t)extraBytes[0]
                              | (uint32_t)extraBytes[1] <<  8
                              | (uint32_t)extraBytes[2] << 16
                              | (uint32_t)extraBytes[3] << 24;

    uint8_t key[32] = {};
    if (!_derivePkcKey(ourPrivKey, remotePubKey, key))
        return false;

    // Full 16-byte CTR nonce: [packetId(4)|0(4)|fromNode(4)|extraNonce(4)]
    uint8_t nonce[16];
    mc_buildNonce(packetId, fromNode, nonce);
    nonce[12] = static_cast<uint8_t>(extraNonce);
    nonce[13] = static_cast<uint8_t>(extraNonce >>  8);
    nonce[14] = static_cast<uint8_t>(extraNonce >> 16);
    nonce[15] = static_cast<uint8_t>(extraNonce >> 24);

    const bool ok = mc_aes256ctr(key, nonce, in, plainLen, out);
    mbedtls_platform_zeroize(key, 32);
    return ok;
}

// ── mc_pkcDecryptCtrRaw — AES-256-CTR with no extraNonce ─────────────────
// Wire payload is entirely ciphertext.  Nonce = [pktId|0|fromNode|0].
bool mc_pkcDecryptCtrRaw(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                         uint32_t packetId, uint32_t fromNode,
                         const uint8_t* in, size_t len, uint8_t* out)
{
    if (len == 0) return false;

    uint8_t key[32] = {};
    if (!_derivePkcKey(ourPrivKey, remotePubKey, key))
        return false;

    uint8_t nonce[16];
    mc_buildNonce(packetId, fromNode, nonce);
    // nonce[12:15] remain 0 — no extraNonce

    const bool ok = mc_aes256ctr(key, nonce, in, len, out);
    mbedtls_platform_zeroize(key, 32);
    return ok;
}
