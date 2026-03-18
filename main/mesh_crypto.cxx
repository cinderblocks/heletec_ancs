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
 * AES-CTR (channel + PKC ciphers) uses mbedtls_aes, which is
 * hardware-accelerated on ESP32-S3.
 *
 * On device:  linked against ESP-IDF's mbedtls component.
 * On host:    linked against system mbedtls (brew install mbedtls).
 */

#include "mesh_crypto.h"

#include <mbedtls/aes.h>
#include <mbedtls/platform_util.h>
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

// ── mc_pkcCrypt ───────────────────────────────────────────────────────────
bool mc_pkcCrypt(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                 uint32_t packetId, uint32_t fromNode,
                 const uint8_t* in, size_t len, uint8_t* out)
{
    uint8_t sharedSecret[32] = {};
    if (!mc_x25519SharedSecret(ourPrivKey, remotePubKey, sharedSecret))
    {
        mbedtls_platform_zeroize(sharedSecret, 32);
        return false;
    }

    uint8_t nonce[16];
    mc_buildNonce(packetId, fromNode, nonce);

    const bool ok = mc_aes256ctr(sharedSecret, nonce, in, len, out);
    mbedtls_platform_zeroize(sharedSecret, 32);
    return ok;
}
