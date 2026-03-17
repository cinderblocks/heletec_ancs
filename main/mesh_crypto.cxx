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
 * PLATFORM-FREE: only mbedtls + standard C/C++ headers included.
 * No ESP-IDF, FreeRTOS, or hardware-specific functions.
 *
 * AES-CTR uses mbedtls_aes (hardware-accelerated on ESP32-S3).
 * X25519 uses a direct RFC 7748 §5 Montgomery ladder built on mbedtls_mpi
 * big-integer arithmetic, bypassing mbedtls_ecp entirely.  This avoids
 * version-dependent projective-coordinate and Z-blinding differences in
 * mbedtls's ECP module that produce results not matching the RFC 7748 test
 * vectors on some builds.  No CSPRNG or entropy source is needed.
 *
 * On device:  linked against ESP-IDF's mbedtls component.
 * On host:    linked against ESP-IDF's bundled mbedtls (built from source by
 *             the test CMake project when IDF_PATH is set), or system mbedtls
 *             as a fallback.
 */

#include "mesh_crypto.h"

#include <mbedtls/aes.h>
#include <mbedtls/bignum.h>
#include <mbedtls/platform_util.h>
#include <cstring>

// ── Internal helper: byte-reverse 32 bytes (LE ↔ BE for mbedtls MPI) ──────
// Meshtastic X25519 keys are little-endian (RFC 7748 wire format).
// mbedtls MPI is big-endian. Every key must be reversed on import/export.
static void reverseBytes32(const uint8_t* src, uint8_t* dst)
{
    for (int i = 0; i < 32; i++)
        dst[i] = src[31 - i];
}

// ── x25519_ladder ─────────────────────────────────────────────────────────
// RFC 7748 §5 Montgomery ladder implemented directly using mbedtls_mpi
// big-integer arithmetic.  Bypasses mbedtls_ecp entirely to avoid
// version-dependent projective-coordinate and Z-blinding differences that
// produce results not matching the RFC 7748 test vectors.
//
// Computes X25519(k, u):
//   k     = 32-byte scalar        (little-endian, clamped internally)
//   u     = 32-byte u-coordinate  (little-endian, MSB masked internally)
//   out   = 32-byte result        (little-endian)
//
// The algorithm follows the pseudocode in RFC 7748 §5 verbatim, with all
// intermediate values reduced mod p = 2^255 − 19 after every operation.
static bool x25519_ladder(const uint8_t k_le[32],
                          const uint8_t u_le[32],
                          uint8_t       out_le[32])
{
    // All MPI variables used in the ladder.
    mbedtls_mpi p, a24, uc, sc;              // constants + inputs
    mbedtls_mpi x2, z2, x3, z3;             // ladder state
    mbedtls_mpi A, AA, B, BB, E;             // intermediates (x2 arm)
    mbedtls_mpi C, D, DA, CB;               // intermediates (x3 arm)
    mbedtls_mpi t1, t2;                      // temporaries

    mbedtls_mpi_init(&p);  mbedtls_mpi_init(&a24);
    mbedtls_mpi_init(&uc); mbedtls_mpi_init(&sc);
    mbedtls_mpi_init(&x2); mbedtls_mpi_init(&z2);
    mbedtls_mpi_init(&x3); mbedtls_mpi_init(&z3);
    mbedtls_mpi_init(&A);  mbedtls_mpi_init(&AA);
    mbedtls_mpi_init(&B);  mbedtls_mpi_init(&BB);
    mbedtls_mpi_init(&E);  mbedtls_mpi_init(&C);
    mbedtls_mpi_init(&D);  mbedtls_mpi_init(&DA);
    mbedtls_mpi_init(&CB); mbedtls_mpi_init(&t1);
    mbedtls_mpi_init(&t2);

    bool ok = false;

    do {
        // ── Phase 1: constants ───────────────────────────────────────────
        // p = 2^255 − 19
        if (mbedtls_mpi_lset(&p, 1) != 0) break;
        if (mbedtls_mpi_shift_l(&p, 255) != 0) break;
        if (mbedtls_mpi_sub_int(&p, &p, 19) != 0) break;

        // a24 = (486662 − 2) / 4 = 121665
        if (mbedtls_mpi_lset(&a24, 121665) != 0) break;

        // ── Phase 2: import & prepare inputs ─────────────────────────────
        // Mask the MSB of the u-coordinate (RFC 7748 §5).
        uint8_t uMasked[32];
        memcpy(uMasked, u_le, 32);
        uMasked[31] &= 0x7F;

        uint8_t be[32];
        reverseBytes32(uMasked, be);
        if (mbedtls_mpi_read_binary(&uc, be, 32) != 0) break;
        if (mbedtls_mpi_mod_mpi(&uc, &uc, &p) != 0) break;

        // Clamp the scalar per RFC 7748 §5.
        uint8_t clamped[32];
        memcpy(clamped, k_le, 32);
        clamped[0]  &= 0xF8;   // clear bits 0, 1, 2
        clamped[31] &= 0x7F;   // clear bit 255
        clamped[31] |= 0x40;   // set   bit 254

        reverseBytes32(clamped, be);
        mbedtls_platform_zeroize(clamped, 32);
        if (mbedtls_mpi_read_binary(&sc, be, 32) != 0) break;
        mbedtls_platform_zeroize(be, 32);

        // ── Phase 3: ladder state initialisation ─────────────────────────
        // x_2 = 1,  z_2 = 0   (point at infinity)
        // x_3 = u,  z_3 = 1   (base point / remote public key)
        if (mbedtls_mpi_lset(&x2, 1) != 0) break;
        if (mbedtls_mpi_lset(&z2, 0) != 0) break;
        if (mbedtls_mpi_copy(&x3, &uc) != 0) break;
        if (mbedtls_mpi_lset(&z3, 1) != 0) break;

        // ── Phase 4: main loop — 255 iterations (bit 254 down to 0) ─────
        int swap = 0;
        bool err = false;

        for (int t = 254; t >= 0 && !err; t--)
        {
            const int kt = mbedtls_mpi_get_bit(&sc, t);
            swap ^= kt;
            if (swap) {
                mbedtls_mpi_swap(&x2, &x3);
                mbedtls_mpi_swap(&z2, &z3);
            }
            swap = kt;

            // A  = x_2 + z_2  (mod p)
            if (mbedtls_mpi_add_mpi(&A, &x2, &z2) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&A, &A, &p)   != 0) { err = true; break; }

            // AA = A²  (mod p)
            if (mbedtls_mpi_mul_mpi(&AA, &A, &A)   != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&AA, &AA, &p)  != 0) { err = true; break; }

            // B  = x_2 − z_2  (mod p)
            if (mbedtls_mpi_sub_mpi(&B, &x2, &z2) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&B, &B, &p)   != 0) { err = true; break; }

            // BB = B²  (mod p)
            if (mbedtls_mpi_mul_mpi(&BB, &B, &B)   != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&BB, &BB, &p)  != 0) { err = true; break; }

            // E  = AA − BB  (mod p)
            if (mbedtls_mpi_sub_mpi(&E, &AA, &BB) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&E, &E, &p)   != 0) { err = true; break; }

            // C  = x_3 + z_3  (mod p)
            if (mbedtls_mpi_add_mpi(&C, &x3, &z3) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&C, &C, &p)   != 0) { err = true; break; }

            // D  = x_3 − z_3  (mod p)
            if (mbedtls_mpi_sub_mpi(&D, &x3, &z3) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&D, &D, &p)   != 0) { err = true; break; }

            // DA = D · A  (mod p)
            if (mbedtls_mpi_mul_mpi(&DA, &D, &A)   != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&DA, &DA, &p)  != 0) { err = true; break; }

            // CB = C · B  (mod p)
            if (mbedtls_mpi_mul_mpi(&CB, &C, &B)   != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&CB, &CB, &p)  != 0) { err = true; break; }

            // x_3 = (DA + CB)²  (mod p)
            if (mbedtls_mpi_add_mpi(&t1, &DA, &CB) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&t1, &t1, &p)  != 0) { err = true; break; }
            if (mbedtls_mpi_mul_mpi(&x3, &t1, &t1) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&x3, &x3, &p)  != 0) { err = true; break; }

            // z_3 = x_1 · (DA − CB)²  (mod p)
            if (mbedtls_mpi_sub_mpi(&t1, &DA, &CB) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&t1, &t1, &p)  != 0) { err = true; break; }
            if (mbedtls_mpi_mul_mpi(&t2, &t1, &t1) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&t2, &t2, &p)  != 0) { err = true; break; }
            if (mbedtls_mpi_mul_mpi(&z3, &uc, &t2) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&z3, &z3, &p)  != 0) { err = true; break; }

            // x_2 = AA · BB  (mod p)
            if (mbedtls_mpi_mul_mpi(&x2, &AA, &BB) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&x2, &x2, &p)  != 0) { err = true; break; }

            // z_2 = E · (AA + a24 · E)  (mod p)
            if (mbedtls_mpi_mul_mpi(&t1, &a24, &E) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&t1, &t1, &p)  != 0) { err = true; break; }
            if (mbedtls_mpi_add_mpi(&t1, &AA, &t1) != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&t1, &t1, &p)  != 0) { err = true; break; }
            if (mbedtls_mpi_mul_mpi(&z2, &E, &t1)  != 0) { err = true; break; }
            if (mbedtls_mpi_mod_mpi(&z2, &z2, &p)  != 0) { err = true; break; }
        }
        if (err) break;

        // Final conditional swap (using the last value of swap = k_0).
        if (swap) {
            mbedtls_mpi_swap(&x2, &x3);
            mbedtls_mpi_swap(&z2, &z3);
        }

        // ── Phase 5: affine conversion — result = x_2 · z_2⁻¹ mod p ────
        // z_2 = 0 → point at infinity → result is all-zeros (degenerate but valid).
        if (mbedtls_mpi_cmp_int(&z2, 0) == 0) {
            memset(out_le, 0, 32);
            ok = true;
            break;
        }

        if (mbedtls_mpi_inv_mod(&t1, &z2, &p)  != 0) break;
        if (mbedtls_mpi_mul_mpi(&x2, &x2, &t1) != 0) break;
        if (mbedtls_mpi_mod_mpi(&x2, &x2, &p)  != 0) break;

        // Export: BE → LE wire format.
        uint8_t resBE[32];
        if (mbedtls_mpi_write_binary(&x2, resBE, 32) != 0) break;
        reverseBytes32(resBE, out_le);
        mbedtls_platform_zeroize(resBE, 32);
        ok = true;
    } while (false);

    // ── Phase 6: cleanup ─────────────────────────────────────────────────
    mbedtls_mpi_free(&t2); mbedtls_mpi_free(&t1);
    mbedtls_mpi_free(&CB); mbedtls_mpi_free(&DA);
    mbedtls_mpi_free(&D);  mbedtls_mpi_free(&C);
    mbedtls_mpi_free(&E);  mbedtls_mpi_free(&BB);
    mbedtls_mpi_free(&B);  mbedtls_mpi_free(&AA);
    mbedtls_mpi_free(&A);
    mbedtls_mpi_free(&z3); mbedtls_mpi_free(&x3);
    mbedtls_mpi_free(&z2); mbedtls_mpi_free(&x2);
    mbedtls_mpi_free(&sc); mbedtls_mpi_free(&uc);
    mbedtls_mpi_free(&a24);mbedtls_mpi_free(&p);
    return ok;
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

    // Take a mutable copy — mbedtls_aes_crypt_ctr increments the counter in-place.
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
// X25519(k, 9) — scalar multiply the private key by the Curve25519 base point.
bool mc_x25519PublicKey(const uint8_t privateKey[32], uint8_t publicKeyOut[32])
{
    // RFC 7748 §4.1: the base point u = 9 (little-endian: 0x09, 0x00 … 0x00).
    static const uint8_t BASEPOINT[32] = { 9 };
    return x25519_ladder(privateKey, BASEPOINT, publicKeyOut);
}

// ── mc_x25519SharedSecret ─────────────────────────────────────────────────
// X25519(ourPrivKey, remotePubKey) — scalar multiply for ECDH shared secret.
bool mc_x25519SharedSecret(const uint8_t ourPrivKey[32],
                           const uint8_t remotePubKey[32],
                           uint8_t sharedOut[32])
{
    return x25519_ladder(ourPrivKey, remotePubKey, sharedOut);
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