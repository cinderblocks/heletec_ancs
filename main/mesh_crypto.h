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
 * mesh_crypto.h — Meshtastic cryptographic primitives, zero platform deps.
 *
 * X25519:   standalone implementation derived from TweetNaCl (public domain).
 * AES-CTR:  mbedtls_aes (hardware-accelerated on ESP32-S3) — channel cipher.
 * AES-CCM:  mbedtls_ccm — PKI/PKC direct-message cipher.
 * SHA-256:  mbedtls_sha256 — key derivation for PKC.
 *
 * No ESP-IDF, FreeRTOS, or hardware-specific headers.
 *
 * This file exists so that every crypto operation can be unit-tested on the
 * host with known RFC 7748 / NIST SP 800-38A test vectors, independently
 * of the hardware LoRa layer.
 *
 * Firmware call-sites in meshtastic_proto.cxx and meshnode.cxx delegate to
 * these free functions; the LoRa class itself is never directly tested.
 */

#pragma once

#include <cstdint>
#include <cstddef>

// ── Nonce ─────────────────────────────────────────────────────────────────

/**
 * Build the 16-byte Meshtastic nonce (initNonce / initCounter).
 *
 *   bytes  [0:3]  = packetId, little-endian  (low 4 bytes of uint64_t)
 *   bytes  [4:7]  = 0x00 × 4                 (high 4 bytes — packetId fits in 32 bits)
 *   bytes  [8:11] = fromNode, little-endian
 *   bytes [12:15] = 0x00 × 4                 (channel) or extraNonce (PKC)
 *
 * For channel encryption (AES-CTR), all 16 bytes are used as the counter.
 * For PKC (AES-CCM with L=2), only the first 13 bytes (15−L) are the nonce;
 * bytes [12:15] are still filled so callers can set extraNonce before
 * truncating to 13 for CCM.
 */
void mc_buildNonce(uint32_t packetId, uint32_t fromNode, uint8_t nonce[16]);

// ── SHA-256 ───────────────────────────────────────────────────────────────

/**
 * Compute SHA-256(data, len) → 32-byte hash.
 * Used to derive the AES key from the X25519 shared secret for PKC.
 */
bool mc_sha256(const uint8_t* data, size_t len, uint8_t hash[32]);

// ── AES-CTR ciphers ───────────────────────────────────────────────────────

/**
 * AES-128-CTR encrypt / decrypt (CTR is symmetric — same function for both).
 */
bool mc_aes128ctr(const uint8_t key[16], const uint8_t nonce_in[16],
                  const uint8_t* in, size_t len, uint8_t* out);

/**
 * AES-256-CTR encrypt / decrypt.
 */
bool mc_aes256ctr(const uint8_t key[32], const uint8_t nonce_in[16],
                  const uint8_t* in, size_t len, uint8_t* out);

/**
 * Meshtastic channel cipher — AES-128-CTR using the standard Meshtastic
 * nonce layout.  Combines mc_buildNonce() + mc_aes128ctr().
 *
 * @param psk      16-byte channel PSK (DEFAULT_PSK for the LongFast channel).
 * @param packetId From the 4-byte OTA header field bytes [8:11].
 * @param fromNode From the 4-byte OTA header field bytes [4:7].
 */
bool mc_channelCrypt(const uint8_t psk[16],
                     uint32_t packetId, uint32_t fromNode,
                     const uint8_t* in, size_t len, uint8_t* out);

// ── X25519 key exchange ───────────────────────────────────────────────────

/**
 * Derive an X25519 public key from a private key.
 * Keys are in little-endian RFC 7748 wire format (32 bytes).
 */
bool mc_x25519PublicKey(const uint8_t privateKey[32], uint8_t publicKeyOut[32]);

/**
 * Compute an X25519 ECDH shared secret.
 * Keys are in little-endian RFC 7748 wire format.
 */
bool mc_x25519SharedSecret(const uint8_t ourPrivKey[32],
                           const uint8_t remotePubKey[32],
                           uint8_t sharedOut[32]);

/**
 * Alternative X25519 using mbedtls ECP (for cross-checking TweetNaCl).
 * @return 0 on success, or a negative mbedtls error code.
 */
int mc_x25519SharedSecret_alt(const uint8_t ourPrivKey[32],
                              const uint8_t remotePubKey[32],
                              uint8_t sharedOut[32],
                              int (*f_rng)(void*, unsigned char*, size_t) = nullptr,
                              void* p_rng = nullptr);

// ── PKC direct-message cipher (AES-256-CCM) ──────────────────────────────
//
// Meshtastic PKI/PKC encryption (firmware v2.5+):
//   1. ECDH shared secret = X25519(ourPriv, remotePub)
//   2. AES key = SHA-256(shared secret)          ← 32 bytes
//   3. Nonce   = [packetId(8), fromNode(4), extraNonce_low_byte(1)]
//                (13 bytes for CCM L=2, from the 16-byte initNonce layout)
//   4. AES-256-CCM encrypt with M=8 (8-byte auth tag)
//   5. Wire payload = [ciphertext] [8-byte tag] [4-byte extraNonce]
//
// Total overhead appended to plaintext: MESHTASTIC_PKC_OVERHEAD = 12 bytes
//   (8-byte CCM auth tag + 4-byte extraNonce sent in the clear)
//
// The extraNonce is the receiver's node ID (toNode).  On the wire the last
// 4 bytes of the encrypted payload carry this value so the receiver can
// reconstruct the nonce.

/// Total bytes appended beyond the plaintext: 8 (CCM tag) + 4 (extraNonce).
static constexpr size_t MC_PKC_OVERHEAD = 12;

/// PKC overhead for CTR mode (older firmware): just 4-byte extraNonce, no tag.
static constexpr size_t MC_PKC_CTR_OVERHEAD = 4;

/**
 * PKC encrypt: ECDH + SHA-256 + AES-256-CCM.
 *
 * @param out  Output buffer — must hold at least len + MC_PKC_OVERHEAD bytes.
 *             Layout: [ciphertext (len)] [8-byte CCM tag] [4-byte extraNonce LE]
 */
bool mc_pkcEncrypt(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                   uint32_t packetId, uint32_t fromNode, uint32_t toNode,
                   const uint8_t* in, size_t len, uint8_t* out);

/**
 * PKC decrypt: ECDH + SHA-256 + AES-256-CCM (M=8, L=2).
 * Wire layout: [ciphertext] [8-byte CCM tag] [4-byte extraNonce].
 * ExtraNonce is extracted from the last 4 bytes of `in`.
 */
bool mc_pkcDecrypt(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                   uint32_t packetId, uint32_t fromNode,
                   const uint8_t* in, size_t len, uint8_t* out);

/**
 * Flexible PKC decrypt: AES-256-CCM with explicit key, extraNonce, and tag size.
 *
 * Wire layout: [ciphertext(wireLen - tagSize)] [CCM tag(tagSize)].
 * No extraNonce on the wire — caller provides it explicitly (e.g. toNode
 * from the OTA header).
 *
 * @param key        Pre-derived 32-byte AES key (SHA-256 of ECDH, or raw ECDH).
 * @param packetId   Packet ID from the OTA header.
 * @param fromNode   Sender node ID (goes into nonce[8:11]).
 * @param extraNonce Value for nonce[12:15] — typically the receiver's node ID.
 * @param in         Wire payload: [ciphertext][tag].
 * @param wireLen    Total bytes of `in`.
 * @param out        Output buffer for plaintext (wireLen - tagSize bytes).
 * @param tagSize    CCM auth tag size in bytes (4 or 8).
 * @return true if CCM authentication succeeds.
 */
bool mc_pkcDecryptCcmEx(const uint8_t key[32],
                        uint32_t packetId, uint32_t fromNode,
                        uint32_t extraNonce,
                        const uint8_t* in, size_t wireLen,
                        uint8_t* out, size_t tagSize);

/**
 * CCM decrypt with pre-built nonce of variable length (7-13 bytes).
 * Allows testing L=2 (13-byte nonce) vs L=3 (12-byte nonce) etc.
 * Wire = [ciphertext(wireLen - tagSize)] [tag(tagSize)].
 */
bool mc_pkcDecryptCcmFlex(const uint8_t key[32],
                          const uint8_t* nonce, size_t nonceLen,
                          const uint8_t* in, size_t wireLen,
                          uint8_t* out, size_t tagSize);

/**
 * AES-256-GCM decrypt with variable IV length and tag size.
 * Wire = [ciphertext(wireLen - tagSize)] [tag(tagSize)].
 */
bool mc_pkcDecryptGcm(const uint8_t key[32],
                      const uint8_t* iv, size_t ivLen,
                      const uint8_t* in, size_t wireLen,
                      uint8_t* out, size_t tagSize);/**
 * PKC decrypt — AES-256-CTR fallback for older Meshtastic firmware.
 *
 * Some firmware versions (pre-2.5) use AES-256-CTR instead of AES-256-CCM
 * for PKC direct messages.  Wire layout: [ciphertext] [4-byte extraNonce].
 * No authentication tag — caller must validate the decrypted output.
 *
 * @param in   Wire payload: [ciphertext] [4-byte extraNonce].
 * @param len  Total input length INCLUDING the 4-byte extraNonce.
 *             Must be > MC_PKC_CTR_OVERHEAD (4).
 * @param out  Output buffer — must hold at least (len - 4) bytes.
 * @return true if decryption was performed (no auth check — validate output!).
 */
bool mc_pkcDecryptCtr(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                      uint32_t packetId, uint32_t fromNode,
                      const uint8_t* in, size_t len, uint8_t* out);

/**
 * PKC decrypt — AES-256-CTR with no extraNonce overhead.
 *
 * Fallback for firmware that uses CTR with nonce=[pktId|0|fromNode|0],
 * with no extraNonce appended to the wire payload.
 * Full wire payload is ciphertext.
 *
 * @param in   Wire payload (all ciphertext).
 * @param len  Total input length = plaintext length.
 * @param out  Output buffer — must hold at least len bytes.
 * @return true if decryption was performed.
 */
bool mc_pkcDecryptCtrRaw(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                         uint32_t packetId, uint32_t fromNode,
                         const uint8_t* in, size_t len, uint8_t* out);
