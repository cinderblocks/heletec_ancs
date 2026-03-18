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
 * X25519:  standalone implementation derived from TweetNaCl (public domain).
 * AES-CTR: mbedtls_aes (hardware-accelerated on ESP32-S3).
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
 * Build the 16-byte Meshtastic CTR nonce used for both channel (AES-128)
 * and PKC (AES-256) encryption.
 *
 * Layout (CryptoEngine::initCounter() in Meshtastic firmware):
 *   bytes  [0:3]  = packetId, little-endian
 *   bytes  [4:7]  = 0x00 × 4
 *   bytes  [8:11] = fromNode, little-endian
 *   bytes [12:15] = 0x00 × 4
 */
void mc_buildNonce(uint32_t packetId, uint32_t fromNode, uint8_t nonce[16]);

// ── AES-CTR ciphers ───────────────────────────────────────────────────────

/**
 * AES-128-CTR encrypt / decrypt (CTR is symmetric — same function for both).
 *
 * @param key      16-byte AES key.
 * @param nonce_in 16-byte initial counter value (not modified — internal copy taken).
 * @param in       Input buffer (plaintext or ciphertext).
 * @param len      Number of bytes to process.
 * @param out      Output buffer (must be at least len bytes; may alias in).
 * @return true on success, false on mbedtls error.
 */
bool mc_aes128ctr(const uint8_t key[16], const uint8_t nonce_in[16],
                  const uint8_t* in, size_t len, uint8_t* out);

/**
 * AES-256-CTR encrypt / decrypt.
 *
 * @param key      32-byte AES key (e.g. X25519 shared secret for PKC DMs).
 * @param nonce_in 16-byte initial counter value (not modified — internal copy taken).
 */
bool mc_aes256ctr(const uint8_t key[32], const uint8_t nonce_in[16],
                  const uint8_t* in, size_t len, uint8_t* out);

/**
 * Meshtastic channel cipher — AES-128-CTR using the standard Meshtastic
 * nonce layout.  Combines mc_buildNonce() + mc_aes128ctr().
 *
 * Used for BOTH encrypting TX packets and decrypting RX packets (CTR is
 * symmetric: encrypt(encrypt(PT)) == PT).
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
 *
 * Uses a self-contained implementation (constant-time, RFC 7748 compliant).
 * Keys are in little-endian RFC 7748 wire format (32 bytes).
 *
 * @param privateKey  32-byte X25519 private key (little-endian, RFC 7748).
 * @param publicKeyOut 32-byte output buffer for the public key (little-endian).
 * @return true on success, false on error.
 */
bool mc_x25519PublicKey(const uint8_t privateKey[32], uint8_t publicKeyOut[32]);

/**
 * Compute an X25519 ECDH shared secret.
 *
 * Uses a self-contained implementation (constant-time, RFC 7748 compliant).
 * Keys are in little-endian RFC 7748 wire format.  The 32-byte output is
 * also little-endian, matching the output of Arduino Crypto's
 * Curve25519::eval() that Meshtastic ESP32 firmware uses directly as the
 * AES-256-CTR key for PKC direct messages.
 *
 * @param ourPrivKey    Our 32-byte X25519 private key (little-endian).
 * @param remotePubKey  Sender's 32-byte X25519 public key (little-endian).
 * @param sharedOut     32-byte output buffer for the shared secret (little-endian).
 * @return true on success, false on error (e.g. low-order point).
 */
bool mc_x25519SharedSecret(const uint8_t ourPrivKey[32],
                           const uint8_t remotePubKey[32],
                           uint8_t sharedOut[32]);

/**
 * PKC direct-message cipher:
 *   1. ECDH → 32-byte shared secret
 *   2. mc_buildNonce(packetId, fromNode)
 *   3. AES-256-CTR(sharedSecret, nonce, in) → out
 *
 * This is the full PKC DM encryption/decryption primitive used by
 * _decryptPkc() (and its TX equivalent).  The shared secret is zeroized
 * from the stack immediately after use.
 *
 * @param ourPrivKey   Our 32-byte X25519 private key (little-endian).
 * @param remotePubKey Remote node's 32-byte X25519 public key (little-endian).
 * @param packetId     OTA header bytes [8:11] (little-endian on wire).
 * @param fromNode     OTA header bytes [4:7]  (little-endian on wire).
 *                     NOTE: fromNode is always the *originator*, even when
 *                     Alice uses this to *encrypt* a message to Bob.
 * @return true on success, false on ECDH or AES error.
 */
bool mc_pkcCrypt(const uint8_t ourPrivKey[32], const uint8_t remotePubKey[32],
                 uint32_t packetId, uint32_t fromNode,
                 const uint8_t* in, size_t len, uint8_t* out);
