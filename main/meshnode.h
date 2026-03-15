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

#ifndef MESHNODE_H_
#define MESHNODE_H_

#include <cstdint>

/**
 * MeshNode — persistent Meshtastic node identity.
 *
 * Derives the node ID from the lower 4 bytes of the ESP32 Bluetooth MAC
 * address (matching the standard Meshtastic convention).  Long name and
 * short name are read from NVS on init() and fall back to Kconfig defaults
 * when no value has been stored.
 *
 * Thread-safety: init() must be called once from app_main before any task
 * starts.  All read accessors (nodeId, shortName, longName, nodeIdStr) are
 * thereafter read-only and safe to call from any task without locking.
 * The write accessors (setShortName, setLongName) should only be called
 * from a single writer at a time (e.g. a future BLE admin handler).
 *
 * Usage:
 *   Node.init();                // call once in app_main
 *   Node.nodeId();              // → 0xdeadbeef
 *   Node.nodeIdStr();           // → "!deadbeef"
 *   Node.shortName();           // → "BLUE"
 *   Node.longName();            // → "HatefulBlue"
 *   Node.nextPacketId();        // → random uint32_t
 */
class MeshNode
{
public:
    MeshNode() = default;

    /**
     * Initialise the node identity.
     *
     * Reads the Bluetooth MAC from eFuse, computes the node ID, then reads
     * NVS namespace "mesh" for stored "short" and "long" name keys.  Falls
     * back to CONFIG_MESH_NODE_SHORT_NAME / CONFIG_MESH_NODE_LONG_NAME when
     * the NVS keys are absent or NVS is uninitialised.
     *
     * Logs the node ID and names at INFO level.
     */
    void init();

    /**
     * 32-bit node ID derived from the lower 4 bytes of the BT MAC.
     * e.g. BT MAC = AA:BB:CC:DD:EE:FF → nodeId = 0xCCDDEEFF
     */
    uint32_t nodeId() const { return _nodeId; }

    /**
     * Node ID as a Meshtastic "!xxxxxxxx" string (9 chars + NUL).
     * e.g. "!deadbeef"
     */
    const char* nodeIdStr() const { return _nodeIdStr; }

    /**
     * Short name — 1–4 printable ASCII characters.
     */
    const char* shortName() const { return _shortName; }

    /**
     * Long name — up to 32 UTF-8 characters.
     */
    const char* longName() const { return _longName; }

    /**
     * 6-byte Bluetooth MAC address (same source as node ID derivation).
     * Used for User proto field 4 (macaddr) for compatibility with older
     * Meshtastic firmware.
     */
    const uint8_t* macaddr() const { return _mac; }

    /**
     * 32-byte X25519 public key field for the Meshtastic User proto.
     * In IsLicensed (unencrypted) mode this is all-zeros — no PKC is used.
     * In encrypted mode this is derived from the persisted private key.
     */
    const uint8_t* publicKey() const { return _publicKey; }

    /**
     * Whether this node has a valid PKC keypair (encrypted mode only).
     */
    bool hasPkcKeys() const { return _hasPkcKeys; }

    /**
     * Compute an X25519 ECDH shared secret from our private key and a
     * remote node's public key.  The 32-byte result is written to sharedOut.
     *
     * Input keys are little-endian (Meshtastic/RFC 7748 wire format) and
     * are reversed internally to big-endian for the mbedtls MPI API.
     * The shared secret output is little-endian, matching the output of
     * Curve25519::eval() (Arduino Crypto library) that Meshtastic ESP32
     * firmware uses directly as the AES-256 key.
     *
     * @param remotePubKey  Remote node's 32-byte X25519 public key
     *                      (little-endian, as received on the wire).
     * @param sharedOut     Output buffer — must be at least 32 bytes.
     *                      Filled with the 32-byte little-endian shared secret.
     * @return true on success, false if no PKC keys or ECDH failed.
     */
    bool computeSharedSecret(const uint8_t* remotePubKey,
                             uint8_t* sharedOut) const;

    /**
     * Persist a new short name to NVS and update the in-memory cache.
     * @param name  Must be 1–4 printable ASCII characters; silently
     *              truncated to 4 chars.  nullptr is ignored.
     */
    void setShortName(const char* name);

    /**
     * Persist a new long name to NVS and update the in-memory cache.
     * @param name  Up to 32 characters; silently truncated.
     *              nullptr is ignored.
     */
    void setLongName(const char* name);

    /**
     * Return a cryptographically random 32-bit packet ID.
     * Uses the ESP32-S3 hardware RNG via esp_random().
     * Safe to call from any task.
     */
    uint32_t nextPacketId() const;

private:
    uint32_t _nodeId        = 0;
    uint8_t  _mac[6]       = {};   // Bluetooth MAC
    uint8_t  _publicKey[32] = {};  // All-zeros in IsLicensed (unencrypted) mode
    uint8_t  _privateKey[32]= {};  // X25519 private key (encrypted mode only)
    bool     _hasPkcKeys    = false; // true when a valid keypair is loaded
    char     _nodeIdStr[12] = {};  // "!xxxxxxxx\0" — 10 chars + NUL
    char     _shortName[5]  = {};  // max 4 chars + NUL
    char     _longName[33]  = {};  // max 32 chars + NUL
};

extern MeshNode Node;

#endif // MESHNODE_H_