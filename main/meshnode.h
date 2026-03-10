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
 *   Node.shortName();           // → "YIFF"
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
    uint32_t _nodeId       = 0;
    char     _nodeIdStr[12]= {};   // "!xxxxxxxx\0" — 10 chars + NUL
    char     _shortName[5] = {};   // max 4 chars + NUL
    char     _longName[33] = {};   // max 32 chars + NUL
};

extern MeshNode Node;

#endif // MESHNODE_H_
