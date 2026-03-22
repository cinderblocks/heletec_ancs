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

#ifndef LORA_H_
#define LORA_H_

#include "mesh_codec.h"
#include "task.h"
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <cstdint>
#include <cstring>

/**
 * Counters and signal info accumulated by the LoRa receive loop.
 * Snapshot is safe to copy across tasks (taken under _statsLock).
 */
struct LoRaStats {
    // Packet counters (cumulative since boot)
    uint32_t preambles    = 0; ///< PREAMBLE_DETECTED events (sync word search started)
    uint32_t headersValid = 0; ///< HEADER_VALID events (header CRC OK)
    uint32_t rxPackets    = 0; ///< raw IRQ_RX_DONE events
    uint32_t crcErrors    = 0; ///< CRC error events
    uint32_t headerErrors = 0; ///< header error events
    uint32_t decryptOk    = 0; ///< packets that survived AES decryption
    uint32_t textMessages = 0; ///< TEXT_MESSAGE_APP packets displayed

    // TX counters (cumulative since boot)
    uint32_t txPackets    = 0; ///< successful TX_DONE events
    uint32_t txErrors     = 0; ///< TX attempts that failed (not TX_DONE)
    uint32_t txTimeouts   = 0; ///< TX attempts that timed out waiting for DIO1

    // Last received packet signal quality
    int16_t lastRssi = 0;
    float   lastSnr  = 0.f;

    // State
    enum class State : uint8_t {
        Disabled,    ///< CONFIG_LORA_ENABLED = n
        InitFailed,  ///< SX1262 did not initialise
        Listening,   ///< running, waiting for packets
    } state = State::Disabled;
};

/**
 * LoRa — SX1262 driver + Meshtastic 2.7.x receive/transmit task.
 *
 * Continuously listens on the configured Meshtastic channel (default:
 * US LongFast @ 907.125 MHz, SF11/BW250/CR4-5, private sync word 0x2B).
 *
 * Protocol target: Meshtastic firmware 2.7.x (mesh.proto 2.7.15).
 *
 * Uses standard AES-128-CTR channel encryption for full interoperability
 * with the existing Meshtastic mesh.  TX is always encrypted with the
 * default PSK so that stock Meshtastic nodes can decode our packets.
 *
 * In normal (encrypted) mode, X25519 PKC direct messages (chanHash=0x00)
 * are also decrypted using ECDH shared secrets with AES-256-CCM.
 * When the sender's public key is not yet known, the raw packet is buffered
 * and a NodeInfo request is sent; decryption is retried when the key arrives.
 *
 * Observed isLicensed / isUnmessageable flags in received NodeInfo packets
 * are stored in the neighbour table (MeshUser) for display / filtering.
 *
 * In IsLicensed (ham radio) mode:
 *   - TX packets are still AES-128-CTR encrypted for interop (the PSK is
 *     publicly known / factory default, satisfying Part 97 requirements).
 *   - is_licensed=true is advertised in User proto field 6.
 *   - Public key is all-zeros (no PKC).
 *   - RX: AES-CTR decrypt is tried first; if protobuf parse fails, the raw
 *     payload is used as plaintext (handles other unencrypted nodes).
 *
 * TX packets include:
 *   - POSITION_APP      — GPS fix with speed/heading (periodic + adaptive)
 *   - NODEINFO_APP      — node identity, role, PKC public key
 *   - TELEMETRY_APP     — device metrics (battery, uptime)
 *   - MAP_REPORT_APP    — node identity + firmware_version for MQTT bridges (2.7.x)
 *   - TRACEROUTE_APP    — route-discovery replies
 *
 * Runs as a FreeRTOS task on core 1 (BLE / draw tasks run on core 0).
 */
class LoRa : public Task
{
public:
    explicit LoRa(const char* name, uint16_t stackSize = 10240);

    /// Return a copy of the most recently received text message.  Thread-safe.
    MeshMessage lastMessage() const;

    /// Return a snapshot of LoRa counters and state.  Thread-safe.
    LoRaStats stats() const;

    /// Transmit a raw LoRa payload.  Blocks until TX_DONE or timeout.
    /// Must only be called from the LoRa task (run loop).
    /// Returns true on success, false on error/timeout.
    bool transmit(const uint8_t* data, uint8_t len);

    /**
     * Broadcast a POSITION_APP packet with the supplied GPS fix data.
     * lat/lng in decimal degrees, altM in metres above MSL,
     * pdop_x100 = PDOP × 100 (e.g. 120 = PDOP 1.20), sats = visible sats,
     * unixTime = UTC epoch seconds (0 → uses system clock).
     * Must only be called from the LoRa task.  Returns true on TX success.
     */
    bool sendPosition(double lat, double lng, float altM,
                      uint32_t pdop_x100, uint32_t sats,
                      uint32_t unixTime = 0);

    /**
     * Broadcast (or unicast) a NODEINFO_APP packet advertising this device.
     * @param to          Destination node ID. 0xFFFFFFFF = broadcast (default).
     * @param wantResponse Set true for initial broadcasts; false for replies.
     * @param requestId   Packet ID of the incoming NodeInfo request being answered.
     *                    Placed in Data proto field 6 (request_id, fixed32).
     *                    Meshtastic Router.cpp matches this against the pending
     *                    request and calls sendToPhone() to update the app.
     */
    bool sendNodeInfo(uint32_t to = 0xFFFFFFFF, bool wantResponse = true,
                      uint32_t requestId = 0);

    /**
     * Broadcast a TELEMETRY_APP Device Metrics packet (uptime + nominal battery).
     * Keeps this node appearing as "active" in Meshtastic app node lists.
     * Must only be called from the LoRa task.  Returns true on TX success.
     */
    bool sendTelemetry();

    /**
     * Broadcast a MAP_REPORT_APP (port 73) packet for public mesh map visibility.
     * Announces this node's identity, hardware model, region, modem preset, and
     * current GPS fix to any MQTT bridge in range.  Bridges forward it to
     * meshtastic.network/map so the node appears on the public map.
     * Only sent when a GPS fix is available.
     * Must only be called from the LoRa task.  Returns true on TX success.
     */
    bool sendMapReport();

    /// Number of unique neighbour nodes seen since boot (0–8).  Thread-safe.
    size_t neighborCount() const;

    /// Copy of the neighbour entry at index idx (0-based).  Thread-safe.
    /// Returns a zeroed MeshPosition/MeshUser/MeshNodeStatus if idx is out of range.
    MeshPosition   neighborPosition(size_t idx) const;
    MeshUser       neighborUser(size_t idx) const;
    MeshNodeStatus neighborStatus(size_t idx) const;

protected:
    void run(void* data) override;

private:
    // ── Hardware pins — Heltec Wireless Tracker 1.1 schematic ─────────────
    static constexpr gpio_num_t PIN_NSS   = GPIO_NUM_8;
    static constexpr gpio_num_t PIN_SCK   = GPIO_NUM_9;
    static constexpr gpio_num_t PIN_MOSI  = GPIO_NUM_10;
    static constexpr gpio_num_t PIN_MISO  = GPIO_NUM_11;
    static constexpr gpio_num_t PIN_RESET = GPIO_NUM_12;
    static constexpr gpio_num_t PIN_BUSY  = GPIO_NUM_13;
    static constexpr gpio_num_t PIN_DIO1  = GPIO_NUM_14;

    // ── SPI ───────────────────────────────────────────────────────────────
    // SPI2_HOST (HSPI) — SPI3_HOST is reserved for the TFT.
    static constexpr spi_host_device_t SPI_HOST   = SPI2_HOST;
    static constexpr int               SPI_FREQ_HZ = 8 * 1000 * 1000; // 8 MHz

    spi_device_handle_t _spi = nullptr;

    // ── DIO1 interrupt → task notification ────────────────────────────────
    TaskHandle_t _taskHandle = nullptr; // set at top of run() before ISR install
    static void IRAM_ATTR _dio1Isr(void* arg);

    // ── SX1262 SPI helpers ────────────────────────────────────────────────
    void _waitBusy(uint32_t timeoutMs = 200) const;
    void _transact(const uint8_t* tx, uint8_t* rx, size_t len);
    void _writeReg(uint16_t addr, uint8_t val);
    uint8_t _readReg(uint16_t addr);
    void _writeBuffer(uint8_t offset, const uint8_t* data, uint8_t len);

    // ── SX1262 init & config ──────────────────────────────────────────────
    bool _initSx1262();
    void _setFrequency(uint32_t freqHz);
    void _calibrateImage(uint32_t freqHz);
    void _setModulation(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);
    void _setRx();
    void _setRxIrq();
    void _setTxIrq();
    void _fixIqPolarity(); ///< SX1262 errata 15.3 — fix IQ polarity register

    uint16_t _getIrqStatus();
    void     _clearIrq(uint16_t mask);
    uint8_t  _getChipMode();    ///< bits [6:4] of GetStatus: 2=STBY_RC 5=RX 6=TX
    uint16_t _getDeviceErrors();///< opError bitmask; bit8=PA_RAMP_ERR bit6=PLL_LOCK_ERR
    int16_t  _getInstRssi();    ///< instantaneous RSSI in RX mode (dBm); noise floor check

    // ── Meshtastic constants ──────────────────────────────────────────────
    // Over-the-air header: [to(4), from(4), id(4), flags(1), chan(1), pad(2)]
    static constexpr size_t MESH_HDR = 16;

    // PortNum values (Meshtastic 2.7.x portnums.proto)
    static constexpr uint32_t PORT_UNKNOWN       = 0;  ///< UNKNOWN_APP - Message sent from outside the mesh in a form that is not understood
    static constexpr uint32_t PORT_TEXT          = 1;  ///< TEXT_MESSAGE_APP
    static constexpr uint32_t PORT_POSITION      = 3;  ///< POSITION_APP
    static constexpr uint32_t PORT_NODEINFO      = 4;  ///< NODEINFO_APP
    static constexpr uint32_t PORT_ROUTING       = 5;  ///< ROUTING_APP — ACK/NACK (received, not dispatched)
    static constexpr uint32_t PORT_NODE_STATUS   = 36; ///< NODE_STATUS_APP — online heartbeat + MQTT/router flags (2.7.x)
    static constexpr uint32_t PORT_TELEMETRY     = 67; ///< TELEMETRY_APP
    static constexpr uint32_t PORT_TRACEROUTE    = 70; ///< TRACEROUTE_APP — route discovery
    static constexpr uint32_t PORT_NEIGHBORINFO  = 71; ///< NEIGHBORINFO_APP — mesh neighbour tables (received, not dispatched)
    static constexpr uint32_t PORT_MAP_REPORT    = 73; ///< MAP_REPORT_APP — public mesh map visibility

    // Meshtastic default channel AES-128 PSK (factory LongFast).
    // Used for both RX decryption and TX encryption.
    static const uint8_t DEFAULT_PSK[16];

    // Meshtastic LoRa sync word.
    // Meshtastic firmware uses RadioLib setSyncWord(0x2B), NOT the generic
    // LoRa private sync word 0x12.  RadioLib converts 0x2B to SX1262
    // register values via: MSB=(sw & 0xF0)|0x04, LSB=((sw & 0x0F)<<4)|0x04
    //   → MSB = 0x24, LSB = 0xB4  (registers 0x0740:0x0741)
    // The old value 0x1424 was the generic LoRa private sync word (0x12 in
    // RadioLib), causing preamble detection but zero sync word matches.
    static constexpr uint8_t SYNC_HI = 0x24;
    static constexpr uint8_t SYNC_LO = 0xB4;

    // ── Meshtastic packet processing ──────────────────────────────────────
    void _processPacket(const uint8_t* buf, uint8_t len,
                        int16_t rssi, float snr);
    /// AES-128-CTR cipher — same function for both encrypt and decrypt (CTR is symmetric).
    /// Nonce: [packetId LE (4B) | 0 (4B) | fromNode LE (4B) | 0 (4B)]
    bool _decrypt(const uint8_t* in, size_t len,
                  uint32_t packetId, uint32_t fromNode,
                  uint8_t* out);

    /// AES-256-CCM PKC (public key cryptography) cipher for direct messages.
    /// Uses SHA-256(X25519 ECDH shared secret) as the 256-bit key.
    /// chanHash=0x00 marks PKC-encrypted packets on the wire.
    ///
    /// Standard Meshtastic PKC decrypt: SHA-256(X25519 ECDH) key, AES-256-CCM,
    /// nonce = [pktId_LE(4)|0(4)|fromNode_LE(4)|0(4)] truncated to 13 bytes, no AAD.
    /// Wire = [ciphertext(N)] [8-byte CCM tag].
    /// @param plainLen  Output: plaintext byte count on success.
    /// @return true on CCM auth success.
    bool _decryptPkc(const uint8_t* in, size_t len,
                     const uint8_t* hdr, size_t hdrLen,
                     uint32_t packetId, uint32_t fromNode,
                     uint32_t toNode,
                     uint8_t* out, size_t& plainLen);
    bool _parseData(const uint8_t* data, size_t len,
                    uint32_t& portnum,
                    const uint8_t*& payload, size_t& payloadLen,
                    bool& wantResponse);

    /// Decode a Meshtastic Position proto payload into pos.
    static bool _parsePosition(const uint8_t* data, size_t len, MeshPosition& pos);
    /// Decode a Meshtastic User proto payload into user.
    static bool _parseUser(const uint8_t* data, size_t len, MeshUser& user);
    /// Decode a Meshtastic NodeStatus proto payload into status.
    static bool _parseNodeStatus(const uint8_t* data, size_t len, MeshNodeStatus& status);
    /// Insert or update the neighbour table entry for fromNode.
    void _upsertNeighbor(uint32_t fromNode,
                         const MeshPosition*   pos,        // nullptr = no update
                         const MeshUser*       user,       // nullptr = no update
                         const MeshNodeStatus* nodeStatus = nullptr); // nullptr = no update

    // ── Protobuf encoder helpers (all static, no heap) ────────────────────
    /// Encode a varint into buf. Returns bytes written.
    static size_t   _pbVarint(uint8_t* buf, uint64_t val);
    /// Zigzag-encode a signed 32-bit integer → unsigned (sint32 wire format).
    static uint32_t _pbZigzag(int32_t val);
    /// Write a length-delimited field (tag byte, length varint, raw bytes).
    static size_t   _pbLenField(uint8_t* buf, uint8_t tag,
                                const uint8_t* data, size_t dataLen);
    /// Write a protobuf string field (length-delimited, null-terminated src).
    static size_t   _pbString(uint8_t* buf, uint8_t tag, const char* s);

    /// Encode a Meshtastic Position proto into buf. Returns bytes written.
    static size_t _encodePosition(uint8_t* buf, size_t cap,
                                  int32_t lat_i, int32_t lon_i, int32_t alt_m,
                                  uint32_t pdop_x100, uint32_t sats,
                                  uint32_t unixTime,
                                  uint32_t speed_cm_s = 0,
                                  uint32_t track_x100 = 0);

    /// Encode a Meshtastic User (NodeInfo) proto into buf. Returns bytes written.
    static size_t _encodeUser(uint8_t* buf, size_t cap,
                              uint32_t nodeId,
                              const char* longName, const char* shortName);

    /// Encode a Meshtastic Telemetry (Device Metrics) proto. Returns bytes written.
    static size_t _encodeTelemetry(uint8_t* buf, size_t cap,
                                   uint32_t unixTime, uint32_t uptimeSec,
                                   uint8_t batteryLevel, float batteryVoltage);

    /// Encode a Meshtastic MapReport proto (MAP_REPORT_APP payload).
    /// Carries node identity, region, modem preset, firmware version, and GPS fix.
    /// Field 4 (firmware_version) is included for Meshtastic 2.7.x map-bridge compat.
    /// Returns bytes written.
    static size_t _encodeMapReport(uint8_t* buf, size_t cap,
                                   const char* longName, const char* shortName,
                                   int32_t lat_i, int32_t lon_i, int32_t alt_m,
                                   uint32_t numNeighbors,
                                   const char* firmwareVersion = nullptr);

    /// Wrap an encoded payload in a Meshtastic Data proto. Returns bytes written.
    /// dest is fixed32 (field 4). requestId is fixed32 (field 6 = request_id).
    /// Confirmed from live traffic: Meshtastic uses field 6 (request_id) in
    /// responses, not field 7 (reply_id). Router.cpp matches on request_id.
    static size_t _encodeData(uint8_t* buf, size_t cap,
                              uint32_t portnum,
                              const uint8_t* payload, size_t payloadLen,
                              bool want_response = false,
                              uint32_t dest = 0,
                              uint32_t source = 0,
                              uint32_t requestId = 0);

    /// Build a complete OTA Meshtastic packet ready for transmit().
    /// Data proto is always AES-128-CTR encrypted (channel PSK), even in
    /// IsLicensed mode — stock Meshtastic nodes match chanHash=0x08 to
    /// their encrypted channel and will AES-CTR decrypt our payload.
    /// Sending plaintext with chanHash=0x08 causes receivers to "decrypt"
    /// the raw bytes into garbage and silently drop the packet.
    /// @param to        Destination node ID (0xFFFFFFFF = broadcast).
    /// @param requestId Incoming pktId for unicast NodeInfo responses (0 = none).
    ///                  Placed in Data field 6 (request_id, fixed32) so the
    ///                  receiving firmware can correlate the response.
    bool _buildTxPacket(uint8_t* out, uint8_t& outLen,
                        uint32_t portnum,
                        const uint8_t* payload, size_t payloadLen,
                        bool want_response = false,
                        uint32_t to = 0xFFFFFFFF,
                        uint32_t requestId = 0);

    // ── Thread-safe last-message store ────────────────────────────────────
    mutable portMUX_TYPE _msgLock   = portMUX_INITIALIZER_UNLOCKED;
    MeshMessage          _lastMsg   = {};

    // ── Thread-safe stats store ───────────────────────────────────────────
    mutable portMUX_TYPE _statsLock = portMUX_INITIALIZER_UNLOCKED;
    LoRaStats            _stats     = {};

    // ── Periodic TX state (run loop only — no locking needed) ─────────────
    TickType_t _lastPosTxTick       = 0; ///< tick of last POSITION_APP TX
    TickType_t _lastNodeInfoTxTick  = 0; ///< tick of last NODEINFO_APP TX
    TickType_t _lastTelemetryTxTick = 0; ///< tick of last TELEMETRY_APP TX
    TickType_t _lastMapReportTxTick = 0; ///< tick of last MAP_REPORT_APP TX
    uint8_t    _nodeInfoBootCount   = 0; ///< NodeInfo broadcasts sent since boot

    // ── Adaptive position broadcast state ────────────────────────────────
    // Track last sent lat/lon to detect significant movement and trigger
    // an early broadcast. A movement of ~50m (0.0005°) triggers an early send
    // but only if at least MIN_POS_TX_INTERVAL_SEC has elapsed to prevent
    // flooding the channel while moving continuously.
    static constexpr uint32_t MIN_POS_TX_INTERVAL_SEC = 30;  ///< minimum between any two TX
    int32_t _lastTxLat_i = 0;   ///< latitude_i × 1e7 of last sent position
    int32_t _lastTxLon_i = 0;   ///< longitude_i × 1e7 of last sent position
    bool    _hasTxPos    = false;///< true once we've sent at least one position

    // ── Neighbour table ───────────────────────────────────────────────────
    static constexpr size_t NEIGHBOR_MAX = 8;
    struct NeighborEntry {
        MeshPosition   pos        = {};
        MeshUser       user       = {};
        MeshNodeStatus nodeStatus = {}; ///< last NODE_STATUS_APP received from this node
        bool           occupied   = false;
    };
    mutable portMUX_TYPE _neighborLock = portMUX_INITIALIZER_UNLOCKED;
    NeighborEntry        _neighbors[NEIGHBOR_MAX] = {};

    // ── Packet deduplication ring (Phase 6) ───────────────────────────────
    static constexpr size_t SEEN_IDS_MAX = 16;
    uint32_t _seenIds[SEEN_IDS_MAX] = {};
    size_t   _seenCursor            = 0;

    // ── PKC key-request rate limiter ──────────────────────────────────────
    // When a PKC DM arrives from a node whose public key we don't have, we
    // send a NODEINFO request (want_response=true) so they reply with their
    // key.  To avoid flooding, we only request once per node per 60 s.
    static constexpr size_t   PKC_REQ_RING_MAX = 4;
    static constexpr uint32_t PKC_REQ_COOLDOWN_MS = 60000;
    struct PkcKeyReq {
        uint32_t   nodeId = 0;
        TickType_t tick   = 0;
    };
    PkcKeyReq _pkcKeyReqs[PKC_REQ_RING_MAX] = {};
    size_t    _pkcReqCursor = 0; ///< ring-buffer write cursor for _pkcKeyReqs

    // ── Pending PKC packet buffer ─────────────────────────────────────────
    // Stores raw undecryptable PKC packets (missing sender public key) so
    // they can be retried once we learn the sender's key from a NODEINFO.
    // Entries expire after 5 minutes — stale DMs are not useful.
    struct PendingPkcPacket {
        uint8_t    data[256] = {};
        uint8_t    len       = 0;
        int16_t    rssi      = 0;
        float      snr       = 0.f;
        uint32_t   fromNode  = 0;
        uint32_t   pktId     = 0;   ///< for dedup within buffer
        TickType_t tick       = 0;
        bool       occupied  = false;
    };
    static constexpr size_t   PKC_PENDING_MAX       = 4;
    static constexpr uint32_t PKC_PENDING_EXPIRE_MS = 300000; // 5 minutes
    PendingPkcPacket _pkcPending[PKC_PENDING_MAX] = {};

    /// Buffer a raw PKC packet for later decryption (called when key is missing).
    void _bufferPkcPacket(const uint8_t* buf, uint8_t len,
                          int16_t rssi, float snr);
    /// Retry decryption of any buffered PKC packets from fromNode.
    /// Called after we learn a node's public key from their NODEINFO.
    void _retryPkcBuffer(uint32_t fromNode);
};

extern LoRa Lora;

#endif // LORA_H_
