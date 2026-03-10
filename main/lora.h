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

#include "task.h"
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <cstdint>
#include <cstring>

/**
 * A received Meshtastic text message — plain-old-data, safe to copy across tasks.
 */
struct MeshMessage {
    uint32_t fromNode = 0;   ///< sender node ID (hex, e.g. 0xdeadbeef)
    char     text[65] = {};  ///< UTF-8 text, null-terminated, max 64 chars
    int16_t  rssi     = 0;   ///< signal strength in dBm
    float    snr      = 0.f; ///< signal-to-noise ratio in dB
    bool     valid    = false;
};

/**
 * Counters and signal info accumulated by the LoRa receive loop.
 * Snapshot is safe to copy across tasks (taken under _statsLock).
 */
struct LoRaStats {
    // Packet counters (cumulative since boot)
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
 * LoRa — SX1262 driver + Meshtastic receive task.
 *
 * Continuously listens on the configured Meshtastic channel (default:
 * US LongFast @ 906.875 MHz, SF11/BW250/CR4-5, private sync word 0x1424).
 *
 * Received packets are decrypted with the Meshtastic default channel PSK
 * and, if they contain a TEXT_MESSAGE_APP Data proto, the text is stored
 * and the draw task is notified via Hardware::notifyDraw(DRAW_LORA).
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
     * Broadcast a NODEINFO_APP packet advertising this device's identity.
     * Uses Node.nodeId() / Node.longName() / Node.shortName().
     * Must only be called from the LoRa task.  Returns true on TX success.
     */
    bool sendNodeInfo();

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

    uint16_t _getIrqStatus();
    void     _clearIrq(uint16_t mask);

    // ── Meshtastic constants ──────────────────────────────────────────────
    // Over-the-air header: [to(4), from(4), id(4), flags(1), chan(1), pad(2)]
    static constexpr size_t MESH_HDR = 16;

    // PortNum values used by this firmware
    static constexpr uint32_t PORT_TEXT     = 1;  ///< TEXT_MESSAGE_APP
    static constexpr uint32_t PORT_POSITION = 3;  ///< POSITION_APP
    static constexpr uint32_t PORT_NODEINFO = 67; ///< NODEINFO_APP

    // Meshtastic default channel AES-128 PSK.
    // Derived from the "Default" channel name; used by every out-of-box device.
    static const uint8_t DEFAULT_PSK[16];

    // Private mesh LoRa sync word (RadioLib 0x12 → SX1262 registers 0x1424).
    static constexpr uint8_t SYNC_HI = 0x14;
    static constexpr uint8_t SYNC_LO = 0x24;

    // ── Meshtastic packet processing ──────────────────────────────────────
    void _processPacket(const uint8_t* buf, uint8_t len,
                        int16_t rssi, float snr);
    bool _decrypt(const uint8_t* ciphertext, size_t len,
                  uint32_t packetId, uint32_t fromNode,
                  uint8_t* plaintext);
    bool _parseData(const uint8_t* data, size_t len,
                    uint32_t& portnum,
                    const uint8_t*& payload, size_t& payloadLen);

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
                                  uint32_t unixTime);

    /// Encode a Meshtastic User (NodeInfo) proto into buf. Returns bytes written.
    static size_t _encodeUser(uint8_t* buf, size_t cap,
                              uint32_t nodeId,
                              const char* longName, const char* shortName);

    /// Wrap an encoded payload in a Meshtastic Data proto. Returns bytes written.
    static size_t _encodeData(uint8_t* buf, size_t cap,
                              uint32_t portnum,
                              const uint8_t* payload, size_t payloadLen);

    /// Build a complete encrypted OTA Meshtastic packet ready for transmit().
    /// Writes header + AES-CTR ciphertext into out[256]. Returns false on error.
    bool _buildTxPacket(uint8_t* out, uint8_t& outLen,
                        uint32_t portnum,
                        const uint8_t* payload, size_t payloadLen);

    // ── Thread-safe last-message store ────────────────────────────────────
    mutable portMUX_TYPE _msgLock   = portMUX_INITIALIZER_UNLOCKED;
    MeshMessage          _lastMsg   = {};

    // ── Thread-safe stats store ───────────────────────────────────────────
    mutable portMUX_TYPE _statsLock = portMUX_INITIALIZER_UNLOCKED;
    LoRaStats            _stats     = {};

    // ── Periodic TX state (run loop only — no locking needed) ─────────────
    TickType_t _lastPosTxTick = 0;  ///< tick of last POSITION_APP TX (0 = never)
};

extern LoRa Lora;

#endif // LORA_H_
