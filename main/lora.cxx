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

#include "lora.h"
#include "gps.h"
#include "hardware.h"
#include "meshnode.h"
#include "sdkconfig.h"

#include <esp_log.h>
#include <esp_rom_sys.h>
#include <mbedtls/aes.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cinttypes>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <algorithm>

static const char* TAG = "lora";

// ── Kconfig fallback defaults ─────────────────────────────────────────────
// Guard every symbol that has been added incrementally so the file compiles
// correctly even when sdkconfig hasn't been regenerated yet.
#ifndef CONFIG_LORA_TX_ENABLED
#  define CONFIG_LORA_TX_ENABLED 1
#endif
#ifndef CONFIG_LORA_TX_POWER_DBM
#  define CONFIG_LORA_TX_POWER_DBM 22
#endif
#ifndef CONFIG_LORA_POSITION_TX_INTERVAL_SEC
#  define CONFIG_LORA_POSITION_TX_INTERVAL_SEC 300
#endif
#ifndef CONFIG_LORA_NODEINFO_TX_INTERVAL_SEC
#  define CONFIG_LORA_NODEINFO_TX_INTERVAL_SEC 1800
#endif
#ifndef CONFIG_LORA_TELEMETRY_TX_INTERVAL_SEC
#  define CONFIG_LORA_TELEMETRY_TX_INTERVAL_SEC 600
#endif
#ifndef CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC
#  define CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC 1800
#endif
#ifndef CONFIG_LORA_CHANNEL_NUM
#  define CONFIG_LORA_CHANNEL_NUM 0
#endif
#ifndef CONFIG_LORA_FREQUENCY_HZ
#  define CONFIG_LORA_FREQUENCY_HZ 0
#endif
#ifndef CONFIG_MESH_NODE_ROLE
#  define CONFIG_MESH_NODE_ROLE 5   // TRACKER — matches hardware purpose
#endif
#ifndef CONFIG_LORA_REGION_CODE
#  define CONFIG_LORA_REGION_CODE 1 // US
#endif

// ── Meshtastic frequency computation ──────────────────────────────────────
// Matches RadioInterface.cpp getFreq() in Meshtastic firmware 2.5+.
//
// US_915 region: freqStart=902.0 MHz, freqEnd=928.0 MHz
// LongFast BW = 250 kHz → numChannels = (928.0-902.0)/0.250 = 104
// Frequency = freqStart + bw/2 + slotNum * bw
//           = 902000000 + 125000 + slot * 250000
//           = 902125000 + slot * 250000
//
// Meshtastic slot numbering:
//   config channel_num=0 → derive from channel name hash
//     - hash("") using Meshtastic's hash function → slot for default channel
//   config channel_num=N (1-based) → slot = N - 1
//
// Meshtastic default channel (empty name ""):
//   Uses CRC-like hash in Channels.cpp → getFrequencySlot():
//     hash = 0xdeadbeef ^ length; for each byte: hash ^= byte, hash *= 0x01000193
//     For length=0 (empty string): hash = 0xdeadbeef ^ 0 = 0xdeadbeef
//     slot = 0xdeadbeef % 104 = 3735928559 % 104 = 103
//     freq = 902125000 + 103 * 250000 = 927875000 Hz (927.875 MHz)
//
//   NOTE: This can vary between Meshtastic firmware versions. The user's
//   nearby node may use a different algorithm. The Rx pkt log line shows
//   the actual channel hash of received packets — cross-reference with
//   the Meshtastic app to find the correct slot.

static uint32_t computeLoraFrequency()
{
    // If an explicit frequency is set in Kconfig, use it directly.
    if (CONFIG_LORA_FREQUENCY_HZ != 0)
        return static_cast<uint32_t>(CONFIG_LORA_FREQUENCY_HZ);

    // Otherwise compute from channel slot number.
    // US_915 region: freqStart=902.0 MHz, BW=250 kHz, 104 channels
    static constexpr uint32_t FREQ_START   = 902000000;
    static constexpr uint32_t BW_HZ        = 250000;
    static constexpr uint32_t NUM_CHANNELS = 104;

    uint32_t slot = (CONFIG_LORA_CHANNEL_NUM > 0)
                  ? static_cast<uint32_t>(CONFIG_LORA_CHANNEL_NUM) - 1
                  : 20u; // Meshtastic US LongFast factory default

    if (slot >= NUM_CHANNELS) slot = 0;
    return FREQ_START + BW_HZ / 2 + slot * BW_HZ;
}

// Cached at file scope — computed once, used throughout.
static const uint32_t LORA_FREQ_HZ = computeLoraFrequency();

// ── SX1262 opcodes ────────────────────────────────────────────────────────
static constexpr uint8_t CMD_SET_STANDBY         = 0x80;
static constexpr uint8_t CMD_SET_RX              = 0x82;
static constexpr uint8_t CMD_SET_TX              = 0x83;
static constexpr uint8_t CMD_SET_PACKET_TYPE     = 0x8A;
static constexpr uint8_t CMD_SET_RF_FREQ         = 0x86;
static constexpr uint8_t CMD_SET_PA_CONFIG        = 0x95;
static constexpr uint8_t CMD_SET_TX_PARAMS       = 0x8E;
static constexpr uint8_t CMD_SET_MOD_PARAMS      = 0x8B;
static constexpr uint8_t CMD_SET_PKT_PARAMS      = 0x8C;
static constexpr uint8_t CMD_SET_BUF_BASE_ADDR   = 0x8F;
static constexpr uint8_t CMD_SET_DIO_IRQ         = 0x08;
static constexpr uint8_t CMD_GET_IRQ_STATUS      = 0x12;
static constexpr uint8_t CMD_CLEAR_IRQ           = 0x02;
static constexpr uint8_t CMD_GET_RX_BUF_STATUS   = 0x13;
static constexpr uint8_t CMD_GET_PKT_STATUS      = 0x14;
static constexpr uint8_t CMD_READ_BUFFER         = 0x1E;
static constexpr uint8_t CMD_WRITE_BUFFER        = 0x0E;
static constexpr uint8_t CMD_WRITE_REGISTER      = 0x0D;
static constexpr uint8_t CMD_READ_REGISTER       = 0x1D;
static constexpr uint8_t CMD_SET_REGULATOR_MODE  = 0x96;
static constexpr uint8_t CMD_CALIBRATE           = 0x89;
static constexpr uint8_t CMD_CALIBRATE_IMAGE     = 0x98;
static constexpr uint8_t CMD_SET_DIO2_RF_SWITCH  = 0x9D;
static constexpr uint8_t CMD_SET_DIO3_TCXO       = 0x97;
static constexpr uint8_t CMD_GET_STATUS          = 0xC0;
static constexpr uint8_t CMD_GET_DEVICE_ERRORS   = 0x17;
static constexpr uint8_t CMD_CLEAR_DEVICE_ERRORS = 0x07;

// ── SX1262 register addresses ─────────────────────────────────────────────
static constexpr uint16_t REG_LORA_SYNC_MSB = 0x0740; // LoRa sync word byte 0
static constexpr uint16_t REG_LORA_SYNC_LSB = 0x0741; // LoRa sync word byte 1
static constexpr uint16_t REG_IQ_CONFIG     = 0x0736; // IQ polarity (errata 15.3)
static constexpr uint16_t REG_RX_GAIN       = 0x08AC; // RX LNA gain mode

// ── IRQ bit masks (SX1262 Table 13-29, verified against RadioLib) ─────────
static constexpr uint16_t IRQ_TX_DONE      = (1u << 0);
static constexpr uint16_t IRQ_RX_DONE      = (1u << 1);
static constexpr uint16_t IRQ_PREAMBLE_DET = (1u << 2);
static constexpr uint16_t IRQ_HEADER_VALID = (1u << 4);
static constexpr uint16_t IRQ_HEADER_ERR   = (1u << 5);
static constexpr uint16_t IRQ_CRC_ERROR    = (1u << 6);
static constexpr uint16_t IRQ_TIMEOUT      = (1u << 9);

// DIO1 mask — events that wake the task via ISR
static constexpr uint16_t IRQ_RX_DIO1     = IRQ_RX_DONE | IRQ_HEADER_ERR |
                                             IRQ_CRC_ERROR | IRQ_TIMEOUT;
// Global mask — events visible in GetIrqStatus (superset of DIO1).
// PREAMBLE_DETECTED and HEADER_VALID are included so they appear alongside
// RX_DONE / HEADER_ERR when we read the IRQ register, without generating
// extra DIO1 interrupts.
static constexpr uint16_t IRQ_RX_GLOBAL   = IRQ_RX_DIO1 | IRQ_PREAMBLE_DET |
                                             IRQ_HEADER_VALID;
static constexpr uint16_t IRQ_TX_MASK     = IRQ_TX_DONE | IRQ_TIMEOUT;
// TIMEOUT is included so DIO1 fires if the chip somehow enters a fault state;
// normal TX uses SetTx(0) (no hardware timeout) so TX_DONE is the only
// expected exit path.

// ── Meshtastic OTA TX constants ───────────────────────────────────────────
//
// ── Meshtastic default channel PSK ────────────────────────────────────────
// AES-128 key for the default LongFast channel (factory default on every
// Meshtastic device).  Used for both RX decryption and TX encryption so
// this node interoperates with the existing encrypted mesh.
// is_licensed affects whether OUR node *should* encrypt, but since other
// nodes in the mesh still encrypt, we must decrypt their packets and encrypt
// ours for end-to-end interoperability.
/* static */ const uint8_t LoRa::DEFAULT_PSK[16] = {
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};

// DEFAULT_CHAN_HASH: 1-byte channel hash placed in OTA header byte [13].
//
// Meshtastic computes this via Channels::getHash():
//   xorHash(name, strlen(name)) ^ xorHash(key.bytes, key.length)
// where key = Channels::getKey() returns the EXPANDED PSK (not raw psk bytes).
//
// is_licensed only disables CryptoEngine encrypt/decrypt — it does NOT
// change the channel PSK or hash.  The channel identity stays the same
// so IsLicensed and encrypted nodes share the same channel hash.
//
// Default "LongFast" channel (factory PSK {0x01} → expanded 16-byte key):
//   xorHash("LongFast", 8) = 0x0A
//   xorHash(expanded_default_PSK, 16) = 0x02
//   hash = 0x0A ^ 0x02 = 0x08
static constexpr uint8_t DEFAULT_CHAN_HASH = 0x08;

// HW_MODEL: Meshtastic HardwareModel enum value for this board.
// meshtastic_HardwareModel_HELTEC_WIRELESS_TRACKER = 48 (mesh.proto)
static constexpr uint32_t HW_MODEL = 48;

// Meshtastic OTA flags byte layout (RadioLibInterface.cpp / mesh_pb.h):
//
//   bit layout (LSB → MSB):
//     bits [2:0]  hop_limit  — remaining relay hops (decremented by each router)
//     bit  [3]    want_ack   — request implicit ACK from next hop
//     bit  [4]    via_mqtt   — packet arrived via MQTT bridge (NOT for OTA origin!)
//     bits [7:5]  hop_start  — ORIGINAL hop_limit at time of first transmission
//
//   encode: flags = (hop_limit & 7) | (want_ack ? 8 : 0) | (via_mqtt ? 16 : 0) | ((hop_start & 7) << 5)
//   decode: hop_limit = flags & 7;  want_ack = flags & 8;  via_mqtt = flags & 16;  hop_start = flags >> 5;
//
// For self-originated packets hop_start MUST equal hop_limit so that
// receiving nodes correctly compute hops_away = hop_start - hop_limit = 0
// and flag this device as a "direct" neighbour in their NodeDB.
//
// via_mqtt MUST be 0 for OTA-originated packets.  Setting it causes the
// Meshtastic app to display the MQTT cloud icon next to the node, and some
// firmware versions may drop or not relay the packet.
//
// *** BUG HISTORY (0x1B / 0x5B) ***
// The previous values encoded (using the WRONG formula hop_limit | (hop_start<<3) | (want_ack<<6)):
//   0x1B = 0b00011011 → hop_limit=3, want_ack=1 (bit3!), via_mqtt=1 (bit4!), hop_start=0
//   0x5B = 0b01011011 → hop_limit=3, want_ack=1,         via_mqtt=1 (bit4!), hop_start=2
// Both set via_mqtt=1 → MQTT icon in Meshtastic app.
// hop_start=0 (broadcast) and hop_start=2 (unicast) ≠ hop_limit=3 → device never
// appeared as a direct neighbour in remote NodeDB.
//
// Correct values (hop_start = hop_limit = 3, via_mqtt = 0):
//   Broadcast (want_ack=0): 3 | 0 | 0 | (3<<5) = 3 | 96 = 99  = 0x63
//   Unicast   (want_ack=1): 3 | 8 | 0 | (3<<5) = 3 | 8 | 96 = 107 = 0x6B
static constexpr uint8_t FLAGS_BROADCAST = 0x63;
static constexpr uint8_t FLAGS_UNICAST   = 0x6B;

// ─────────────────────────────────────────────────────────────────────────

LoRa::LoRa(const char* name, uint16_t stackSize)
:   Task(name, stackSize, 4)  // priority 4 — below BLE (5), above draw (3)
{ }

// ── DIO1 ISR ──────────────────────────────────────────────────────────────
/* static */ void IRAM_ATTR LoRa::_dio1Isr(void* arg)
{
    LoRa* self = static_cast<LoRa*>(arg);
    BaseType_t higher = pdFALSE;
    vTaskNotifyGiveFromISR(self->_taskHandle, &higher);
    portYIELD_FROM_ISR(higher);
}

// ── _waitBusy ─────────────────────────────────────────────────────────────
void LoRa::_waitBusy(uint32_t timeoutMs) const
{
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    while (gpio_get_level(PIN_BUSY))
    {
        if (xTaskGetTickCount() >= deadline)
        {
            ESP_LOGW(TAG, "BUSY timeout");
            break;
        }
        vTaskDelay(1); // yield while waiting (~1 ms per tick)
    }
}

// ── _transact ─────────────────────────────────────────────────────────────
// Full-duplex SPI transaction.  Waits for BUSY before asserting CS.
// rx may be nullptr if results are not needed.
void LoRa::_transact(const uint8_t* tx, uint8_t* rx, size_t len)
{
    _waitBusy();

    spi_transaction_t t = {};
    t.length    = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;  // nullptr is fine — IDF ignores rx when null

    // When rx is nullptr, set the RX_IN_PROGRESS flag so the IDF doesn't
    // try to dereference it.  Easiest: provide a local dummy buffer for small
    // transfers, or use the SPI_TRANS_USE_RXDATA flag for ≤4 bytes.
    // For simplicity, provide rx always at call sites that care; pass nullptr only
    // for pure write commands.
    spi_device_polling_transmit(_spi, &t);
}

// ── _writeReg ─────────────────────────────────────────────────────────────
void LoRa::_writeReg(uint16_t addr, uint8_t val)
{
    uint8_t tx[4] = {
        CMD_WRITE_REGISTER,
        static_cast<uint8_t>(addr >> 8),
        static_cast<uint8_t>(addr & 0xFF),
        val
    };
    _transact(tx, nullptr, sizeof(tx));
}

// ── _readReg ──────────────────────────────────────────────────────────────
// SX1262 ReadRegister (0x1D) layout — 5 bytes total:
//   TX:  [0x1D] [addrH] [addrL] [NOP#1]  [NOP#2]
//   RX:  [0x00] [0x00]  [0x00]  [status] [data]
// The status byte arrives on NOP#1; actual register data arrives on NOP#2.
// Sending only 4 bytes returns rx[3] = status (0xA2 for STBY_RC), never
// clocking out the data byte — hence the "got 0xA2A2" symptom.
uint8_t LoRa::_readReg(uint16_t addr)
{
    uint8_t tx[5] = {
        CMD_READ_REGISTER,
        static_cast<uint8_t>(addr >> 8),
        static_cast<uint8_t>(addr & 0xFF),
        0x00,   // NOP#1 — status byte returned here (discarded)
        0x00    // NOP#2 — actual register data returned here
    };
    uint8_t rx[5] = {};
    _transact(tx, rx, sizeof(tx));
    return rx[4];
}

// ── _setFrequency ─────────────────────────────────────────────────────────
void LoRa::_setFrequency(uint32_t freqHz)
{
    // freq_reg = freqHz * 2^25 / 32_000_000
    const uint32_t reg =
        static_cast<uint32_t>((static_cast<uint64_t>(freqHz) << 25) / 32000000ULL);
    uint8_t tx[5] = {
        CMD_SET_RF_FREQ,
        static_cast<uint8_t>(reg >> 24),
        static_cast<uint8_t>(reg >> 16),
        static_cast<uint8_t>(reg >>  8),
        static_cast<uint8_t>(reg)
    };
    _transact(tx, nullptr, sizeof(tx));
}

// ── _calibrateImage ───────────────────────────────────────────────────────
// Selects the image-rejection calibration range for the target frequency.
void LoRa::_calibrateImage(uint32_t freqHz)
{
    uint8_t f1, f2;
    if      (freqHz >= 902000000) { f1 = 0xE1; f2 = 0xE9; }  // 902–928 MHz
    else if (freqHz >= 863000000) { f1 = 0xD7; f2 = 0xD8; }  // 863–870 MHz
    else if (freqHz >= 779000000) { f1 = 0xC1; f2 = 0xC5; }  // 779–787 MHz
    else if (freqHz >= 470000000) { f1 = 0x75; f2 = 0x81; }  // 470–510 MHz
    else                          { f1 = 0x6B; f2 = 0x6F; }  // 430–440 MHz
    uint8_t tx[3] = { CMD_CALIBRATE_IMAGE, f1, f2 };
    _transact(tx, nullptr, sizeof(tx));
}

// ── _setModulation ────────────────────────────────────────────────────────
// sf  = 5..12 (spreading factor)
// bw  = SX1262 BW code (SX1262 datasheet Table 13-48, non-sequential):
//         0x04 = 125 kHz   0x05 = 250 kHz   0x06 = 500 kHz
//         (NOT 4/5/6 ≡ these are the literal register values, not an index)
// cr  = 1=4/5, 2=4/6, 3=4/7, 4=4/8
// ldro = low data rate optimise (1 if symbol duration > 16 ms)
void LoRa::_setModulation(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    uint8_t tx[5] = { CMD_SET_MOD_PARAMS, sf, bw, cr, ldro };
    _transact(tx, nullptr, sizeof(tx));
}

// ── _fixIqPolarity ────────────────────────────────────────────────────────
// SX1262 Errata 15.3 — Inverted IQ Operation
//
// After reset, register 0x0736 bit 2 defaults to 1 (inverted IQ polarity).
// SetPacketParams with IQ=standard (0x00) is supposed to clear it, but per
// the errata it does NOT — the register retains the power-on default.
//
// Result: preamble and sync word are detected (they're symmetric / correlator-
// based), but every header CRC fails because the data symbols are decoded
// with inverted chirp direction.  This produces the exact symptom observed:
// hdr_err increments, rx=0 forever, RSSI/SNR are stale.
//
// Fix: after any SetPacketParams call (and especially before SetRx), read-
// modify-write register 0x0736 to enforce the correct polarity.
//   Standard IQ: clear bit 2  (val &= ~0x04)
//   Inverted IQ: set bit 2    (val |= 0x04)
// We always use standard IQ, so we always clear bit 2.
void LoRa::_fixIqPolarity()
{
    uint8_t val = _readReg(REG_IQ_CONFIG);
    val &= ~0x04u; // clear bit 2 for standard (non-inverted) IQ
    _writeReg(REG_IQ_CONFIG, val);
}

// ── _setRx ────────────────────────────────────────────────────────────────
// Enter continuous RX (no timeout; chip stays in RX, fires DIO1 on each
// packet or error, and automatically restarts).
void LoRa::_setRx()
{
    // Apply the IQ polarity errata fix before every RX entry.
    // This guarantees correct header decoding regardless of how many
    // SetPacketParams calls happened since the last _setRx().
    _fixIqPolarity();

    // 0xFFFFFF = Rx Continuous mode
    uint8_t tx[4] = { CMD_SET_RX, 0xFF, 0xFF, 0xFF };
    _transact(tx, nullptr, sizeof(tx));
}

// ── _getIrqStatus ─────────────────────────────────────────────────────────
// SX1262 GetIrqStatus SPI layout — 4 bytes total:
//   TX:  [0x12] [NOP]    [NOP]    [NOP]
//   RX:  [0x00] [status] [IRQ_H]  [IRQ_L]
// The previous 3-byte version captured only IRQ_H and missed IRQ_L entirely,
// so IRQ_TX_DONE (bit 0) and IRQ_RX_DONE (bit 1) — both in IRQ_L — were
// never visible.  Every TX appeared to fail and every RX packet was silently
// dropped.
uint16_t LoRa::_getIrqStatus()
{
    uint8_t tx[4] = { CMD_GET_IRQ_STATUS, 0, 0, 0 };
    uint8_t rx[4] = {};
    _transact(tx, rx, sizeof(tx));
    return (static_cast<uint16_t>(rx[2]) << 8) | rx[3];
}

// ── _clearIrq ─────────────────────────────────────────────────────────────
void LoRa::_clearIrq(uint16_t mask)
{
    uint8_t tx[3] = {
        CMD_CLEAR_IRQ,
        static_cast<uint8_t>(mask >> 8),
        static_cast<uint8_t>(mask & 0xFF)
    };
    _transact(tx, nullptr, sizeof(tx));
}

// ── _writeBuffer ──────────────────────────────────────────────────────────
// Write data into the SX1262 data buffer starting at `offset`.
// SX1262 WriteBuffer layout: [0x0E, offset, data0, data1, …]
void LoRa::_writeBuffer(uint8_t offset, const uint8_t* data, uint8_t len)
{
    // 2-byte header + up to 255 payload bytes
    uint8_t tx[2 + 255] = { CMD_WRITE_BUFFER, offset };
    memcpy(tx + 2, data, len);
    _transact(tx, nullptr, 2 + len);
}

// ── _setRxIrq ─────────────────────────────────────────────────────────────
// Configure DIO1 IRQ mask for receive.
// Global mask includes PREAMBLE_DETECTED and HEADER_VALID so they're visible
// in GetIrqStatus for diagnostics, but only the DIO1 mask events trigger the
// ISR (no extra wakeups for preamble/header-valid).
void LoRa::_setRxIrq()
{
    uint8_t t[] = { CMD_SET_DIO_IRQ,
                    static_cast<uint8_t>(IRQ_RX_GLOBAL >> 8),
                    static_cast<uint8_t>(IRQ_RX_GLOBAL),       // global
                    static_cast<uint8_t>(IRQ_RX_DIO1 >> 8),
                    static_cast<uint8_t>(IRQ_RX_DIO1),         // DIO1
                    0x00, 0x00,   // DIO2
                    0x00, 0x00 }; // DIO3
    _transact(t, nullptr, sizeof(t));
}

// ── _setTxIrq ─────────────────────────────────────────────────────────────
// Configure DIO1 IRQ mask for transmit: TX_DONE | TIMEOUT.
void LoRa::_setTxIrq()
{
    const uint16_t mask = IRQ_TX_MASK;
    uint8_t t[] = { CMD_SET_DIO_IRQ,
                    static_cast<uint8_t>(mask >> 8), static_cast<uint8_t>(mask),  // global
                    static_cast<uint8_t>(mask >> 8), static_cast<uint8_t>(mask),  // DIO1
                    0x00, 0x00,   // DIO2
                    0x00, 0x00 }; // DIO3
    _transact(t, nullptr, sizeof(t));
}

// ── _getChipMode ──────────────────────────────────────────────────────────
// Returns the 3-bit chip mode from GetStatus bits [6:4]:
//   2 = STBY_RC   3 = STBY_XOSC   4 = FS
//   5 = RX        6 = TX
// GetStatus layout:  TX=[0xC0, NOP]  RX=[0x00, status]
uint8_t LoRa::_getChipMode()
{
    uint8_t tx[2] = { CMD_GET_STATUS, 0x00 };
    uint8_t rx[2] = {};
    _transact(tx, rx, sizeof(tx));
    return (rx[1] >> 4) & 0x07;
}

// ── _getDeviceErrors ──────────────────────────────────────────────────────
// Returns the 16-bit opError bitmask from GetDeviceErrors.
//   bit 0  RC64K_CALIB_ERR     bit 1  RC13M_CALIB_ERR
//   bit 2  PLL_CALIB_ERR       bit 3  ADC_CALIB_ERR
//   bit 4  IMG_CALIB_ERR       bit 5  XOSC_START_ERR (TCXO didn't start)
//   bit 6  PLL_LOCK_ERR        bit 8  PA_RAMP_ERR    (PA overcurrent / fault)
// Layout: TX=[0x17,NOP,NOP,NOP]  RX=[0x00,status,err_H,err_L]
uint16_t LoRa::_getDeviceErrors()
{
    uint8_t tx[4] = { CMD_GET_DEVICE_ERRORS, 0, 0, 0 };
    uint8_t rx[4] = {};
    _transact(tx, rx, sizeof(tx));
    return (static_cast<uint16_t>(rx[2]) << 8) | rx[3];
}

// ── _getInstRssi ──────────────────────────────────────────────────────────
// Returns the instantaneous RSSI reading while in RX mode (dBm, negative).
// SX1262 GetRssiInst (0x15) layout — 3 bytes:
//   TX:  [0x15] [NOP]    [NOP]
//   RX:  [0x00] [status] [rssiInst]
// RSSI_dBm = -(rssiInst) / 2
// Useful as a noise-floor check: −115 to −120 dBm = receiver is alive.
// Returns 0 if called outside RX mode (value is meaningless but harmless).
int16_t LoRa::_getInstRssi()
{
    uint8_t tx[3] = { 0x15, 0x00, 0x00 };
    uint8_t rx[3] = {};
    _transact(tx, rx, sizeof(tx));
    return -(static_cast<int16_t>(rx[2])) / 2;
}

// ── transmit ──────────────────────────────────────────────────────────────
// Transmit a raw LoRa payload using direct GPIO polling — no ISR involvement.
//
// After SetTx the chip asserts BUSY HIGH during TCXO warm-up + PLL lock
// (~5–10 ms on this board), then deasserts BUSY LOW when actively transmitting.
// TX_DONE fires after the last bit leaves the antenna; the SX1262 then sets
// the TX_DONE IRQ flag and drives DIO1 HIGH.  We poll PIN_DIO1 directly
// rather than waiting for a task-notification from the ISR.
//
// Why polling instead of ISR:
//   ulTaskNotifyTake fires on a rising edge of DIO1.  If DIO1 is already HIGH
//   from an incompletely-cleared prior RX event, ClearIrq may not bring it LOW
//   in time and there is NO rising edge when TX_DONE fires — the ISR never runs
//   and ulTaskNotifyTake times out (IRQ=0x0000).
//
// Sequence:
//   1  SetStandby(RC)  — valid from any state, including continuous RX
//   2  SetRfFrequency  — force PLL recalibration (Rx→Tx errata)
//   3  SetPktParams    — actual payload length
//   4  WriteBuffer     — load payload
//   5  SetDioIrqParams — TX_DONE | TIMEOUT on DIO1
//   6  Drain stale ISR notifications
//   7  ClearIrq        — bring DIO1 LOW before SetTx
//   8  SetTx(0)        — begin transmission, no hardware timeout
//   9  Poll BUSY HIGH  — confirms chip accepted SetTx
//  10  Poll BUSY LOW   — chip entered TX mode (TCXO + PLL ready)
//  11  Poll DIO1 HIGH  — TX_DONE fired (packet fully transmitted)
//  12  Read + clear IRQ, restore RX config, re-enter RX
//
// Must only be called from the LoRa task.
bool LoRa::transmit(const uint8_t* data, uint8_t len)
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    if (len == 0 || data == nullptr) return false;

    ESP_LOGD(TAG, "TX: %u bytes", len);

    // 1. Enter STBY_RC — always valid from any chip state
    { uint8_t t[] = { CMD_SET_STANDBY, 0x00 };
      _transact(t, nullptr, sizeof(t)); }

    // 2. Clear any stale device errors from a previous failed TX so
    //    _getDeviceErrors() after this TX gives a clean diagnostic.
    { uint8_t t[] = { CMD_CLEAR_DEVICE_ERRORS, 0x00, 0x00 };
      _transact(t, nullptr, sizeof(t)); }

    // 3. Re-issue SetRfFrequency to force a clean PLL lock (Rx→Tx errata)
    _setFrequency(LORA_FREQ_HZ);

    // 4. Set packet params with the exact TX payload length
    { uint8_t t[] = { CMD_SET_PKT_PARAMS,
                      0x00, 0x10, 0x00, len, 0x01, 0x00 };
      _transact(t, nullptr, sizeof(t)); }

    // 5. Load payload into SX1262 FIFO at TX base address (0x00)
    _writeBuffer(0x00, data, len);

    // 6. Configure DIO1 for TX events (TX_DONE | TIMEOUT)
    _setTxIrq();

    // 7. Discard any stale notification that the RX ISR may have queued
    ulTaskNotifyTake(pdTRUE, 0);

    // 8. Clear all IRQ flags and device errors before SetTx
    _clearIrq(0xFFFF);

    // 9. SetTx — no hardware timeout; chip transmits until TX_DONE
    { uint8_t t[] = { CMD_SET_TX, 0x00, 0x00, 0x00 };
      _transact(t, nullptr, sizeof(t)); }

    // ── Phase A: confirm chip accepted SetTx (BUSY goes HIGH) ────────────
    {
        const TickType_t t0 = xTaskGetTickCount();
        while (!gpio_get_level(PIN_BUSY))
        {
            if ((xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(100))
            {
                const uint16_t errs = _getDeviceErrors();
                const uint8_t  mode = _getChipMode();
                ESP_LOGE(TAG,
                    "TX: BUSY never HIGH — SetTx rejected "
                    "(mode=0x%02x errs=0x%04x)", mode, errs);
                portENTER_CRITICAL(&_statsLock);
                _stats.txErrors++;
                portEXIT_CRITICAL(&_statsLock);
                _initSx1262(); _setRx();
                return false;
            }
            vTaskDelay(1);
        }
    }

    // ── Phase B: wait for BUSY LOW (TCXO warm + PLL locked, TX active) ───
    _waitBusy(500);

    // ── Phase B2: confirm chip is actually in TX mode (mode == 6) ─────────
    {
        const uint8_t mode = _getChipMode();
        if (mode != 0x06)
        {
            // Chip left TX immediately — PA fault or calibration error
            const uint16_t errs = _getDeviceErrors();
            const uint16_t irqD = _getIrqStatus();
            ESP_LOGE(TAG,
                "TX: chip left TX mode immediately "
                "(mode=0x%02x irq=0x%04x errs=0x%04x) — "
                "PA_RAMP_ERR=%d XOSC_ERR=%d PLL_ERR=%d",
                mode, irqD, errs,
                (errs >> 8) & 1,   // bit 8 = PA_RAMP_ERR
                (errs >> 5) & 1,   // bit 5 = XOSC_START_ERR
                (errs >> 6) & 1);  // bit 6 = PLL_LOCK_ERR
            _clearIrq(0xFFFF);
            _setRxIrq();
            { uint8_t t2[] = { CMD_SET_PKT_PARAMS,
                               0x00, 0x10, 0x00, 0xFF, 0x01, 0x00 };
              _transact(t2, nullptr, sizeof(t2)); }
            portENTER_CRITICAL(&_statsLock);
            _stats.txErrors++;
            portEXIT_CRITICAL(&_statsLock);
            _initSx1262(); _setRx();
            return false;
        }
    }

    // ── Phase C: poll GetStatus until chip exits TX mode ─────────────────
    // The chip returns to STBY_RC (mode 2) when TX_DONE fires.
    // Correct air time for SF11/BW250/CR4-5 @ ~80 bytes ≈ 970 ms.
    // (The old BW codes 7/8/9 mapped to 10.42 kHz not 250 kHz, giving ~23s
    //  air time — that was the root cause of the 5-second timeout.)
    // 4-second limit is generous for all supported presets.
    {
        const TickType_t t0 = xTaskGetTickCount();
        while (true)
        {
            vTaskDelay(2); // 2 ms poll — well below the ~800 ms air time

            const uint8_t mode = _getChipMode();
            if (mode != 0x06)
                break; // chip left TX mode — TX_DONE (or error)

            if ((xTaskGetTickCount() - t0) >= pdMS_TO_TICKS(5000))
            {
                const uint16_t errs = _getDeviceErrors();
                const uint16_t irqD = _getIrqStatus();
                ESP_LOGE(TAG,
                    "TX: chip stuck in TX after 5 s "
                    "(mode=0x%02x irq=0x%04x errs=0x%04x)",
                    mode, irqD, errs);
                _clearIrq(0xFFFF);
                _setRxIrq();
                { uint8_t t2[] = { CMD_SET_PKT_PARAMS,
                                   0x00, 0x10, 0x00, 0xFF, 0x01, 0x00 };
                  _transact(t2, nullptr, sizeof(t2)); }
                portENTER_CRITICAL(&_statsLock);
                _stats.txTimeouts++;
                portEXIT_CRITICAL(&_statsLock);
                _initSx1262(); _setRx();
                return false;
            }
        }
    }

    // ── Chip exited TX — check IRQ for TX_DONE ────────────────────────────
    const uint16_t irq  = _getIrqStatus();
    const uint16_t errs = _getDeviceErrors();
    _clearIrq(0xFFFF);

    // Restore RX IRQ mask and max-payload packet params
    _setRxIrq();
    { uint8_t t[] = { CMD_SET_PKT_PARAMS,
                      0x00, 0x10, 0x00, 0xFF, 0x01, 0x00 };
      _transact(t, nullptr, sizeof(t)); }

    bool ok = false;
    if (irq & IRQ_TX_DONE)
    {
        ESP_LOGI(TAG, "TX complete (%u bytes)", len);
        portENTER_CRITICAL(&_statsLock);
        _stats.txPackets++;
        portEXIT_CRITICAL(&_statsLock);
        ok = true;
    }
    else
    {
        ESP_LOGW(TAG,
            "TX: chip exited TX but TX_DONE not set "
            "(irq=0x%04x errs=0x%04x "
            "PA_RAMP_ERR=%d XOSC_ERR=%d PLL_ERR=%d)",
            irq, errs,
            (errs >> 8) & 1,
            (errs >> 5) & 1,
            (errs >> 6) & 1);
        portENTER_CRITICAL(&_statsLock);
        _stats.txErrors++;
        portEXIT_CRITICAL(&_statsLock);
    }

    _setRx();
    return ok;
}

// ── _initSx1262 ───────────────────────────────────────────────────────────
bool LoRa::_initSx1262()
{
    // ── Hard reset ────────────────────────────────────────────────────────
    gpio_set_level(PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(PIN_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    _waitBusy(500);

    // ── SetStandby(STDBY_RC) ──────────────────────────────────────────────
    { uint8_t t[] = { CMD_SET_STANDBY, 0x00 }; _transact(t, nullptr, sizeof(t)); }

    // ── DIO3 → TCXO voltage control (1.8 V, 5 ms warm-up = 320 ticks @ 64 kHz)
    // Heltec Wireless Tracker 1.1 uses a 1.8 V TCXO on DIO3 (tcxoVoltage = 0x02).
    { uint8_t t[] = { CMD_SET_DIO3_TCXO, 0x02, 0x00, 0x01, 0x40 };
      _transact(t, nullptr, sizeof(t)); }

    // ── Calibrate all blocks (they run after TCXO is ready) ──────────────
    // bits: RC64k | RC13M | PLL | ADC_pulse | ADC_bulkN | ADC_bulkP | image
    { uint8_t t[] = { CMD_CALIBRATE, 0x7F }; _transact(t, nullptr, sizeof(t)); }
    _waitBusy(500);

    // ── CalibrateImage for the configured frequency band ──────────────────
    _calibrateImage(LORA_FREQ_HZ);
    _waitBusy(200);

    // ── DIO2 → RF switch control (drives TX/RX antenna switch) ───────────
    { uint8_t t[] = { CMD_SET_DIO2_RF_SWITCH, 0x01 }; _transact(t, nullptr, sizeof(t)); }

    // ── DC-DC regulator (SX1262 always has internal DCDC) ────────────────
    { uint8_t t[] = { CMD_SET_REGULATOR_MODE, 0x01 }; _transact(t, nullptr, sizeof(t)); }

    // ── LoRa packet type ──────────────────────────────────────────────────
    { uint8_t t[] = { CMD_SET_PACKET_TYPE, 0x01 }; _transact(t, nullptr, sizeof(t)); }

    // ── RF frequency ─────────────────────────────────────────────────────
    _setFrequency(LORA_FREQ_HZ);

    // ── PA config for HP PA on SX1262 ──────────────────────────────────────
    // paDutyCycle=4, hpMax=7, deviceSel=0 (SX1262), paLut=1
    { uint8_t t[] = { CMD_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 };
      _transact(t, nullptr, sizeof(t)); }

    // ── TX output power + ramp time 200 µs ──────────────────────────────
    // SX1262 SetTxParams power byte is signed: -9 to +22 dBm.
    // Cast via int8_t to handle negative values correctly in the SPI byte.
#if CONFIG_LORA_TX_ENABLED
    static constexpr int8_t txPow = static_cast<int8_t>(CONFIG_LORA_TX_POWER_DBM);
#else
    static constexpr int8_t txPow = 22; // irrelevant for RX-only, but must be valid
#endif
    { uint8_t t[] = { CMD_SET_TX_PARAMS,
                      static_cast<uint8_t>(txPow), 0x04 };
      _transact(t, nullptr, sizeof(t)); }

    // ── Modem preset (SF, BW, CR, LDRO) ──────────────────────────────────
    // BW codes: 0x04=125kHz  0x05=250kHz  0x06=500kHz  (SX1262 Table 13-48)
#if   defined(CONFIG_LORA_PRESET_LONG_FAST)
    _setModulation(11, 0x05, 1, 0); // SF11, 250 kHz, CR4/5, LDRO off
#elif defined(CONFIG_LORA_PRESET_LONG_SLOW)
    _setModulation(12, 0x04, 4, 1); // SF12, 125 kHz, CR4/8, LDRO on
#elif defined(CONFIG_LORA_PRESET_MEDIUM_SLOW)
    _setModulation(10, 0x05, 1, 0); // SF10, 250 kHz, CR4/5, LDRO off
#elif defined(CONFIG_LORA_PRESET_SHORT_FAST)
    _setModulation( 7, 0x05, 1, 0); // SF7,  250 kHz, CR4/5, LDRO off
#else
    _setModulation(11, 0x05, 1, 0); // default: LongFast (SF11/BW250)
#endif

    // ── Packet parameters ─────────────────────────────────────────────────
    // preamble=16, variable header, maxPayload=255, CRC on, IQ normal
    { uint8_t t[] = { CMD_SET_PKT_PARAMS,
                      0x00, 0x10, // preamble = 16 (MSB first)
                      0x00,       // explicit (variable-length) header
                      0xFF,       // max payload
                      0x01,       // CRC on
                      0x00 };     // IQ normal (not inverted)
      _transact(t, nullptr, sizeof(t)); }

    // ── SX1262 Errata 15.3 — fix IQ polarity register ────────────────────
    // Must be called after every SetPacketParams.  See _fixIqPolarity() for
    // the full explanation.  Without this, register 0x0736 bit 2 stays at
    // the power-on default (inverted IQ), causing every header CRC to fail.
    _fixIqPolarity();

    // ── RX gain ──────────────────────────────────────────────────────────
    // 0x94 = normal LNA gain (default after reset, handles full input range).
    // 0x96 = boosted LNA gain (+3 dB sensitivity) — NOT used here because
    // it reduces the maximum tolerable input level, causing CRC errors from
    // nearby nodes (e.g. nop jmp at -29 dBm → irq=0x0056 CRC_ERROR+RX_DONE).
    // With a noise floor of -104 dBm we have >70 dB of link margin on the
    // nearest node — sensitivity boost provides no benefit and causes harm.
    _writeReg(REG_RX_GAIN, 0x94);

    // ── Buffer base addresses (TX=0, RX=128 — separate regions to prevent
    //    overlap during the TX→RX transition) ────────────────────────────────
    { uint8_t t[] = { CMD_SET_BUF_BASE_ADDR, 0x00, 0x80 };
      _transact(t, nullptr, sizeof(t)); }

    // ── LoRa sync word = Meshtastic (0x2B in RadioLib = 0x24B4 in regs)
    _writeReg(REG_LORA_SYNC_MSB, SYNC_HI);
    _writeReg(REG_LORA_SYNC_LSB, SYNC_LO);

    // ── DIO1 IRQ: RX_DONE | HEADER_ERR | CRC_ERROR | TIMEOUT ────────────
    _setRxIrq();

    // ── Verify the sync word was written correctly ────────────────────────
    const uint8_t s0 = _readReg(REG_LORA_SYNC_MSB);
    const uint8_t s1 = _readReg(REG_LORA_SYNC_LSB);
    if (s0 != SYNC_HI || s1 != SYNC_LO)
    {
        ESP_LOGE(TAG, "SX1262 sync word readback failed (got 0x%02X%02X, want 0x%02X%02X) "
                 "— check SPI wiring", s0, s1, SYNC_HI, SYNC_LO);
        return false;
    }

    ESP_LOGI(TAG, "SX1262 ready — freq=%u Hz (slot=%u), sync=0x%02X%02X, iq_cfg=0x%02X, rx_gain=0x%02X",
             (unsigned)LORA_FREQ_HZ,
             (unsigned)((LORA_FREQ_HZ - 902125000) / 250000),
             SYNC_HI, SYNC_LO,
             _readReg(REG_IQ_CONFIG), _readReg(REG_RX_GAIN));
    return true;
}

// ── _decrypt ──────────────────────────────────────────────────────────────
// Meshtastic AES-128-CTR cipher (decrypt == encrypt for CTR mode).
// Used for BOTH decrypting incoming packets and encrypting outgoing ones.
//
// Nonce layout (CryptoEngine::initCounter() in Meshtastic firmware):
//   bytes  [0:3]  = packetId LE
//   bytes  [4:7]  = 0x00 × 4
//   bytes  [8:11] = fromNode LE
//   bytes [12:15] = 0x00 × 4
bool LoRa::_decrypt(const uint8_t* in, size_t len,
                    uint32_t packetId, uint32_t fromNode,
                    uint8_t* out)
{
    if (len == 0) return false;

    uint8_t nonce[16] = {};
    nonce[0]  = static_cast<uint8_t>(packetId);
    nonce[1]  = static_cast<uint8_t>(packetId >>  8);
    nonce[2]  = static_cast<uint8_t>(packetId >> 16);
    nonce[3]  = static_cast<uint8_t>(packetId >> 24);
    // [4:7]  = 0
    nonce[8]  = static_cast<uint8_t>(fromNode);
    nonce[9]  = static_cast<uint8_t>(fromNode >>  8);
    nonce[10] = static_cast<uint8_t>(fromNode >> 16);
    nonce[11] = static_cast<uint8_t>(fromNode >> 24);
    // [12:15] = 0

    uint8_t streamBlock[16] = {};
    size_t  nc_off = 0;

    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    bool ok = false;
    if (mbedtls_aes_setkey_enc(&ctx, DEFAULT_PSK, 128) == 0)
        ok = (mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce, streamBlock, in, out) == 0);
    mbedtls_aes_free(&ctx);
    return ok;
}

// ── _parseData ────────────────────────────────────────────────────────────
// Minimal protobuf decoder for the Meshtastic Data message.
//   Field 1 (portnum):       wire type 0 (varint)
//   Field 2 (payload):       wire type 2 (length-delimited)
//   Field 3 (want_response): wire type 0 (varint bool)
// Returns true when portnum was found; payload/payloadLen may be nullptr/0
// if field 2 was absent.
bool LoRa::_parseData(const uint8_t* data, size_t len,
                      uint32_t& portnum,
                      const uint8_t*& payload, size_t& payloadLen,
                      bool& wantResponse)
{
    portnum      = 0;
    payload      = nullptr;
    payloadLen   = 0;
    wantResponse = false;

    size_t pos = 0;
    while (pos < len)
    {
        // Read varint tag
        uint32_t tag   = 0;
        int      shift = 0;
        while (pos < len)
        {
            uint8_t b = data[pos++];
            tag |= static_cast<uint32_t>(b & 0x7F) << shift;
            if (!(b & 0x80)) break;
            shift += 7;
            if (shift > 28) return false; // malformed
        }

        const uint32_t fieldNum  = tag >> 3;
        const uint8_t  wireType  = tag & 0x07;

        if (wireType == 0) // varint
        {
            uint64_t val   = 0;
            int      vshift = 0;
            while (pos < len)
            {
                uint8_t b = data[pos++];
                val |= static_cast<uint64_t>(b & 0x7F) << vshift;
                if (!(b & 0x80)) break;
                vshift += 7;
                if (vshift > 63) return false;
            }
            if (fieldNum == 1) portnum      = static_cast<uint32_t>(val);
            if (fieldNum == 3) wantResponse = (val != 0);
        }
        else if (wireType == 2) // length-delimited
        {
            uint32_t msgLen = 0;
            int      lshift = 0;
            while (pos < len)
            {
                uint8_t b = data[pos++];
                msgLen |= static_cast<uint32_t>(b & 0x7F) << lshift;
                if (!(b & 0x80)) break;
                lshift += 7;
                if (lshift > 28) return false;
            }
            if (pos + msgLen > len) return false; // truncated
            if (fieldNum == 2)
            {
                payload    = data + pos;
                payloadLen = msgLen;
            }
            pos += msgLen;
        }
        else if (wireType == 5) pos += 4; // fixed32
        else if (wireType == 1) pos += 8; // fixed64
        else return false; // unknown wire type
    }

    return portnum != 0;
}

// ─────────────────────────────────────────────────────────────────────────
// Protobuf encoder helpers
// All functions write directly into caller-supplied buffers.  Buffers must
// be sized by the caller; sizes are documented per function.
// ─────────────────────────────────────────────────────────────────────────

// ── _pbVarint ─────────────────────────────────────────────────────────────
// Encode a 64-bit unsigned value as a protobuf base-128 varint.
// Returns the number of bytes written (1–10).
/* static */ size_t LoRa::_pbVarint(uint8_t* buf, uint64_t val)
{
    size_t n = 0;
    while (val > 0x7F)
    {
        buf[n++] = static_cast<uint8_t>((val & 0x7F) | 0x80);
        val >>= 7;
    }
    buf[n++] = static_cast<uint8_t>(val & 0x7F);
    return n;
}

// ── _pbZigzag ─────────────────────────────────────────────────────────────
// Zigzag-encode a signed 32-bit integer for protobuf sint32 wire format.
// Maps: 0→0, -1→1, 1→2, -2→3, 2→4 …  Formula: (n<<1) ^ (n>>31).
/* static */ uint32_t LoRa::_pbZigzag(int32_t val)
{
    return (static_cast<uint32_t>(val) << 1) ^ static_cast<uint32_t>(val >> 31);
}

// ── _pbLenField ───────────────────────────────────────────────────────────
// Write a length-delimited protobuf field: tag byte, length varint, raw data.
// tag must be pre-computed as (fieldNum << 3) | 2.
// buf must have at least 1 + 5 + dataLen bytes available.
/* static */ size_t LoRa::_pbLenField(uint8_t* buf, uint8_t tag,
                                       const uint8_t* data, size_t dataLen)
{
    size_t n = 0;
    buf[n++] = tag;
    n += _pbVarint(buf + n, dataLen);
    memcpy(buf + n, data, dataLen);
    n += dataLen;
    return n;
}

// ── _pbString ─────────────────────────────────────────────────────────────
// Write a protobuf string field (length-delimited, from a null-terminated src).
/* static */ size_t LoRa::_pbString(uint8_t* buf, uint8_t tag, const char* s)
{
    const size_t sLen = (s != nullptr) ? strlen(s) : 0;
    return _pbLenField(buf, tag,
                       reinterpret_cast<const uint8_t*>(s ? s : ""), sLen);
}

// ── _encodePosition ───────────────────────────────────────────────────────
// Encode a Meshtastic Position proto (meshtastic/mesh.proto).
//
// Field assignments verified against live OTA hex dumps from real Meshtastic
// nodes (firmware 2.5+, circa 2024-2026):
//
//   Field  1 (latitude_i,    sfixed32): tag 0x0D — degrees × 1e7
//   Field  2 (longitude_i,   sfixed32): tag 0x15 — degrees × 1e7
//   Field  3 (altitude,      int32):    tag 0x18 — metres above MSL (varint)
//   Field  4 (time,          sfixed32): tag 0x25 — UTC Unix seconds
//            *** Previously WRONG at field 9 (tag 0x4D). Field 9 is pos_flags
//            (varint), not time. Meshtastic discards positions without a valid
//            timestamp because the freshness check fails. ***
//   Field  5 (location_source,varint):  tag 0x28 — PositionSource::GPS = 2
//   Field 10 (ground_speed,   varint):  tag 0x50 — cm/s (100 cm/s = 1 m/s = 3.6 km/h)
//   Field 11 (ground_track,   varint):  tag 0x58 — heading × 100 (521 = 5.21°)
//   Field 14 (sats_in_view,   varint):  tag 0x70 — satellite count
//            *** Previously WRONG at field 7 (tag 0x38). Field 7 is
//            google_plus_code (string, wire type 2). Wire-type mismatch
//            caused receivers to skip the satellite count. ***
//   Field 18 (precision_bits, varint):  tag 0x90 0x01 — bits of lat/lon precision.
//            32 = full precision (all 32 bits valid). Meshtastic app uses this
//            to decide how precisely to display the location. Absent or 0 means
//            "apply maximum privacy blur" which makes tracker positions useless.
//
// Removed: field 6 PDOP (field 6 is altitude_source enum in current proto,
//          not PDOP; real Meshtastic nodes don't send PDOP in position packets).
//
// buf must be at least 80 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodePosition(uint8_t* buf, size_t /*cap*/,
                                           int32_t lat_i, int32_t lon_i,
                                           int32_t alt_m,
                                           uint32_t pdop_x100, uint32_t sats,
                                           uint32_t unixTime,
                                           uint32_t speed_cm_s,
                                           uint32_t track_x100)
{
    size_t n = 0;

    // Helper: write a 32-bit value as little-endian fixed (wire type 5)
    auto writeFixed32 = [&](uint8_t tag, uint32_t val) {
        buf[n++] = tag;
        buf[n++] = static_cast<uint8_t>(val);
        buf[n++] = static_cast<uint8_t>(val >>  8);
        buf[n++] = static_cast<uint8_t>(val >> 16);
        buf[n++] = static_cast<uint8_t>(val >> 24);
    };

    // Helper: write a varint field (single-byte tag only — works for fields 1-15)
    auto writeVarint = [&](uint8_t tag, uint64_t val) {
        buf[n++] = tag;
        n += _pbVarint(buf + n, val);
    };

    // Field 1: latitude_i (sfixed32) — always present
    writeFixed32(0x0D, static_cast<uint32_t>(lat_i));

    // Field 2: longitude_i (sfixed32) — always present
    writeFixed32(0x15, static_cast<uint32_t>(lon_i));

    // Field 3: altitude (int32, varint) — skip if zero
    if (alt_m != 0)
        writeVarint(0x18, static_cast<uint64_t>(static_cast<int64_t>(alt_m)));

    // Field 4: time (sfixed32) — UTC Unix seconds
    // *** This was the critical bug: previously encoded at field 9 (0x4D).
    //     Field 9 is pos_flags (varint). Meshtastic nodes consider positions
    //     without a recent timestamp as stale and don't forward them. ***
    if (unixTime != 0)
        writeFixed32(0x25, unixTime);

    // Field 5: location_source = GPS (PositionSource::GPS = 2, varint)
    writeVarint(0x28, 2);

    // Field 10: ground_speed (varint, cm/s) — skip if zero
    if (speed_cm_s != 0)
        writeVarint(0x50, speed_cm_s);

    // Field 11: ground_track (varint, 0.01 degrees) — skip if zero
    if (track_x100 != 0)
        writeVarint(0x58, track_x100);

    // Field 14: sats_in_view (varint)
    // *** Previously WRONG at field 7 (google_plus_code, string wire type). ***
    if (sats != 0)
        writeVarint(0x70, sats);

    // Field 18: precision_bits (varint) — tag is 2 bytes because field > 15
    // (18 << 3) | 0 = 144 = 0x90, encoded as 0x90 0x01 in varint.
    // Value 32 = all 32 bits of lat/lon are significant (full GPS precision).
    // Without this the Meshtastic app may apply a privacy blur and show only
    // an approximate position, making this device useless as a tracker.
    buf[n++] = 0x90; buf[n++] = 0x01;  // tag: field 18, wire type 0 (varint)
    n += _pbVarint(buf + n, 32);

    // Suppress unused-parameter warnings for pdop_x100 (no longer encoded;
    // field 6 is altitude_source enum in current proto, not PDOP)
    (void)pdop_x100;

    return n;
}

// ── _encodeUser ───────────────────────────────────────────────────────────
// Encode a Meshtastic User proto (NODEINFO_APP payload).
//
// meshtastic/mesh.proto User message fields sent:
//   Field 1 (id,           string):  "!xxxxxxxx"
//   Field 2 (long_name,    string):  up to 32 chars
//   Field 3 (short_name,   string):  up to 4 chars
//   Field 4 (macaddr,      bytes):   6-byte BT MAC
//   Field 5 (hw_model,     enum):    HardwareModel varint (tag 0x28)
//   Field 7 (role,         enum):    DeviceRole varint (tag 0x38); omitted when 0 = CLIENT
//   Field 8 (public_key,   bytes):   32-byte X25519 public key (tag 0x42)
//   Field 9 (is_unmessageable, bool):false (tag 0x48), required by Meshtastic 2.5+
//
// DeviceRole enum (meshtastic/config.proto):
//   CLIENT = 0 (proto3 default — field 7 omitted)
//   TRACKER = 5 (GPS tracker icon, Smart Position)
//
// buf must be at least 120 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeUser(uint8_t* buf, size_t /*cap*/,
                                       uint32_t nodeId,
                                       const char* longName,
                                       const char* shortName)
{
    size_t n = 0;

    // Field 1: id — "!xxxxxxxx"
    char idStr[12] = {};
    snprintf(idStr, sizeof(idStr), "!%08" PRIx32, nodeId);
    n += _pbString(buf + n, 0x0A, idStr);

    // Field 2: long_name
    n += _pbString(buf + n, 0x12, longName);

    // Field 3: short_name
    n += _pbString(buf + n, 0x1A, shortName);

    // Field 4: macaddr (bytes, 6 bytes) — tag = (4<<3)|2 = 0x22
    n += _pbLenField(buf + n, 0x22, Node.macaddr(), 6);

    // Field 5: hw_model (HardwareModel enum, varint) — tag = (5<<3)|0 = 0x28
    buf[n++] = 0x28;
    n += _pbVarint(buf + n, HW_MODEL);

    // Field 7: role (DeviceRole enum, varint) — tag = (7<<3)|0 = 0x38
    // Only encoded when non-zero; proto3 default (0 = CLIENT) needs no wire bytes.
    // TRACKER = 5 shows the GPS tracker icon in the Meshtastic app and enables
    // Smart Position behaviour on the receiving firmware.
#if CONFIG_MESH_NODE_ROLE != 0
    buf[n++] = 0x38;
    n += _pbVarint(buf + n, CONFIG_MESH_NODE_ROLE);
#endif

    // Field 8: public_key — omitted in IsLicensed (unencrypted) mode.
    // No PKC keys are generated or exchanged.

    // Field 9: is_unmessageable (bool, varint) — tag = (9<<3)|0 = 0x48
    buf[n++] = 0x48;
    buf[n++] = 0x00;

    return n;
}

// ── _encodeData ───────────────────────────────────────────────────────────
// Wrap an encoded application payload in a Meshtastic Data proto.
//
// meshtastic/mesh.proto Data message — verified against live hex dumps:
//   Field 1 (portnum,       PortNum):  varint      — tag = 0x08
//   Field 2 (payload,       bytes):    LEN         — tag = 0x12
//   Field 3 (want_response, bool):     varint      — tag = 0x18 (broadcasts only)
//   Field 4 (dest,          fixed32):  4-byte LE   — tag = 0x25 (unicast only)
//   Field 6 (request_id,    fixed32):  4-byte LE   — tag = 0x35 (responses only)
//   Field 9 (ok_to_mqtt,    bool):     varint      — tag = 0x48 (unicast responses)
//
// *** BUG HISTORY (field 6 vs 7): We previously used tag 0x3D (field 7 = reply_id)
// but Meshtastic firmware uses field 6 (request_id, tag 0x35) in responses.
// Meshtastic's Router.cpp correlates responses by looking for a pending request
// where stored_pktId == incoming.request_id.  Our reply_id (field 7) was never
// checked → the correlation always failed → sendToPhone() was never triggered
// → the phone app never saw us. Confirmed via live hex dump from nop jmp:
//   "35 70 bc 3d 7d" = tag 0x35 (field 6 request_id), fixed32 = our pktId. ***
//
// buf must be at least payloadLen + 32 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeData(uint8_t* buf, size_t /*cap*/,
                                       uint32_t portnum,
                                       const uint8_t* payload, size_t payloadLen,
                                       bool want_response,
                                       uint32_t dest,
                                       uint32_t source,
                                       uint32_t requestId)
{
    size_t n = 0;

    // Helper: write a fixed32 field (tag byte, 4 bytes LE)
    auto writeFixed32 = [&](uint8_t tag, uint32_t val) {
        buf[n++] = tag;
        buf[n++] = static_cast<uint8_t>(val);
        buf[n++] = static_cast<uint8_t>(val >>  8);
        buf[n++] = static_cast<uint8_t>(val >> 16);
        buf[n++] = static_cast<uint8_t>(val >> 24);
    };

    // Field 1: portnum (varint)
    buf[n++] = 0x08;
    n += _pbVarint(buf + n, portnum);

    // Field 2: payload (length-delimited bytes)
    n += _pbLenField(buf + n, 0x12, payload, payloadLen);

    // Field 3: want_response — only set for broadcast requests, never in responses
    if (want_response)
    {
        buf[n++] = 0x18;
        buf[n++] = 0x01;
    }

    // Field 4: dest (fixed32) — unicast destination node ID
    if (dest != 0)
        writeFixed32(0x25, dest);

    // source (field 5) intentionally omitted — only set by relay nodes, not originators

    // Field 6: request_id (fixed32) — tag = (6<<3)|5 = 0x35
    // Set by the RESPONDER to the incoming packet's pktId so the receiving
    // firmware can match it to the pending want_response request and call
    // sendToPhone() to notify the connected phone app.
    if (requestId != 0)
    {
        writeFixed32(0x35, requestId);
        // Field 9: ok_to_mqtt=false — included alongside request_id in unicast
        // responses to indicate the packet should not be forwarded to MQTT.
        // All real Meshtastic nodes include this explicitly in unicast responses.
        buf[n++] = 0x48; // tag (9<<3)|0
        buf[n++] = 0x00; // false
    }

    return n;
}

// ── _buildTxPacket ────────────────────────────────────────────────────────
// Build a complete, AES-128-CTR encrypted, ready-to-transmit Meshtastic
// OTA packet.
//
// Layout of out[]:
//   [0..15]  16-byte OTA header (plaintext)
//   [16..]   AES-128-CTR encrypted Data proto
//
// out must be at least 256 bytes.  Returns false only on encoding error.
bool LoRa::_buildTxPacket(uint8_t* out, uint8_t& outLen,
                           uint32_t portnum,
                           const uint8_t* payload, size_t payloadLen,
                           bool want_response,
                           uint32_t to,
                           uint32_t requestId)
{
    if (payload == nullptr || payloadLen == 0 || payloadLen > 220) return false;

    const uint32_t packetId = Node.nextPacketId();
    const uint32_t fromNode = Node.nodeId();
    const bool     unicast  = (to != 0xFFFFFFFF);

    // 1. Encode Data proto.
    //    Broadcasts: portnum + payload + want_response
    //    Unicast responses: portnum + payload + dest + request_id + ok_to_mqtt=false
    uint8_t dataBuf[244] = {};
    const size_t dataLen = _encodeData(dataBuf, sizeof(dataBuf),
                                        portnum, payload, payloadLen,
                                        want_response,
                                        unicast ? to : 0,          // dest (field 4)
                                        0,                         // source — omitted for self-originated
                                        unicast ? requestId : 0);  // request_id (field 6)
    if (dataLen == 0 || dataLen > 239) return false;

    // Hex dump of plaintext Data proto for wire-level debugging.
    // Compare against RX hex dumps from real Meshtastic nodes to find
    // any encoding differences.
    {
        char hex[120 * 3 + 1] = {};
        for (size_t i = 0; i < dataLen && i < 120; i++)
            snprintf(hex + i * 3, 4, "%02x ", dataBuf[i]);
        ESP_LOGI(TAG, "TX Data proto (%u bytes): %s", (unsigned)dataLen, hex);
    }

    // 2. Build 16-byte OTA header
    out[0] = static_cast<uint8_t>(to);
    out[1] = static_cast<uint8_t>(to >>  8);
    out[2] = static_cast<uint8_t>(to >> 16);
    out[3] = static_cast<uint8_t>(to >> 24);
    out[4] = static_cast<uint8_t>(fromNode);
    out[5] = static_cast<uint8_t>(fromNode >>  8);
    out[6] = static_cast<uint8_t>(fromNode >> 16);
    out[7] = static_cast<uint8_t>(fromNode >> 24);
    out[8]  = static_cast<uint8_t>(packetId);
    out[9]  = static_cast<uint8_t>(packetId >>  8);
    out[10] = static_cast<uint8_t>(packetId >> 16);
    out[11] = static_cast<uint8_t>(packetId >> 24);
    out[12] = unicast ? FLAGS_UNICAST : FLAGS_BROADCAST;
    out[13] = DEFAULT_CHAN_HASH;
    out[14] = 0x00;
    out[15] = 0x00;

    // 3. Encrypt Data proto with AES-128-CTR (encrypt == decrypt for CTR mode)
    if (!_decrypt(dataBuf, dataLen, packetId, fromNode, out + MESH_HDR))
    {
        ESP_LOGW(TAG, "_buildTxPacket: AES-CTR encrypt failed");
        return false;
    }
    outLen = static_cast<uint8_t>(MESH_HDR + dataLen);

    ESP_LOGI(TAG,
        "OTA hdr: %02x%02x%02x%02x %02x%02x%02x%02x "
        "%02x%02x%02x%02x flags=%02x hash=%02x  port=%u  pktlen=%u",
        out[0],  out[1],  out[2],  out[3],
        out[4],  out[5],  out[6],  out[7],
        out[8],  out[9],  out[10], out[11],
        out[12], out[13],
        portnum, (unsigned)outLen);

    return true;
}

// ── sendPosition ──────────────────────────────────────────────────────────
bool LoRa::sendPosition(double lat, double lng, float altM,
                         uint32_t pdop_x100, uint32_t sats, uint32_t unixTime)
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    // If no timestamp supplied, use the system clock (synced via BLE CTS or GPS)
    if (unixTime == 0)
        unixTime = static_cast<uint32_t>(time(nullptr));

    // Convert decimal degrees to Meshtastic integer format (degrees × 1e7)
    const int32_t lat_i = static_cast<int32_t>(lat * 1e7);
    const int32_t lon_i = static_cast<int32_t>(lng * 1e7);
    const int32_t alt_m = static_cast<int32_t>(altM);

    // Ground speed: GPS gives km/h, Meshtastic wants cm/s.
    // 1 km/h = 100/3.6 cm/s ≈ 27.78 cm/s
    const float  speedKmh   = gps.speed();
    const float  courseDeg  = gps.course();
    const uint32_t speed_cm_s = static_cast<uint32_t>(speedKmh * 100.0f / 3.6f);
    // Ground track: Meshtastic stores heading as integer degrees × 100
    // (i.e. 1° = 100, 180° = 18000).  Skip if speed is negligible (< 2 km/h)
    // to avoid noisy heading values when stationary.
    const uint32_t track_x100 = (speedKmh >= 2.0f)
                               ? static_cast<uint32_t>(courseDeg * 100.0f)
                               : 0;

    // Encode Position proto (~100 bytes covers all fields including precision_bits)
    uint8_t posBuf[100] = {};
    const size_t posLen = _encodePosition(posBuf, sizeof(posBuf),
                                           lat_i, lon_i, alt_m,
                                           pdop_x100, sats, unixTime,
                                           speed_cm_s, track_x100);
    if (posLen == 0) return false;

    // Build and transmit the complete OTA packet
    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_POSITION, posBuf, posLen,
                        /*want_response=*/false)) return false;

    ESP_LOGI(TAG, "TX POSITION lat=%.6f lon=%.6f alt=%dm sats=%" PRIu32
             " spd=%.1fkm/h hdg=%.1f ts=%" PRIu32,
             lat, lng, (int)altM, sats, (double)speedKmh, (double)courseDeg, unixTime);

    return transmit(pkt, pktLen);
}

// ── sendNodeInfo ──────────────────────────────────────────────────────────
bool LoRa::sendNodeInfo(uint32_t to, bool wantResponse, uint32_t requestId)
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    uint8_t userBuf[128] = {};
    const size_t userLen = _encodeUser(userBuf, sizeof(userBuf),
                                        Node.nodeId(),
                                        Node.longName(),
                                        Node.shortName());
    if (userLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_NODEINFO, userBuf, userLen,
                        wantResponse, to, requestId)) return false;

    ESP_LOGI(TAG, "TX NODEINFO id=%s long=\"%s\" short=\"%s\" hw=%u to=0x%08" PRIx32
             " req_id=0x%08" PRIx32,
             Node.nodeIdStr(), Node.longName(), Node.shortName(),
             (unsigned)HW_MODEL, to, requestId);

    return transmit(pkt, pktLen);
}

// ── _encodeTelemetry ──────────────────────────────────────────────────────
// Encode a Meshtastic Telemetry proto with Device Metrics.
//
// meshtastic/telemetry.proto:
//   Field 1 (time,           fixed32):  UTC Unix seconds — tag = 0x0D
//   Field 2 (device_metrics, message):  oneof variant — tag = 0x12
//     Inner meshtastic/telemetry.proto DeviceMetrics:
//       Field 1 (battery_level, uint32): 0-100 %   — tag = 0x08
//       Field 2 (voltage,       float):  volts     — tag = 0x15 (fixed32)
//       Field 5 (uptime_seconds,uint32): varint    — tag = 0x28
//
// buf must be at least 32 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeTelemetry(uint8_t* buf, size_t /*cap*/,
                                            uint32_t unixTime, uint32_t uptimeSec,
                                            uint8_t batteryLevel, float batteryVoltage)
{
    // Helper: write a 32-bit value as little-endian fixed (wire type 5)
    auto writeFixed32 = [](uint8_t* dst, uint8_t tag, uint32_t val) -> size_t {
        dst[0] = tag;
        dst[1] = static_cast<uint8_t>(val);
        dst[2] = static_cast<uint8_t>(val >> 8);
        dst[3] = static_cast<uint8_t>(val >> 16);
        dst[4] = static_cast<uint8_t>(val >> 24);
        return 5;
    };

    // Inner DeviceMetrics message
    uint8_t dm[20] = {};
    size_t  dmn    = 0;

    // DeviceMetrics field 1: battery_level (uint32, varint) — tag = 0x08
    dm[dmn++] = 0x08;
    dmn += _pbVarint(dm + dmn, batteryLevel);

    // DeviceMetrics field 2: voltage (float, fixed32) — tag = (2<<3)|5 = 0x15
    {
        uint32_t vbits;
        memcpy(&vbits, &batteryVoltage, sizeof(vbits));
        dmn += writeFixed32(dm + dmn, 0x15, vbits);
    }

    // DeviceMetrics field 5: uptime_seconds (uint32, varint) — tag = 0x28
    dm[dmn++] = 0x28;
    dmn += _pbVarint(dm + dmn, uptimeSec);

    size_t n = 0;

    // Telemetry field 1: time (fixed32) — tag = (1<<3)|5 = 0x0D
    n += writeFixed32(buf + n, 0x0D, unixTime);

    // Telemetry field 2: device_metrics (embedded message) — tag = 0x12
    n += _pbLenField(buf + n, 0x12, dm, dmn);

    return n;
}

// ── sendTelemetry ─────────────────────────────────────────────────────────
// Broadcast a TELEMETRY_APP Device Metrics packet so other Meshtastic nodes
// mark this device as "active" and show it in their app node lists.
bool LoRa::sendTelemetry()
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    const uint32_t now    = static_cast<uint32_t>(time(nullptr));
    const uint32_t uptime = static_cast<uint32_t>(
        xTaskGetTickCount() / (TickType_t)configTICK_RATE_HZ);

    // Use cached battery values (refreshed every 30 s by the battery timer).
    // Avoids blocking the LoRa TX task for the ADC settle delay.
    const uint8_t batLevel   = Heltec.cachedBatteryLevel();
    const float   batVoltage = Heltec.cachedBatteryVoltage();

    uint8_t telBuf[40] = {};
    const size_t telLen = _encodeTelemetry(telBuf, sizeof(telBuf), now, uptime,
                                            batLevel, batVoltage);
    if (telLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_TELEMETRY, telBuf, telLen,
                        /*want_response=*/false)) return false;

    ESP_LOGI(TAG, "TX TELEMETRY uptime=%us bat=%u%% %.2fV time=%" PRIu32,
             (unsigned)uptime, batLevel, (double)batVoltage, now);
    return transmit(pkt, pktLen);
}

// ── _encodeMapReport ──────────────────────────────────────────────────────
// Encode a Meshtastic MapReport proto (MAP_REPORT_APP payload, port 73).
//
// MapReport is sent periodically to MQTT bridges so this node appears on
// the public Meshtastic map (meshtastic.network/map).  Fields verified
// against Meshtastic firmware mesh.proto 2.5+:
//
//   Field  1 (long_name,          string):  tag 0x0A
//   Field  2 (short_name,         string):  tag 0x12
//   Field  3 (hw_model,           varint):  tag 0x18  HardwareModel enum
//   Field  4 (firmware_version,   string):  tag 0x22  (omitted — not tracked)
//   Field  5 (region,             varint):  tag 0x28  RegionCode enum
//   Field  6 (modem_preset,       varint):  tag 0x30  ModemPreset enum
//   Field  7 (has_default_channel,bool):    tag 0x38  true when using factory PSK
//   Field  8 (latitude_i,         fixed32): tag 0x45  degrees × 1e7
//   Field  9 (longitude_i,        fixed32): tag 0x4D  degrees × 1e7
//   Field 10 (altitude,           varint):  tag 0x50  metres above MSL
//   Field 11 (position_precision, varint):  tag 0x58  32 = full GPS precision
//   Field 12 (num_online_local_nodes,varint):tag 0x60 neighbour count
//
// ModemPreset enum (config.proto): LONG_FAST=0, LONG_SLOW=1, MEDIUM_SLOW=3, SHORT_FAST=6
// RegionCode enum  (config.proto): US=1, EU_868=3, ANZ=6, IN=10, SG_923=18
//
// buf must be at least 120 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeMapReport(uint8_t* buf, size_t /*cap*/,
                                            const char* longName,
                                            const char* shortName,
                                            int32_t lat_i, int32_t lon_i,
                                            int32_t alt_m,
                                            uint32_t numNeighbors)
{
    size_t n = 0;

    // Helper: write a fixed32 field (tag + 4 bytes LE)
    auto writeFixed32 = [&](uint8_t tag, uint32_t val) {
        buf[n++] = tag;
        buf[n++] = static_cast<uint8_t>(val);
        buf[n++] = static_cast<uint8_t>(val >>  8);
        buf[n++] = static_cast<uint8_t>(val >> 16);
        buf[n++] = static_cast<uint8_t>(val >> 24);
    };

    // Field 1: long_name (string)
    n += _pbString(buf + n, 0x0A, longName);

    // Field 2: short_name (string)
    n += _pbString(buf + n, 0x12, shortName);

    // Field 3: hw_model (varint)
    buf[n++] = 0x18;
    n += _pbVarint(buf + n, HW_MODEL);

    // Field 5: region (RegionCode varint) — from CONFIG_LORA_REGION_CODE
    buf[n++] = 0x28;
    n += _pbVarint(buf + n, static_cast<uint64_t>(CONFIG_LORA_REGION_CODE));

    // Field 6: modem_preset (ModemPreset varint) — derived from Kconfig choice
    // LONG_FAST=0, LONG_SLOW=1, MEDIUM_SLOW=3, SHORT_FAST=6
#if   defined(CONFIG_LORA_PRESET_LONG_FAST)
    static constexpr uint32_t MODEM_PRESET = 0;
#elif defined(CONFIG_LORA_PRESET_LONG_SLOW)
    static constexpr uint32_t MODEM_PRESET = 1;
#elif defined(CONFIG_LORA_PRESET_MEDIUM_SLOW)
    static constexpr uint32_t MODEM_PRESET = 3;
#elif defined(CONFIG_LORA_PRESET_SHORT_FAST)
    static constexpr uint32_t MODEM_PRESET = 6;
#else
    static constexpr uint32_t MODEM_PRESET = 0; // default LongFast
#endif
    buf[n++] = 0x30;
    n += _pbVarint(buf + n, MODEM_PRESET);

    // Field 7: has_default_channel (bool) — true, we use the factory LongFast PSK
    buf[n++] = 0x38;
    buf[n++] = 0x01;

    // Fields 8 & 9: latitude_i, longitude_i (sfixed32)
    writeFixed32(0x45, static_cast<uint32_t>(lat_i));
    writeFixed32(0x4D, static_cast<uint32_t>(lon_i));

    // Field 10: altitude (int32 varint)
    if (alt_m != 0)
    {
        buf[n++] = 0x50;
        n += _pbVarint(buf + n, static_cast<uint64_t>(static_cast<int64_t>(alt_m)));
    }

    // Field 11: position_precision = 32 (full GPS precision)
    buf[n++] = 0x58;
    n += _pbVarint(buf + n, 32);

    // Field 12: num_online_local_nodes (neighbour count from our table)
    if (numNeighbors > 0)
    {
        buf[n++] = 0x60;
        n += _pbVarint(buf + n, numNeighbors);
    }

    return n;
}

// ── sendMapReport ─────────────────────────────────────────────────────────
// Broadcast a MAP_REPORT_APP (port 73) packet for public mesh map visibility.
bool LoRa::sendMapReport()
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif
#if CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC == 0
    return false; // disabled via Kconfig
#endif

    if (!gps.isFixed())
    {
        ESP_LOGD(TAG, "MAP_REPORT skipped — no GPS fix");
        return false;
    }

    const int32_t lat_i = static_cast<int32_t>(gps.lat() * 1e7);
    const int32_t lon_i = static_cast<int32_t>(gps.lng() * 1e7);
    const int32_t alt_m = static_cast<int32_t>(gps.altitude());

    uint8_t mrBuf[160] = {};
    const size_t mrLen = _encodeMapReport(mrBuf, sizeof(mrBuf),
                                           Node.longName(), Node.shortName(),
                                           lat_i, lon_i, alt_m,
                                           neighborCount());
    if (mrLen == 0) return false;

    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_MAP_REPORT, mrBuf, mrLen,
                        /*want_response=*/false)) return false;

    ESP_LOGI(TAG, "TX MAP_REPORT lat=%.6f lon=%.6f alt=%dm neighbors=%u role=%u",
             (double)lat_i / 1e7, (double)lon_i / 1e7, (int)alt_m,
             (unsigned)neighborCount(), (unsigned)CONFIG_MESH_NODE_ROLE);
    return transmit(pkt, pktLen);
}

// ─────────────────────────────────────────────────────────────────────────
// Neighbour table & RX decoder helpers (Phase 5)
// ─────────────────────────────────────────────────────────────────────────

// ── _parsePosition ────────────────────────────────────────────────────────
// Decode a Meshtastic Position proto payload.
//
// Verified field assignments (Meshtastic firmware 2.5+):
//   Field  1 (latitude_i,    sfixed32): tag 0x0D — degrees × 1e7
//   Field  2 (longitude_i,   sfixed32): tag 0x15 — degrees × 1e7
//   Field  3 (altitude,      int32):    tag 0x18 — metres (varint)
//   Field  4 (time,          sfixed32): tag 0x25 — UTC Unix seconds
//            *** NOT field 9. Field 9 is pos_flags (varint). ***
//   Field  5 (location_source,varint):  tag 0x28 — skip
//   Field 10 (ground_speed,   varint):  tag 0x50 — cm/s
//   Field 11 (ground_track,   varint):  tag 0x58 — degrees × 100
//   Field 14 (sats_in_view,   varint):  tag 0x70 — satellite count
//            *** NOT field 7. Field 7 is google_plus_code (string). ***
//   Field 18 (precision_bits, varint):  tag 0x90 0x01 — skip
/* static */ bool LoRa::_parsePosition(const uint8_t* data, size_t len,
                                        MeshPosition& pos)
{
    size_t p = 0;
    bool gotLatLon = false;

    // Helper: read a little-endian uint32 at position p (must have 4 bytes)
    auto readFixed32 = [&]() -> uint32_t {
        uint32_t v = data[p] | (uint32_t)data[p+1]<<8
                   | (uint32_t)data[p+2]<<16 | (uint32_t)data[p+3]<<24;
        p += 4;
        return v;
    };

    while (p < len)
    {
        // Read tag varint (may be 2 bytes for high-numbered fields like 18)
        uint32_t tag = 0; int shift = 0;
        while (p < len) {
            uint8_t b = data[p++];
            tag |= (uint32_t)(b & 0x7F) << shift;
            if (!(b & 0x80)) break;
            shift += 7; if (shift > 28) return false;
        }
        const uint32_t field    = tag >> 3;
        const uint8_t  wireType = tag & 0x07;

        if (wireType == 0) // varint
        {
            uint64_t val = 0; int vs = 0;
            while (p < len) {
                uint8_t b = data[p++];
                val |= (uint64_t)(b & 0x7F) << vs;
                if (!(b & 0x80)) break;
                vs += 7; if (vs > 63) return false;
            }
            switch (field) {
                case 3:  pos.alt_m    = (int32_t)(int64_t)val; break;
                // Field 9 is pos_flags (uint32), NOT time — skip it
                // Field 14 is sats_in_view (correct)
                case 14: pos.sats     = (uint32_t)val; break;
                default: break;
            }
        }
        else if (wireType == 5) // fixed32 / sfixed32
        {
            if (p + 4 > len) return false;
            uint32_t val = readFixed32();
            switch (field) {
                case 1: // latitude_i (sfixed32)
                    pos.lat_i = static_cast<int32_t>(val);
                    gotLatLon = true;
                    break;
                case 2: // longitude_i (sfixed32)
                    pos.lon_i = static_cast<int32_t>(val);
                    break;
                case 4: // time (sfixed32) — correct field for timestamp
                    pos.unixTime = val;
                    break;
                default: break;
            }
        }
        else if (wireType == 2) // length-delimited — skip
        {
            uint32_t msgLen = 0; int ls = 0;
            while (p < len) {
                uint8_t b = data[p++];
                msgLen |= (uint32_t)(b & 0x7F) << ls;
                if (!(b & 0x80)) break;
                ls += 7; if (ls > 28) return false;
            }
            if (p + msgLen > len) return false;
            p += msgLen;
        }
        else if (wireType == 1) { if (p + 8 > len) return false; p += 8; }
        else return false;
    }
    return gotLatLon;
}

// ── _parseUser ────────────────────────────────────────────────────────────
// Decode a Meshtastic User proto payload.
//   Field 1 (id,         string): "!xxxxxxxx"
//   Field 2 (long_name,  string): up to 32 chars
//   Field 3 (short_name, string): up to 4 chars
//   Field 5 (hw_model,   uint32): HardwareModel enum
/* static */ bool LoRa::_parseUser(const uint8_t* data, size_t len,
                                    MeshUser& user)
{
    size_t p = 0;
    bool gotId = false;

    // Helper lambda to read a varint at p
    auto readVarint = [&](uint64_t& out) -> bool {
        out = 0; int s = 0;
        while (p < len) {
            uint8_t b = data[p++];
            out |= (uint64_t)(b & 0x7F) << s;
            if (!(b & 0x80)) return true;
            s += 7; if (s > 63) return false;
        }
        return false;
    };

    while (p < len)
    {
        uint64_t tag64 = 0;
        if (!readVarint(tag64)) break;
        const uint32_t field    = (uint32_t)(tag64 >> 3);
        const uint8_t  wireType = (uint8_t)(tag64 & 0x07);

        if (wireType == 2) // string / bytes
        {
            uint64_t slen = 0;
            if (!readVarint(slen)) return false;
            if (p + slen > len) return false;
            const char* src = reinterpret_cast<const char*>(data + p);
            switch (field) {
                case 1: // id
                    snprintf(user.id, sizeof(user.id), "%.*s",
                             (int)std::min(slen, (uint64_t)(sizeof(user.id) - 1)), src);
                    gotId = true;
                    break;
                case 2: // long_name
                    snprintf(user.longName, sizeof(user.longName), "%.*s",
                             (int)std::min(slen, (uint64_t)(sizeof(user.longName) - 1)), src);
                    break;
                case 3: // short_name
                    snprintf(user.shortName, sizeof(user.shortName), "%.*s",
                             (int)std::min(slen, (uint64_t)(sizeof(user.shortName) - 1)), src);
                    break;
                case 4: // macaddr — skip (6 bytes)
                    break;
                case 8: // public_key (32 bytes) — needed for PKC decryption
                    if (slen == 32) {
                        memcpy(user.publicKey, src, 32);
                        user.hasPublicKey = true;
                    }
                    break;
                default: break;
            }
            p += (size_t)slen;
        }
        else if (wireType == 0) // varint
        {
            uint64_t val = 0;
            if (!readVarint(val)) return false;
            if (field == 5) user.hwModel = (uint32_t)val;
        }
        else if (wireType == 5) { p += 4; }
        else if (wireType == 1) { p += 8; }
        else return false;
    }
    return gotId;
}

// ── _upsertNeighbor ───────────────────────────────────────────────────────
// Insert or update the neighbour table entry for fromNode.
// pos/user may be nullptr to leave that field unchanged on an existing entry.
void LoRa::_upsertNeighbor(uint32_t fromNode,
                             const MeshPosition* pos,
                             const MeshUser*     user)
{
    portENTER_CRITICAL(&_neighborLock);

    // Find existing slot or the oldest empty slot
    int slot = -1;
    TickType_t oldest = portMAX_DELAY;
    int oldestSlot = 0;

    for (int i = 0; i < (int)NEIGHBOR_MAX; i++)
    {
        if (_neighbors[i].occupied &&
            (_neighbors[i].pos.fromNode == fromNode ||
             _neighbors[i].user.fromNode == fromNode))
        {
            slot = i;
            break;
        }
        if (!_neighbors[i].occupied)
        {
            slot = i;
            break;
        }
        // Track oldest for eviction
        const TickType_t age = _neighbors[i].pos.lastSeen;
        if (age < oldest) { oldest = age; oldestSlot = i; }
    }
    if (slot == -1) slot = oldestSlot; // evict oldest

    // If this slot belongs to a DIFFERENT node (eviction or fresh empty slot),
    // clear it entirely to prevent stale data (especially public keys) from
    // the previous occupant leaking into the new entry.
    const bool isExisting = _neighbors[slot].occupied &&
        (_neighbors[slot].pos.fromNode == fromNode ||
         _neighbors[slot].user.fromNode == fromNode);
    if (!isExisting)
        _neighbors[slot] = {};   // zero entire entry

    _neighbors[slot].occupied = true;
    if (pos)
    {
        _neighbors[slot].pos           = *pos;
        _neighbors[slot].pos.fromNode  = fromNode;
        _neighbors[slot].pos.lastSeen  = xTaskGetTickCount();
        _neighbors[slot].pos.valid     = true;
    }
    else
    {
        _neighbors[slot].pos.fromNode = fromNode;
    }
    if (user)
    {
        _neighbors[slot].user          = *user;
        _neighbors[slot].user.fromNode = fromNode;
        _neighbors[slot].user.valid    = true;

        portEXIT_CRITICAL(&_neighborLock);
    }
    else
    {
        _neighbors[slot].user.fromNode = fromNode;
        portEXIT_CRITICAL(&_neighborLock);
    }
}

// ── neighborCount ─────────────────────────────────────────────────────────
size_t LoRa::neighborCount() const
{
    portENTER_CRITICAL(&_neighborLock);
    size_t n = 0;
    for (size_t i = 0; i < NEIGHBOR_MAX; i++)
        if (_neighbors[i].occupied) n++;
    portEXIT_CRITICAL(&_neighborLock);
    return n;
}

// ── neighborPosition ──────────────────────────────────────────────────────
MeshPosition LoRa::neighborPosition(size_t idx) const
{
    portENTER_CRITICAL(&_neighborLock);
    // Return the idx-th occupied slot
    size_t count = 0;
    MeshPosition result = {};
    for (size_t i = 0; i < NEIGHBOR_MAX; i++)
    {
        if (_neighbors[i].occupied)
        {
            if (count == idx) { result = _neighbors[i].pos; break; }
            count++;
        }
    }
    portEXIT_CRITICAL(&_neighborLock);
    return result;
}

// ── neighborUser ──────────────────────────────────────────────────────────
MeshUser LoRa::neighborUser(size_t idx) const
{
    portENTER_CRITICAL(&_neighborLock);
    size_t count = 0;
    MeshUser result = {};
    for (size_t i = 0; i < NEIGHBOR_MAX; i++)
    {
        if (_neighbors[i].occupied)
        {
            if (count == idx) { result = _neighbors[i].user; break; }
            count++;
        }
    }
    portEXIT_CRITICAL(&_neighborLock);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────
// ── _processPacket ────────────────────────────────────────────────────────
// Parse the raw LoRa payload, decode the Meshtastic Data protobuf
// (plaintext — IsLicensed mode), and dispatch by portnum.
void LoRa::_processPacket(const uint8_t* buf, uint8_t pktLen,
                           int16_t rssi, float snr)
{
    if (pktLen < MESH_HDR + 1)
    {
        ESP_LOGD(TAG, "Packet too short (%u B), ignoring", pktLen);
        return;
    }

    // Parse OTA header (all fields little-endian)
    const uint32_t to      = buf[0]  | (uint32_t)buf[1] <<  8
                           | (uint32_t)buf[2] << 16 | (uint32_t)buf[3] << 24;
    const uint32_t from    = buf[4]  | (uint32_t)buf[5] <<  8
                           | (uint32_t)buf[6] << 16 | (uint32_t)buf[7] << 24;
    const uint32_t pktId   = buf[8]  | (uint32_t)buf[9] <<  8
                           | (uint32_t)buf[10] << 16 | (uint32_t)buf[11] << 24;
    const uint8_t  hopLim  = buf[12] & 0x07;
    const uint8_t  chanHash= buf[13];

    // ── Phase 6: deduplication ─────────────────────────────────────────────
    // Skip packets we have already processed (same packet ID), which can
    // arrive via multiple mesh relay nodes within the same listen window.
    for (size_t i = 0; i < SEEN_IDS_MAX; i++)
    {
        if (_seenIds[i] == pktId && pktId != 0)
        {
            ESP_LOGD(TAG, "Duplicate pkt 0x%08" PRIx32 " — skipped", pktId);
            return;
        }
    }
    // Record this ID in the circular ring
    _seenIds[_seenCursor] = pktId;
    _seenCursor = (_seenCursor + 1) % SEEN_IDS_MAX;

    ESP_LOGI(TAG,
        "Rx pkt: to=0x%08" PRIx32 " from=0x%08" PRIx32 " id=0x%08" PRIx32
        " hop=%u ch=0x%02x len=%u rssi=%d snr=%.1f",
        to, from, pktId, hopLim, chanHash, pktLen, rssi, (double)snr);

    // ── Filter out our own transmissions ─────────────────────────────────
    // With separate TX/RX buffer base addresses, the SX1262 can still
    // receive our own packet if it reflects off nearby metal or the antenna
    // has poor return loss.  Drop early to avoid counting ourselves as a
    // mesh neighbour.
    if (from == Node.nodeId())
    {
        ESP_LOGI(TAG, "Ignoring own echo (hop=%u rssi=%d) — nearby node rebroadcast",
                 hopLim, rssi);
        return;
    }

    // ── Decrypt payload with AES-128-CTR (channel PSK) ───────────────────
    // AES-128-CTR is used by all standard Meshtastic nodes on ch=0x08.
    // Decrypt and re-encrypt are the same operation (CTR mode is symmetric),
    // so this function is shared for both RX and TX.
    const uint8_t* ciphertext = buf + MESH_HDR;
    const size_t   cipherLen  = pktLen - MESH_HDR;

    uint8_t plain[256] = {};
    if (!_decrypt(ciphertext, cipherLen, pktId, from, plain))
    {
        ESP_LOGW(TAG, "AES-CTR failed for pkt 0x%08" PRIx32, pktId);
        return;
    }

    portENTER_CRITICAL(&_statsLock);
    _stats.decryptOk++;
    portEXIT_CRITICAL(&_statsLock);

    // Hex dump of decrypted Data proto for wire-level debugging.
    {
        char hex[120 * 3 + 1] = {};
        for (size_t i = 0; i < cipherLen && i < 120; i++)
            snprintf(hex + i * 3, 4, "%02x ", plain[i]);
        ESP_LOGI(TAG, "RX Data proto (%u bytes): %s", (unsigned)cipherLen, hex);
    }

    const size_t plainLen = cipherLen;

    // Decode the Data protobuf wrapper
    uint32_t       portnum    = 0;
    const uint8_t* payload    = nullptr;
    size_t         payloadLen = 0;
    bool           wantResp   = false;

    if (!_parseData(plain, plainLen, portnum, payload, payloadLen, wantResp))
    {
        ESP_LOGI(TAG, "Parse failed for pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32 " ch=0x%02x",
                 pktId, from, chanHash);
        return;
    }
    // An empty payload is valid for TRACEROUTE_APP — an empty RouteDiscovery proto
    // means a direct traceroute request with no intermediate hops yet.
    // All other portnums require a payload.
    if ((payload == nullptr || payloadLen == 0) && portnum != PORT_TRACEROUTE)
    {
        ESP_LOGI(TAG, "No payload in Data proto for pkt 0x%08" PRIx32
                 " from 0x%08" PRIx32 " portnum=%u",
                 pktId, from, portnum);
        return;
    }

    ESP_LOGI(TAG, "Decoded pkt from 0x%08" PRIx32 " portnum=%u payloadLen=%u want_resp=%d",
             from, portnum, (unsigned)payloadLen, (int)wantResp);

    // ── Dispatch by portnum ───────────────────────────────────────────────

    if (portnum == PORT_TEXT)
    {
        // Only surface text messages that are direct DMs to us or channel
        // broadcasts (group messages). Drop messages addressed to other nodes
        // that we overheard as mesh relays — they are not meant for us.
        const bool isDirect   = (to == Node.nodeId());
        const bool isBroadcast = (to == 0xFFFFFFFFu);
        if (!isDirect && !isBroadcast)
        {
            ESP_LOGI(TAG, "Text from 0x%08" PRIx32 " to 0x%08" PRIx32
                     " — not addressed to us, suppressing notification",
                     from, to);
            return;
        }

        MeshMessage msg;
        msg.fromNode = from;
        msg.rssi     = rssi;
        msg.snr      = snr;
        msg.valid    = true;
        const size_t copyLen = std::min(payloadLen, sizeof(msg.text) - 1);
        memcpy(msg.text, payload, copyLen);
        msg.text[copyLen] = '\0';

        ESP_LOGI(TAG, "Text from 0x%08" PRIx32 " (rssi=%d snr=%.1f) [%s]: %s",
                 from, rssi, (double)snr, isDirect ? "DM" : "CH", msg.text);

        portENTER_CRITICAL(&_statsLock);
        _stats.textMessages++;
        _stats.lastRssi = rssi;
        _stats.lastSnr  = snr;
        portEXIT_CRITICAL(&_statsLock);

        portENTER_CRITICAL(&_msgLock);
        _lastMsg = msg;
        portEXIT_CRITICAL(&_msgLock);

        Heltec.notifyDraw(Hardware::DRAW_LORA);
    }
    else if (portnum == PORT_POSITION)
    {
        MeshPosition pos = {};
        pos.rssi = rssi;
        pos.snr  = snr;

        if (_parsePosition(payload, payloadLen, pos))
        {
            ESP_LOGI(TAG,
                "Position from 0x%08" PRIx32
                " lat=%.5f lon=%.5f alt=%dm sats=%" PRIu32
                " rssi=%d snr=%.1f",
                from,
                (double)pos.lat_i / 1e7,
                (double)pos.lon_i / 1e7,
                (int)pos.alt_m, pos.sats,
                rssi, (double)snr);

            portENTER_CRITICAL(&_statsLock);
            _stats.lastRssi = rssi;
            _stats.lastSnr  = snr;
            portEXIT_CRITICAL(&_statsLock);

            _upsertNeighbor(from, &pos, nullptr);
            Heltec.notifyDraw(Hardware::DRAW_LORA_POS);
        }
        else
        {
            ESP_LOGD(TAG, "Position parse failed for pkt 0x08" PRIx32, pktId);
        }
    }
    else if (portnum == PORT_NODEINFO)
    {
        MeshUser user = {};
        user.rssi = rssi;
        user.snr  = snr;

        if (_parseUser(payload, payloadLen, user))
        {
            ESP_LOGI(TAG,
                "NodeInfo from 0x%08" PRIx32
                " id=%s long=\"%s\" short=\"%s\" hw=%" PRIu32
                " rssi=%d snr=%.1f want_resp=%d pubkey=%s",
                from,
                user.id, user.longName, user.shortName, user.hwModel,
                rssi, (double)snr, (int)wantResp,
                user.hasPublicKey ? "yes" : "no");

            portENTER_CRITICAL(&_statsLock);
            _stats.lastRssi = rssi;
            _stats.lastSnr  = snr;
            portEXIT_CRITICAL(&_statsLock);

            _upsertNeighbor(from, nullptr, &user);
            Heltec.notifyDraw(Hardware::DRAW_LORA_NODE);

            // ── Respond to want_response NodeInfo requests ────────────────
            // Only respond if the packet was addressed to us or broadcast.
#if CONFIG_LORA_TX_ENABLED
            if (wantResp && (to == Node.nodeId() || to == 0xFFFFFFFF))
            {
                ESP_LOGI(TAG, "Responding to NodeInfo request from 0x%08" PRIx32
                         " (request_id=0x%08" PRIx32 ")", from, pktId);
                sendNodeInfo(from, /*wantResponse=*/false, /*requestId=*/pktId);
            }
#endif
        }
        else
        {
            ESP_LOGD(TAG, "NodeInfo parse failed for pkt 0x08" PRIx32, pktId);
        }
    }
    else if (portnum == PORT_TRACEROUTE && to == Node.nodeId())
    {
        // ── Traceroute destination response ───────────────────────────────
        //
        // Meshtastic TRACEROUTE_APP (portnum 70) protocol (firmware 2.3+):
        //
        // Forward phase: the initiating app sends a RouteDiscovery proto with
        // an empty `route` field to the destination node.  Each intermediate
        // router appends its own node ID (and the received SNR) to `route` /
        // `snr_towards` before relaying.
        //
        // Destination phase (this code): we are the final `to` target.
        // We append our node ID to `route` and our received SNR to
        // `snr_towards`, then unicast the RouteDiscovery back to `from`
        // with request_id = incoming pktId so the app can correlate it.
        //
        // Return phase (handled by the mesh): intermediate routers append to
        // `route_back` / `snr_back` on the journey back to the initiator.
        //
        // RouteDiscovery proto wire format (meshtastic/mesh.proto):
        //   Field 1 (route,        repeated fixed32):  tag 0x0D per element
        //   Field 2 (snr_towards,  repeated sint32):   tag 0x10 per element (zigzag)
        //   Field 3 (route_back,   repeated fixed32):  tag 0x1D per element
        //   Field 4 (snr_back,     repeated sint32):   tag 0x20 per element (zigzag)

        static constexpr size_t TRACEROUTE_MAX = 8; // max hop count

        // ── Parse incoming RouteDiscovery ─────────────────────────────────
        uint32_t route[TRACEROUTE_MAX] = {};
        int32_t  snrTowards[TRACEROUTE_MAX] = {};   // already zigzag-encoded uint32
        size_t   routeLen = 0;
        size_t   snrLen   = 0;

        // payload may be nullptr/empty when the initiator sent a direct traceroute
        // with no intermediate hops — the RouteDiscovery proto is empty in that case.
        if (payload != nullptr && payloadLen > 0)
        {
            const uint8_t* rp  = payload;
            const uint8_t* end = payload + payloadLen;
            while (rp < end)
            {
                // Read tag byte (all RouteDiscovery field numbers < 16 → single byte)
                uint8_t tagByte = *rp++;
                uint8_t field   = tagByte >> 3;
                uint8_t wt      = tagByte & 0x07;

                if (field == 1 && wt == 5)          // route: fixed32
                {
                    if (rp + 4 > end) break;
                    uint32_t id = (uint32_t)rp[0]
                                | (uint32_t)rp[1] <<  8
                                | (uint32_t)rp[2] << 16
                                | (uint32_t)rp[3] << 24;
                    if (routeLen < TRACEROUTE_MAX) route[routeLen++] = id;
                    rp += 4;
                }
                else if (field == 2 && wt == 0)     // snr_towards: zigzag sint32
                {
                    uint32_t zz = 0; int sh = 0;
                    while (rp < end) {
                        uint8_t b = *rp++;
                        zz |= (uint32_t)(b & 0x7F) << sh;
                        if (!(b & 0x80)) break;
                        sh += 7;
                    }
                    if (snrLen < TRACEROUTE_MAX) snrTowards[snrLen++] = (int32_t)zz; // store raw zigzag
                    (void)zz;
                }
                else if (wt == 5) { if (rp + 4 <= end) rp += 4; else break; }
                else if (wt == 0) { while (rp < end && (*rp & 0x80)) rp++; if (rp < end) rp++; }
                else if (wt == 1) { if (rp + 8 <= end) rp += 8; else break; }
                else if (wt == 2)
                {
                    uint32_t l = 0; int sh = 0;
                    while (rp < end) {
                        uint8_t b = *rp++;
                        l |= (uint32_t)(b & 0x7F) << sh;
                        if (!(b & 0x80)) break;
                        sh += 7;
                    }
                    if (rp + l <= end) rp += l; else break;
                }
                else break;
            }
        }

        // ── Append our own node ID and received SNR ───────────────────────
        if (routeLen < TRACEROUTE_MAX)
            route[routeLen++] = Node.nodeId();

        // Convert received SNR (float dB) to Meshtastic's 0.25 dB integer units,
        // then zigzag-encode it for the sint32 wire format.
        const int32_t ourSnrRaw  = static_cast<int32_t>(snr * 4.0f);
        const uint32_t ourSnrZz  = _pbZigzag(ourSnrRaw);
        if (snrLen < TRACEROUTE_MAX)
            snrTowards[snrLen++] = static_cast<int32_t>(ourSnrZz); // store as zigzag uint

        // ── Log ───────────────────────────────────────────────────────────
        {
            char routeStr[12 * TRACEROUTE_MAX + 1] = {};
            size_t rs = 0;
            for (size_t i = 0; i < routeLen; i++)
                rs += snprintf(routeStr + rs, sizeof(routeStr) - rs,
                               "!%08" PRIx32 " ", route[i]);
            ESP_LOGI(TAG, "TRACEROUTE from 0x%08" PRIx32 " → route: %s(rssi=%d snr=%.1f)",
                     from, routeStr, rssi, (double)snr);
        }

        // ── Encode response RouteDiscovery ────────────────────────────────
        // Field 1 (route): 5 bytes per element (tag + 4-byte fixed32)
        // Field 2 (snr_towards): 1 tag + up to 5 varint bytes per element
        uint8_t rdBuf[5 * TRACEROUTE_MAX + 6 * TRACEROUTE_MAX + 4] = {};
        size_t  rdLen = 0;

        // Field 1: route (repeated fixed32)
        for (size_t i = 0; i < routeLen; i++)
        {
            rdBuf[rdLen++] = 0x0D; // tag: field 1, wire type 5 (fixed32)
            rdBuf[rdLen++] = static_cast<uint8_t>(route[i]);
            rdBuf[rdLen++] = static_cast<uint8_t>(route[i] >>  8);
            rdBuf[rdLen++] = static_cast<uint8_t>(route[i] >> 16);
            rdBuf[rdLen++] = static_cast<uint8_t>(route[i] >> 24);
        }

        // Field 2: snr_towards (repeated sint32, zigzag varint)
        for (size_t i = 0; i < snrLen; i++)
        {
            rdBuf[rdLen++] = 0x10; // tag: field 2, wire type 0 (varint)
            rdLen += _pbVarint(rdBuf + rdLen,
                               static_cast<uint32_t>(snrTowards[i]));
        }

        // ── Transmit response ─────────────────────────────────────────────
#if CONFIG_LORA_TX_ENABLED
        {
            uint8_t pkt[256] = {};
            uint8_t pktLen2  = 0;
            if (_buildTxPacket(pkt, pktLen2, PORT_TRACEROUTE,
                               rdBuf, rdLen,
                               /*want_response=*/false,
                               /*to=*/from,
                               /*requestId=*/pktId))
            {
                ESP_LOGI(TAG, "TX TRACEROUTE reply to 0x%08" PRIx32
                         " req_id=0x%08" PRIx32 " route_hops=%u",
                         from, pktId, (unsigned)routeLen);
                transmit(pkt, pktLen2);
            }
            else
            {
                ESP_LOGW(TAG, "TRACEROUTE: _buildTxPacket failed");
            }
        }
#endif
    }
    else
    {
        ESP_LOGD(TAG, "Unhandled portnum %" PRIu32 " from 0x%08" PRIx32,
                 portnum, from);
    }
}

// ── lastMessage ───────────────────────────────────────────────────────────
MeshMessage LoRa::lastMessage() const
{
    portENTER_CRITICAL(&_msgLock);
    const MeshMessage copy = _lastMsg;
    portEXIT_CRITICAL(&_msgLock);
    return copy;
}

// ── stats ─────────────────────────────────────────────────────────────────
LoRaStats LoRa::stats() const
{
    portENTER_CRITICAL(&_statsLock);
    const LoRaStats copy = _stats;
    portEXIT_CRITICAL(&_statsLock);
    return copy;
}

// ── run ───────────────────────────────────────────────────────────────────
void LoRa::run(void* /*data*/)
{
#if !CONFIG_LORA_ENABLED
    ESP_LOGI(TAG, "LoRa disabled by config — task exiting");
    return;
#endif

    _taskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "LoRa task started");

    // ── SPI bus init ──────────────────────────────────────────────────────
    spi_bus_config_t bus = {};
    bus.mosi_io_num     = PIN_MOSI;
    bus.miso_io_num     = PIN_MISO;
    bus.sclk_io_num     = PIN_SCK;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = 256;

    esp_err_t err = spi_bus_initialize(SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return;
    }

    spi_device_interface_config_t dev = {};
    dev.mode           = 0;         // CPOL=0, CPHA=0
    dev.clock_speed_hz = SPI_FREQ_HZ;
    dev.spics_io_num   = PIN_NSS;
    dev.queue_size     = 1;

    err = spi_bus_add_device(SPI_HOST, &dev, &_spi);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        return;
    }

    // ── GPIO: RESET (output), BUSY (input), DIO1 (input + interrupt) ─────
    gpio_config_t rst_conf = {};
    rst_conf.pin_bit_mask = 1ULL << PIN_RESET;
    rst_conf.mode         = GPIO_MODE_OUTPUT;
    rst_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    rst_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rst_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&rst_conf);
    gpio_set_level(PIN_RESET, 1); // idle high

    gpio_config_t busy_conf = {};
    busy_conf.pin_bit_mask = (1ULL << PIN_BUSY) | (1ULL << PIN_DIO1);
    busy_conf.mode         = GPIO_MODE_INPUT;
    busy_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    busy_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    busy_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&busy_conf);

    // ── SX1262 init ───────────────────────────────────────────────────────
    if (!_initSx1262())
    {
        ESP_LOGE(TAG, "SX1262 init failed — LoRa task exiting");
        portENTER_CRITICAL(&_statsLock);
        _stats.state = LoRaStats::State::InitFailed;
        portEXIT_CRITICAL(&_statsLock);
        Heltec.showLoraState(false);
        return;
    }

    // ── Install DIO1 rising-edge ISR (after task handle is set) ──────────
    gpio_config_t dio1_conf = {};
    dio1_conf.pin_bit_mask = 1ULL << PIN_DIO1;
    dio1_conf.mode         = GPIO_MODE_INPUT;
    dio1_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    dio1_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dio1_conf.intr_type    = GPIO_INTR_POSEDGE;
    gpio_config(&dio1_conf);

    gpio_install_isr_service(0);   // safe to call multiple times
    gpio_isr_handler_add(PIN_DIO1, _dio1Isr, this);

    // ── Enter RX mode ─────────────────────────────────────────────────────
    _setRx();

    portENTER_CRITICAL(&_statsLock);
    _stats.state = LoRaStats::State::Listening;
    portEXIT_CRITICAL(&_statsLock);
    Heltec.showLoraState(true);

    ESP_LOGI(TAG, "Listening on %u Hz (slot=%u, chip mode=0x%02x)",
             (unsigned)LORA_FREQ_HZ,
             (unsigned)((LORA_FREQ_HZ - 902125000) / 250000),
             _getChipMode());

    TickType_t lastDiagTick = xTaskGetTickCount();

#if CONFIG_LORA_TX_ENABLED
    // Position broadcast interval — computed once from Kconfig.
    // Initialise _lastPosTxTick to (now - interval) so the first broadcast
    // fires as soon as a fix is available, not after one full interval.
    const TickType_t posTxInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_POSITION_TX_INTERVAL_SEC * 1000UL);
    _lastPosTxTick = xTaskGetTickCount() - posTxInterval;

    // NodeInfo interval
    const TickType_t nodeInfoInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_NODEINFO_TX_INTERVAL_SEC * 1000UL);
    // Boot broadcast — send immediately, then 2 more retransmits at 15 s
    // intervals before settling to the configured 1800 s period.
    // This gives nearby nodes 3 chances to receive our identity packet in
    // the first 30 s, covering any brief listen gaps or channel congestion.
    sendNodeInfo();
    _nodeInfoBootCount = 1;
    _lastNodeInfoTxTick = xTaskGetTickCount()
                        - nodeInfoInterval
                        + pdMS_TO_TICKS(15 * 1000UL); // next in 15 s

    // Telemetry interval — first TX fires 15 s after boot (after NodeInfo settles)
    const TickType_t telemetryInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_TELEMETRY_TX_INTERVAL_SEC * 1000UL);
    _lastTelemetryTxTick = xTaskGetTickCount()
                         - telemetryInterval
                         + pdMS_TO_TICKS(15 * 1000UL);

    // MAP_REPORT interval — first TX fires 30 s after boot (after position settles)
    // Disabled at compile time when CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC == 0.
    const TickType_t mapReportInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC * 1000UL);
    _lastMapReportTxTick = xTaskGetTickCount()
                         - mapReportInterval
                         + pdMS_TO_TICKS(30 * 1000UL);
#endif

    // ── Receive loop ──────────────────────────────────────────────────────
    while (true)
    {
        // Wait for DIO1 IRQ (notified by ISR) or wake on timeout to check state.
        // Timeout of 30 s ensures we re-enter RX even if DIO1 was missed.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30000));

        const uint16_t irq = _getIrqStatus();
        _clearIrq(0xFFFF);

        // Count preamble/header-valid events from the global IRQ mask
        // (these don't fire DIO1, but are visible in GetIrqStatus)
        if (irq & IRQ_PREAMBLE_DET)
        {
            portENTER_CRITICAL(&_statsLock);
            _stats.preambles++;
            portEXIT_CRITICAL(&_statsLock);
        }
        if (irq & IRQ_HEADER_VALID)
        {
            portENTER_CRITICAL(&_statsLock);
            _stats.headersValid++;
            portEXIT_CRITICAL(&_statsLock);
        }

        if (irq & IRQ_RX_DONE)
        {
            portENTER_CRITICAL(&_statsLock);
            _stats.rxPackets++;
            portEXIT_CRITICAL(&_statsLock);

            // GetRxBufferStatus — always read so the buffer pointer is valid
            // for diagnostics even on CRC-error packets.
            uint8_t txs[4] = { CMD_GET_RX_BUF_STATUS, 0, 0, 0 };
            uint8_t rxs[4] = {};
            _transact(txs, rxs, sizeof(txs));
            const uint8_t payloadLen = rxs[2];
            const uint8_t rxPtr      = rxs[3];

            ESP_LOGI(TAG, "RX_DONE irq=0x%04x len=%u ptr=0x%02x",
                     irq, payloadLen, rxPtr);

            // ── CRC error check (must be inside RX_DONE, not a separate else-if)
            // The SX1262 sets BOTH RX_DONE and CRC_ERROR for packets that
            // complete reception but fail the LoRa CRC.  The old else-if
            // structure never reached the CRC_ERROR branch when RX_DONE was
            // set, so corrupted packets were silently fed to _processPacket.
            // AES-CTR has no integrity check, so decryption "succeeds" on
            // garbage, and _parseData/parseUser produce random results.
            if (irq & IRQ_CRC_ERROR)
            {
                portENTER_CRITICAL(&_statsLock);
                // Undo the rxPackets++ above — this was not a valid packet
                _stats.rxPackets--;
                _stats.crcErrors++;
                portEXIT_CRITICAL(&_statsLock);

                uint8_t txp[4] = { CMD_GET_PKT_STATUS, 0, 0, 0 };
                uint8_t rxp[4] = {};
                _transact(txp, rxp, sizeof(txp));
                const int16_t rssi = -(static_cast<int16_t>(rxp[2])) / 2;
                const float   snr  = static_cast<float>(static_cast<int8_t>(rxp[3])) / 4.0f;
                ESP_LOGI(TAG, "CRC_ERR (with RX_DONE) irq=0x%04x rssi=%d snr=%.1f — discarding",
                         irq, rssi, (double)snr);
            }
            else if (payloadLen > 0)
            {
                // GetPacketStatus:
                //   TX:  [0x14] [NOP]    [NOP]      [NOP]
                //   RX:  [0x00] [status] [RssiPkt]  [SnrPkt]
                uint8_t txp[4] = { CMD_GET_PKT_STATUS, 0, 0, 0 };
                uint8_t rxp[4] = {};
                _transact(txp, rxp, sizeof(txp));
                const int16_t rssi = -(static_cast<int16_t>(rxp[2])) / 2;
                const float   snr  = static_cast<float>(static_cast<int8_t>(rxp[3])) / 4.0f;

                // ReadBuffer → [cmd, startPtr, dummy_status, data[0], data[1], ...]
                const size_t txLen = 3 + payloadLen;
                uint8_t txb[3 + 255] = { CMD_READ_BUFFER, rxPtr, 0x00 };
                uint8_t rxb[3 + 255] = {};
                _waitBusy();
                spi_transaction_t t = {};
                t.length    = txLen * 8;
                t.tx_buffer = txb;
                t.rx_buffer = rxb;
                spi_device_polling_transmit(_spi, &t);

                _processPacket(rxb + 3, payloadLen, rssi, snr);
            }
        }
        else if (irq & IRQ_HEADER_ERR)
        {
            portENTER_CRITICAL(&_statsLock);
            _stats.headerErrors++;
            portEXIT_CRITICAL(&_statsLock);

            uint8_t txp[4] = { CMD_GET_PKT_STATUS, 0, 0, 0 };
            uint8_t rxp[4] = {};
            _transact(txp, rxp, sizeof(txp));
            const int16_t rssi = -(static_cast<int16_t>(rxp[2])) / 2;
            const float   snr  = static_cast<float>(static_cast<int8_t>(rxp[3])) / 4.0f;

            ESP_LOGI(TAG, "HDR_ERR irq=0x%04x rssi=%d snr=%.1f iq_cfg=0x%02x",
                     irq, rssi, (double)snr, _readReg(REG_IQ_CONFIG));
        }
        else if (irq & IRQ_CRC_ERROR)
        {
            // CRC_ERROR without RX_DONE — unusual but handle gracefully
            portENTER_CRITICAL(&_statsLock);
            _stats.crcErrors++;
            portEXIT_CRITICAL(&_statsLock);

            uint8_t txp[4] = { CMD_GET_PKT_STATUS, 0, 0, 0 };
            uint8_t rxp[4] = {};
            _transact(txp, rxp, sizeof(txp));
            const int16_t rssi = -(static_cast<int16_t>(rxp[2])) / 2;
            const float   snr  = static_cast<float>(static_cast<int8_t>(rxp[3])) / 4.0f;

            ESP_LOGI(TAG, "CRC_ERR (no RX_DONE) irq=0x%04x rssi=%d snr=%.1f",
                     irq, rssi, (double)snr);
        }
        else if (irq & IRQ_TIMEOUT)
        {
            ESP_LOGD(TAG, "LoRa RX timeout (re-entering RX)");
        }
        // else: spurious 30 s wakeup — just re-enter RX

        // ── 30-second diagnostic log ──────────────────────────────────────
        const TickType_t now = xTaskGetTickCount();
        if ((now - lastDiagTick) >= pdMS_TO_TICKS(30000))
        {
            lastDiagTick = now;
            const LoRaStats s        = stats();
            const int16_t   noiseFloor = _getInstRssi();
            const uint8_t   chipMode   = _getChipMode();
            const uint8_t   iqCfg      = _readReg(REG_IQ_CONFIG);
            ESP_LOGI(TAG,
                "DIAG: preamble=%" PRIu32 " hdr_ok=%" PRIu32
                " rx=%" PRIu32 "  crc_err=%" PRIu32
                "  hdr_err=%" PRIu32 "  decrypt_ok=%" PRIu32
                "  text=%" PRIu32 "  tx=%" PRIu32 "  tx_err=%" PRIu32
                "  tx_timeout=%" PRIu32 "  neighbors=%u"
                "  last_rssi=%d  last_snr=%.1f"
                "  noise=%d dBm  mode=0x%02x  iq=0x%02x",
                s.preambles, s.headersValid,
                s.rxPackets, s.crcErrors, s.headerErrors,
                s.decryptOk, s.textMessages,
                s.txPackets, s.txErrors, s.txTimeouts,
                (unsigned)neighborCount(),
                s.lastRssi, (double)s.lastSnr,
                (int)noiseFloor, chipMode, iqCfg);
            if (chipMode != 0x05)
            {
                ESP_LOGW(TAG, "Chip mode 0x%02x != 0x05 (RX) — re-entering RX", chipMode);
                _setRx();
            }
        }

        // Re-enter RX — required after every RX_DONE or error
        _setRx();

#if CONFIG_LORA_TX_ENABLED
        // ── Periodic + adaptive GPS position broadcast ────────────────────
        // Two triggers:
        //   1. Normal: configured interval elapsed (default 300 s).
        //   2. Adaptive: significant movement detected (>~50m) AND at least
        //      MIN_POS_TX_INTERVAL_SEC has elapsed since the last TX.
        //      This makes the device useful as a real-time tracker without
        //      flooding the channel during stationary periods.
        if (gps.isFixed())
        {
            const TickType_t nowTick   = xTaskGetTickCount();
            const TickType_t sincePos  = nowTick - _lastPosTxTick;
            const TickType_t minPosInterval =
                pdMS_TO_TICKS((uint32_t)MIN_POS_TX_INTERVAL_SEC * 1000UL);

            bool doTx = false;

            // Trigger 1: normal interval
            if (sincePos >= posTxInterval)
            {
                doTx = true;
            }
            // Trigger 2: movement-based early broadcast
            else if (sincePos >= minPosInterval)
            {
                const int32_t cur_lat_i = static_cast<int32_t>(gps.lat() * 1e7);
                const int32_t cur_lon_i = static_cast<int32_t>(gps.lng() * 1e7);
                if (_hasTxPos)
                {
                    // 0.0005° ≈ 55m at equator. At our latitude (~29°N) a degree
                    // of longitude ≈ 96km, so this is about 48m in longitude and
                    // 55m in latitude — a reasonable trigger for a moving tracker.
                    const int32_t dlat = cur_lat_i - _lastTxLat_i;
                    const int32_t dlon = cur_lon_i - _lastTxLon_i;
                    const int32_t MOVE_THRESH = 5000; // 0.0005° × 1e7 = 5000 units
                    if (dlat > MOVE_THRESH || dlat < -MOVE_THRESH ||
                        dlon > MOVE_THRESH || dlon < -MOVE_THRESH)
                    {
                        ESP_LOGI(TAG, "Position TX: movement detected (Δlat=%d Δlon=%d), early send",
                                 (int)dlat, (int)dlon);
                        doTx = true;
                    }
                }
                else
                {
                    // First fix — send immediately
                    doTx = true;
                }
            }

            if (doTx)
            {
                const double   txLat = gps.lat();
                const double   txLng = gps.lng();
                const uint32_t sats  = gps.satellites();
                if (sendPosition(txLat, txLng, gps.altitude(), /*pdop_x100=*/0, sats))
                {
                    _lastPosTxTick = xTaskGetTickCount();
                    _lastTxLat_i   = static_cast<int32_t>(txLat * 1e7);
                    _lastTxLon_i   = static_cast<int32_t>(txLng * 1e7);
                    _hasTxPos      = true;
                }
            }
        }
        else if ((xTaskGetTickCount() - _lastPosTxTick) >= posTxInterval)
        {
            // Interval elapsed but no fix — reset timer so we don't spam
            // the log or immediately fire when fix is later acquired after
            // a very long no-fix period.
            _lastPosTxTick = xTaskGetTickCount();
            ESP_LOGD(TAG, "Position TX skipped — no GPS fix");
        }

        // ── Periodic NodeInfo broadcast ───────────────────────────────────
        if ((xTaskGetTickCount() - _lastNodeInfoTxTick) >= nodeInfoInterval)
        {
            sendNodeInfo();
            _nodeInfoBootCount++;
            // First 3 broadcasts: 15 s retransmit window for reliable boot
            // discovery.  From the 4th onward: normal 1800 s interval.
            const TickType_t nextNI = (_nodeInfoBootCount < 3)
                ? pdMS_TO_TICKS(15 * 1000UL)
                : nodeInfoInterval;
            _lastNodeInfoTxTick = xTaskGetTickCount()
                                 - nodeInfoInterval + nextNI;
        }

        // ── Periodic Telemetry broadcast ──────────────────────────────────
        if ((xTaskGetTickCount() - _lastTelemetryTxTick) >= telemetryInterval)
        {
            _lastTelemetryTxTick = xTaskGetTickCount();
            sendTelemetry();
        }

        // ── Periodic MAP_REPORT broadcast (GPS_APP / GPS_TRACKER_APP) ─────
        // MAP_REPORT_APP (port 73) announces this node to MQTT bridges so it
        // appears on the public Meshtastic map (meshtastic.network/map).
        // Only sent when a GPS fix is available and the interval is enabled.
#if CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC > 0
        if ((xTaskGetTickCount() - _lastMapReportTxTick) >= mapReportInterval)
        {
            _lastMapReportTxTick = xTaskGetTickCount();
            sendMapReport(); // silently skips when no GPS fix
        }
#endif
#endif
    }
}
LoRa Lora("LoRa", 10240);
