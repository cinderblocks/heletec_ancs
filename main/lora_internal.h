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
 * lora_internal.h — private constants shared between the three LoRa
 * implementation translation units:
 *
 *   sx1262.cxx          SX1262 SPI hardware driver
 *   meshtastic_proto.cxx Meshtastic protobuf codec, AES crypto, packet dispatch
 *   lora.cxx             FreeRTOS task entry (run loop + extern Lora)
 *
 * Do NOT include this header from any file outside main/.
 */

#pragma once

#include <cstdint>

// ── Kconfig fallback defaults ─────────────────────────────────────────────
// Guard every symbol that has been added incrementally so each TU compiles
// correctly even when sdkconfig hasn't been regenerated yet.
#ifndef CONFIG_LORA_TX_ENABLED
#  define CONFIG_LORA_TX_ENABLED 1
#endif
#ifndef CONFIG_LORA_IS_LICENSED
#  define CONFIG_LORA_IS_LICENSED 0
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
// Meshtastic default LongFast slot (US): slot = 20 (factory default as of
// Meshtastic firmware 2.5 — older versions used slot 103 derived from hash).
// Set CONFIG_LORA_FREQUENCY_HZ to a non-zero value to override completely.
static inline uint32_t computeLoraFrequency()
{
    if (CONFIG_LORA_FREQUENCY_HZ != 0)
        return static_cast<uint32_t>(CONFIG_LORA_FREQUENCY_HZ);

    static constexpr uint32_t FREQ_START   = 902000000;
    static constexpr uint32_t BW_HZ        = 250000;
    static constexpr uint32_t NUM_CHANNELS = 104;

    uint32_t slot = (CONFIG_LORA_CHANNEL_NUM > 0)
                  ? static_cast<uint32_t>(CONFIG_LORA_CHANNEL_NUM) - 1
                  : 20u; // Meshtastic US LongFast factory default

    if (slot >= NUM_CHANNELS) slot = 0;
    return FREQ_START + BW_HZ / 2 + slot * BW_HZ;
}

// Cached at file scope — computed once per TU.  All TUs get the same value.
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
static constexpr uint16_t IRQ_RX_DIO1   = IRQ_RX_DONE | IRQ_HEADER_ERR |
                                           IRQ_CRC_ERROR | IRQ_TIMEOUT;
// Global mask — events visible in GetIrqStatus (superset of DIO1).
static constexpr uint16_t IRQ_RX_GLOBAL = IRQ_RX_DIO1 | IRQ_PREAMBLE_DET |
                                           IRQ_HEADER_VALID;
static constexpr uint16_t IRQ_TX_MASK   = IRQ_TX_DONE | IRQ_TIMEOUT;

// ── Meshtastic OTA protocol constants ────────────────────────────────────

// DEFAULT_CHAN_HASH: 1-byte channel hash placed in OTA header byte [13].
// Default "LongFast" channel:
//   xorHash("LongFast", 8) = 0x0A
//   xorHash(expanded_default_PSK, 16) = 0x02
//   hash = 0x0A ^ 0x02 = 0x08
static constexpr uint8_t DEFAULT_CHAN_HASH = 0x08;

// HW_MODEL: Meshtastic HardwareModel enum value for this board.
// meshtastic_HardwareModel_HELTEC_WIRELESS_TRACKER = 48 (mesh.proto)
static constexpr uint32_t HW_MODEL = 48;

// Meshtastic OTA flags byte (RadioLibInterface.cpp / mesh_pb.h):
//   bits [2:0]  hop_limit  — remaining relay hops
//   bit  [3]    want_ack   — request implicit ACK from next hop
//   bit  [4]    via_mqtt   — 0 for OTA-originated packets
//   bits [7:5]  hop_start  — ORIGINAL hop_limit (must equal hop_limit for direct)
//
// Correct values (hop_start = hop_limit = 3, via_mqtt = 0):
//   Broadcast (want_ack=0): 3 | 0 | 0 | (3<<5) = 99  = 0x63
//   Unicast   (want_ack=1): 3 | 8 | 0 | (3<<5) = 107 = 0x6B
static constexpr uint8_t FLAGS_BROADCAST = 0x63;
static constexpr uint8_t FLAGS_UNICAST   = 0x6B;
