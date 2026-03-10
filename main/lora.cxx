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
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mbedtls/aes.h>
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

// ── Meshtastic default channel PSK ────────────────────────────────────────
// AES-128 key for the "Default" / LongFast channel used by every factory
// Meshtastic device.  Sourced from Meshtastic firmware CryptoEngine.cpp.
/* static */ const uint8_t LoRa::DEFAULT_PSK[16] = {
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};

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

// ── SX1262 register addresses ─────────────────────────────────────────────
static constexpr uint16_t REG_LORA_SYNC_MSB = 0x0740; // LoRa sync word byte 0
static constexpr uint16_t REG_LORA_SYNC_LSB = 0x0741; // LoRa sync word byte 1

// ── IRQ bit masks ─────────────────────────────────────────────────────────
static constexpr uint16_t IRQ_TX_DONE      = (1u << 0);
static constexpr uint16_t IRQ_RX_DONE      = (1u << 1);
static constexpr uint16_t IRQ_HEADER_ERR   = (1u << 5);
static constexpr uint16_t IRQ_CRC_ERROR    = (1u << 6);
static constexpr uint16_t IRQ_TIMEOUT      = (1u << 9);
static constexpr uint16_t IRQ_RX_MASK      = IRQ_RX_DONE | IRQ_HEADER_ERR |
                                              IRQ_CRC_ERROR | IRQ_TIMEOUT;
static constexpr uint16_t IRQ_TX_MASK      = IRQ_TX_DONE | IRQ_TIMEOUT;

// ── Meshtastic OTA TX constants ───────────────────────────────────────────
//
// DEFAULT_CHAN_HASH: 1-byte channel hash placed in the OTA header byte [13].
// Computed as: (sum of all DEFAULT_PSK bytes) & 0xFF.
// The default channel has an empty name (""), so only the PSK is summed:
//   0xD4+0xF1+0xBB+0x3A+0x20+0x29+0x07+0x59+
//   0xF0+0xBC+0xFF+0xAB+0xCF+0x4E+0x69+0x01 = 0x840 → 0x40
// Receivers use this as a quick filter; a mismatch causes them to skip the
// packet without attempting decryption.  Must match real Meshtastic devices.
static constexpr uint8_t DEFAULT_CHAN_HASH = 0x40;

// HW_MODEL: Meshtastic HardwareModel enum value for this board.
// meshtastic_HardwareModel_HELTEC_WIRELESS_TRACKER = 87 (mesh.proto 2.5+)
static constexpr uint32_t HW_MODEL = 87;

// FLAGS_BROADCAST: OTA header flags byte for a normal originating broadcast.
// Bit layout: [hop_start(5:7)] [via_mqtt(4)] [want_ack(3)] [hop_limit(0:2)]
// hop_limit=3, hop_start=3, want_ack=0, via_mqtt=0 → 0b01100011 = 0x63
static constexpr uint8_t FLAGS_BROADCAST = 0x63;

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
// bw  = SX1262 BW code: 7=125kHz, 8=250kHz, 9=500kHz
// cr  = 1=4/5, 2=4/6, 3=4/7, 4=4/8
// ldro = low data rate optimise (1 if symbol time > 16 ms)
void LoRa::_setModulation(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    uint8_t tx[5] = { CMD_SET_MOD_PARAMS, sf, bw, cr, ldro };
    _transact(tx, nullptr, sizeof(tx));
}

// ── _setRx ────────────────────────────────────────────────────────────────
// Enter single continuous RX (no timeout; chip stays in RX until a packet
// arrives, then goes to STBY_RC and fires DIO1).
void LoRa::_setRx()
{
    // 0xFFFFFF = no timeout (continuous single RX mode)
    uint8_t tx[4] = { CMD_SET_RX, 0xFF, 0xFF, 0xFF };
    _transact(tx, nullptr, sizeof(tx));
}

// ── _getIrqStatus ─────────────────────────────────────────────────────────
// No-address get commands: TX=[cmd, NOP, NOP], RX=[status, irqH, irqL]
// Data is at rx[1:2], not rx[2:3].
uint16_t LoRa::_getIrqStatus()
{
    uint8_t tx[3] = { CMD_GET_IRQ_STATUS, 0, 0 };
    uint8_t rx[3] = {};
    _transact(tx, rx, sizeof(tx));
    return (static_cast<uint16_t>(rx[1]) << 8) | rx[2];
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
// Configure DIO1 IRQ mask for receive: RX_DONE | HEADER_ERR | CRC_ERROR | TIMEOUT.
void LoRa::_setRxIrq()
{
    const uint16_t mask = IRQ_RX_MASK;
    uint8_t t[] = { CMD_SET_DIO_IRQ,
                    static_cast<uint8_t>(mask >> 8), static_cast<uint8_t>(mask),  // global
                    static_cast<uint8_t>(mask >> 8), static_cast<uint8_t>(mask),  // DIO1
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

// ── transmit ──────────────────────────────────────────────────────────────
// Transmit a raw LoRa payload.  Sequences:
//   1. Standby(STBY_RC)
//   2. Update packet params with the actual payload length
//   3. WriteBuffer at TX_BASE (offset 0)
//   4. Switch DIO1 IRQ to TX_DONE | TIMEOUT
//   5. Clear any pending task notifications (prevent stale RX wakeup)
//   6. SetTx with 5-second timeout
//   7. Wait for DIO1 (TX_DONE or TIMEOUT)
//   8. Restore RX IRQ mask and re-enter RX
// Must only be called from the LoRa task.
bool LoRa::transmit(const uint8_t* data, uint8_t len)
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    if (len == 0 || data == nullptr) return false;

    ESP_LOGD(TAG, "TX: %u bytes", len);

    // 1. Enter standby so we can write the buffer and reconfigure
    { uint8_t t[] = { CMD_SET_STANDBY, 0x00 }; _transact(t, nullptr, sizeof(t)); }

    // 2. Update packet params with exact payload length (preamble & other
    //    params stay the same as _initSx1262 configured them)
    { uint8_t t[] = { CMD_SET_PKT_PARAMS,
                      0x00, 0x10, // preamble = 16
                      0x00,       // explicit header
                      len,        // actual payload length
                      0x01,       // CRC on
                      0x00 };     // IQ normal
      _transact(t, nullptr, sizeof(t)); }

    // 3. Write payload into SX1262 data buffer at offset 0 (TX_BASE)
    _writeBuffer(0x00, data, len);

    // 4. Switch DIO1 IRQ to TX mode
    _setTxIrq();

    // 5. Drain any stale task-notification from a prior RX DIO1 ISR
    ulTaskNotifyTake(pdTRUE, 0);

    // 6. Clear any lingering IRQ flags
    _clearIrq(0xFFFF);

    // 7. SetTx — timeout = 5 seconds = 5000 ms × 15.625 µs per tick
    //    SX1262 timeout unit = 15.625 µs; 5 s = 320000 ticks = 0x04E200
    { uint8_t t[] = { CMD_SET_TX,
                      0x04, 0xE2, 0x00 };
      _transact(t, nullptr, sizeof(t)); }

    // 8. Wait for DIO1 (TX_DONE or TIMEOUT) — block up to 6 s
    const uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(6000));

    // 9. Read and clear IRQ status
    const uint16_t irq = _getIrqStatus();
    _clearIrq(0xFFFF);

    // 10. Restore RX IRQ mask and packet params (max payload for RX)
    _setRxIrq();
    { uint8_t t[] = { CMD_SET_PKT_PARAMS,
                      0x00, 0x10, // preamble = 16
                      0x00,       // explicit header
                      0xFF,       // max payload (RX)
                      0x01,       // CRC on
                      0x00 };     // IQ normal
      _transact(t, nullptr, sizeof(t)); }

    // 11. Evaluate result
    bool ok = false;
    if (notified == 0)
    {
        // No DIO1 notification within 6 seconds — hard timeout
        ESP_LOGE(TAG, "TX timeout — no DIO1 notification in 6 s");
        portENTER_CRITICAL(&_statsLock);
        _stats.txTimeouts++;
        portEXIT_CRITICAL(&_statsLock);
    }
    else if (irq & IRQ_TX_DONE)
    {
        ESP_LOGI(TAG, "TX complete (%u bytes)", len);
        portENTER_CRITICAL(&_statsLock);
        _stats.txPackets++;
        portEXIT_CRITICAL(&_statsLock);
        ok = true;
    }
    else
    {
        ESP_LOGW(TAG, "TX failed — IRQ=0x%04x", irq);
        portENTER_CRITICAL(&_statsLock);
        _stats.txErrors++;
        portEXIT_CRITICAL(&_statsLock);
    }

    // 12. Re-enter RX
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
    _calibrateImage(CONFIG_LORA_FREQUENCY_HZ);
    _waitBusy(200);

    // ── DIO2 → RF switch control (drives TX/RX antenna switch) ───────────
    { uint8_t t[] = { CMD_SET_DIO2_RF_SWITCH, 0x01 }; _transact(t, nullptr, sizeof(t)); }

    // ── DC-DC regulator (SX1262 always has internal DCDC) ────────────────
    { uint8_t t[] = { CMD_SET_REGULATOR_MODE, 0x01 }; _transact(t, nullptr, sizeof(t)); }

    // ── LoRa packet type ──────────────────────────────────────────────────
    { uint8_t t[] = { CMD_SET_PACKET_TYPE, 0x01 }; _transact(t, nullptr, sizeof(t)); }

    // ── RF frequency ─────────────────────────────────────────────────────
    _setFrequency(CONFIG_LORA_FREQUENCY_HZ);

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
#if   defined(CONFIG_LORA_PRESET_LONG_FAST)
    _setModulation(11, 8, 1, 0); // SF11, 250 kHz, 4/5, no LDRO
#elif defined(CONFIG_LORA_PRESET_LONG_SLOW)
    _setModulation(12, 7, 4, 1); // SF12, 125 kHz, 4/8, LDRO on
#elif defined(CONFIG_LORA_PRESET_MEDIUM_SLOW)
    _setModulation(10, 8, 1, 0); // SF10, 250 kHz, 4/5, no LDRO
#elif defined(CONFIG_LORA_PRESET_SHORT_FAST)
    _setModulation( 7, 8, 1, 0); // SF7,  250 kHz, 4/5, no LDRO
#else
    _setModulation(11, 8, 1, 0); // default: LongFast
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

    // ── Buffer base addresses (TX=0, RX=128 — separate regions to prevent
    //    overlap during the TX→RX transition) ────────────────────────────────
    { uint8_t t[] = { CMD_SET_BUF_BASE_ADDR, 0x00, 0x80 };
      _transact(t, nullptr, sizeof(t)); }

    // ── LoRa sync word = Meshtastic private (0x12 in RadioLib = 0x1424 in regs)
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

    ESP_LOGI(TAG, "SX1262 ready — freq=%u Hz, sync=0x%02X%02X",
             (unsigned)CONFIG_LORA_FREQUENCY_HZ, SYNC_HI, SYNC_LO);
    return true;
}

// ── _decrypt ──────────────────────────────────────────────────────────────
// Meshtastic AES-128-CTR decryption.
// Nonce = [ packetId LE (4 bytes) | fromNode LE (4 bytes) | 0×8 ]
// Counter is the full 128-bit nonce block, incremented as big-endian uint128.
bool LoRa::_decrypt(const uint8_t* ciphertext, size_t len,
                    uint32_t packetId, uint32_t fromNode,
                    uint8_t* plaintext)
{
    if (len == 0) return false;

    uint8_t nonce[16] = {};
    // Construct nonce: packetId LE then fromNode LE
    nonce[0] = static_cast<uint8_t>(packetId);
    nonce[1] = static_cast<uint8_t>(packetId >>  8);
    nonce[2] = static_cast<uint8_t>(packetId >> 16);
    nonce[3] = static_cast<uint8_t>(packetId >> 24);
    nonce[4] = static_cast<uint8_t>(fromNode);
    nonce[5] = static_cast<uint8_t>(fromNode >>  8);
    nonce[6] = static_cast<uint8_t>(fromNode >> 16);
    nonce[7] = static_cast<uint8_t>(fromNode >> 24);
    // bytes [8:15] remain 0

    uint8_t streamBlock[16] = {};
    size_t  nc_off = 0;

    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);

    bool ok = false;
    if (mbedtls_aes_setkey_enc(&ctx, DEFAULT_PSK, 128) == 0)
    {
        ok = (mbedtls_aes_crypt_ctr(&ctx, len, &nc_off,
                                    nonce, streamBlock,
                                    ciphertext, plaintext) == 0);
    }

    mbedtls_aes_free(&ctx);
    return ok;
}

// ── _parseData ────────────────────────────────────────────────────────────
// Minimal protobuf decoder for the Meshtastic Data message.
//   Field 1 (portnum):  wire type 0 (varint)
//   Field 2 (payload):  wire type 2 (length-delimited)
// Returns true when portnum was found; payload/payloadLen may be nullptr/0
// if field 2 was absent.
bool LoRa::_parseData(const uint8_t* data, size_t len,
                      uint32_t& portnum,
                      const uint8_t*& payload, size_t& payloadLen)
{
    portnum    = 0;
    payload    = nullptr;
    payloadLen = 0;

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
            if (fieldNum == 1) portnum = static_cast<uint32_t>(val);
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
//   Field  1 (lat_i,    sint32): zigzag varint — degrees × 1e7
//   Field  2 (lon_i,    sint32): zigzag varint — degrees × 1e7
//   Field  3 (altitude,  int32): plain varint  — metres above MSL
//   Field  6 (PDOP,    uint32):  plain varint  — PDOP × 100
//   Field  7 (sats,    uint32):  plain varint  — satellites in view
//   Field  9 (time,    uint32):  plain varint  — UTC Unix timestamp (seconds)
//
// buf must be at least 80 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodePosition(uint8_t* buf, size_t /*cap*/,
                                           int32_t lat_i, int32_t lon_i,
                                           int32_t alt_m,
                                           uint32_t pdop_x100, uint32_t sats,
                                           uint32_t unixTime)
{
    size_t n = 0;

    // Field 1: latitude_i (sint32, zigzag) — always present
    buf[n++] = 0x08;
    n += _pbVarint(buf + n, _pbZigzag(lat_i));

    // Field 2: longitude_i (sint32, zigzag) — always present
    buf[n++] = 0x10;
    n += _pbVarint(buf + n, _pbZigzag(lon_i));

    // Field 3: altitude (int32, plain varint) — skip if zero
    if (alt_m != 0)
    {
        buf[n++] = 0x18;
        // Cast via int64 for correct sign extension of negative altitudes
        n += _pbVarint(buf + n, static_cast<uint64_t>(static_cast<int64_t>(alt_m)));
    }

    // Field 6: PDOP × 100 (uint32)
    if (pdop_x100 != 0)
    {
        buf[n++] = 0x30;
        n += _pbVarint(buf + n, pdop_x100);
    }

    // Field 7: sats_in_view (uint32)
    if (sats != 0)
    {
        buf[n++] = 0x38;
        n += _pbVarint(buf + n, sats);
    }

    // Field 9: timestamp (uint32 unix seconds)
    if (unixTime != 0)
    {
        buf[n++] = 0x48;
        n += _pbVarint(buf + n, unixTime);
    }

    return n;
}

// ── _encodeUser ───────────────────────────────────────────────────────────
// Encode a Meshtastic User proto (NODEINFO_APP payload).
//
//   Field 1 (id,         string): "!xxxxxxxx"  (node ID as hex string)
//   Field 2 (long_name,  string): up to 32 chars
//   Field 3 (short_name, string): up to 4 chars
//   Field 8 (hw_model,   uint32): HardwareModel enum value
//
// buf must be at least 80 bytes.  Returns bytes written.
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

    // Field 8: hw_model (varint)
    buf[n++] = 0x40;
    n += _pbVarint(buf + n, HW_MODEL);

    return n;
}

// ── _encodeData ───────────────────────────────────────────────────────────
// Wrap an encoded application payload in a Meshtastic Data proto.
//
//   Field 1 (portnum,  uint32): plain varint — PortNum enum value
//   Field 2 (payload,  bytes):  length-delimited raw bytes
//
// buf must be at least payloadLen + 16 bytes.  Returns bytes written.
/* static */ size_t LoRa::_encodeData(uint8_t* buf, size_t /*cap*/,
                                       uint32_t portnum,
                                       const uint8_t* payload, size_t payloadLen)
{
    size_t n = 0;

    // Field 1: portnum (varint)
    buf[n++] = 0x08;
    n += _pbVarint(buf + n, portnum);

    // Field 2: payload (length-delimited bytes)
    n += _pbLenField(buf + n, 0x12, payload, payloadLen);

    return n;
}

// ── _buildTxPacket ────────────────────────────────────────────────────────
// Build a complete, encrypted, ready-to-transmit Meshtastic OTA packet.
//
// Layout of out[]:
//   [0..15]  16-byte OTA header (plaintext)
//   [16..]   AES-128-CTR encrypted Data proto
//
// The packet ID is drawn from Node.nextPacketId() and becomes part of the
// AES-CTR nonce, so every call produces a different ciphertext even for
// identical payloads — safe to call back-to-back.
//
// out must be at least 256 bytes.  Returns false only on encoding error.
bool LoRa::_buildTxPacket(uint8_t* out, uint8_t& outLen,
                           uint32_t portnum,
                           const uint8_t* payload, size_t payloadLen)
{
    if (payload == nullptr || payloadLen == 0 || payloadLen > 220) return false;

    const uint32_t packetId = Node.nextPacketId();
    const uint32_t fromNode = Node.nodeId();

    // 1. Encode Data proto (portnum + payload) into a stack buffer
    //    Max size: 2 (portnum field) + 2 (payload len) + 220 = 224 bytes
    uint8_t dataBuf[240] = {};
    const size_t dataLen = _encodeData(dataBuf, sizeof(dataBuf),
                                        portnum, payload, payloadLen);
    if (dataLen == 0 || dataLen > 239) return false; // won't fit after header

    // 2. Encrypt Data proto in-place with AES-128-CTR
    //    _decrypt() is CTR-mode: encryption == decryption, same function.
    uint8_t cipherBuf[240] = {};
    if (!_decrypt(dataBuf, dataLen, packetId, fromNode, cipherBuf))
    {
        ESP_LOGW(TAG, "_buildTxPacket: AES-CTR encrypt failed");
        return false;
    }

    // 3. Build 16-byte OTA header
    // to: 0xFFFFFFFF = broadcast
    out[0] = 0xFF; out[1] = 0xFF; out[2] = 0xFF; out[3] = 0xFF;
    // from: this node's ID (little-endian)
    out[4] = static_cast<uint8_t>(fromNode);
    out[5] = static_cast<uint8_t>(fromNode >>  8);
    out[6] = static_cast<uint8_t>(fromNode >> 16);
    out[7] = static_cast<uint8_t>(fromNode >> 24);
    // id: packet ID (little-endian)
    out[8]  = static_cast<uint8_t>(packetId);
    out[9]  = static_cast<uint8_t>(packetId >>  8);
    out[10] = static_cast<uint8_t>(packetId >> 16);
    out[11] = static_cast<uint8_t>(packetId >> 24);
    // flags: hop_limit=3, hop_start=3
    out[12] = FLAGS_BROADCAST;
    // channel hash
    out[13] = DEFAULT_CHAN_HASH;
    // padding
    out[14] = 0x00;
    out[15] = 0x00;

    // 4. Append ciphertext
    memcpy(out + MESH_HDR, cipherBuf, dataLen);
    outLen = static_cast<uint8_t>(MESH_HDR + dataLen);

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
    {
        unixTime = static_cast<uint32_t>(time(nullptr));
    }

    // Convert decimal degrees to Meshtastic integer format (degrees × 1e7)
    const int32_t lat_i = static_cast<int32_t>(lat * 1e7);
    const int32_t lon_i = static_cast<int32_t>(lng * 1e7);
    const int32_t alt_m = static_cast<int32_t>(altM);

    // Encode Position proto (max ~80 bytes for our fields)
    uint8_t posBuf[80] = {};
    const size_t posLen = _encodePosition(posBuf, sizeof(posBuf),
                                           lat_i, lon_i, alt_m,
                                           pdop_x100, sats, unixTime);
    if (posLen == 0) return false;

    // Build and transmit the complete OTA packet
    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_POSITION, posBuf, posLen)) return false;

    ESP_LOGI(TAG, "TX POSITION lat=%.6f lon=%.6f alt=%dm sats=%" PRIu32
             " pdop=%.2f ts=%" PRIu32,
             lat, lng, (int)altM, sats, (double)pdop_x100 / 100.0, unixTime);

    return transmit(pkt, pktLen);
}

// ── sendNodeInfo ──────────────────────────────────────────────────────────
bool LoRa::sendNodeInfo()
{
#if !CONFIG_LORA_TX_ENABLED
    return false;
#endif

    // Encode User proto (id + long_name + short_name + hw_model, ≤80 bytes)
    uint8_t userBuf[80] = {};
    const size_t userLen = _encodeUser(userBuf, sizeof(userBuf),
                                        Node.nodeId(),
                                        Node.longName(),
                                        Node.shortName());
    if (userLen == 0) return false;

    // Build and transmit the complete OTA packet
    uint8_t pkt[256] = {};
    uint8_t pktLen   = 0;
    if (!_buildTxPacket(pkt, pktLen, PORT_NODEINFO, userBuf, userLen)) return false;

    ESP_LOGI(TAG, "TX NODEINFO id=%s long=\"%s\" short=\"%s\" hw=%u",
             Node.nodeIdStr(), Node.longName(), Node.shortName(),
             (unsigned)HW_MODEL);

    return transmit(pkt, pktLen);
}

// ─────────────────────────────────────────────────────────────────────────
// ── _processPacket ────────────────────────────────────────────────────────
// Parse the raw LoRa payload, attempt decryption, decode the Meshtastic
// Data protobuf, and store text messages.
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

    ESP_LOGD(TAG,
        "Rx pkt: to=0x%08" PRIx32 " from=0x%08" PRIx32 " id=0x%08" PRIx32
        " hop=%u ch=0x%02x rssi=%d snr=%.1f",
        to, from, pktId, hopLim, chanHash, rssi, (double)snr);

    const uint8_t* ciphertext  = buf  + MESH_HDR;
    const size_t   cipherLen   = pktLen - MESH_HDR;

    // Decrypt with the default channel PSK
    uint8_t plain[256] = {};
    if (!_decrypt(ciphertext, cipherLen, pktId, from, plain))
    {
        ESP_LOGW(TAG, "AES-CTR decrypt failed for pkt 0x%08" PRIx32, pktId);
        return;
    }

    portENTER_CRITICAL(&_statsLock);
    _stats.decryptOk++;
    portEXIT_CRITICAL(&_statsLock);

    // Decode the Data protobuf
    uint32_t       portnum    = 0;
    const uint8_t* payload    = nullptr;
    size_t         payloadLen = 0;

    if (!_parseData(plain, cipherLen, portnum, payload, payloadLen))
    {
        ESP_LOGD(TAG, "Protobuf parse failed — packet may use a different channel key");
        return;
    }

    if (portnum != PORT_TEXT || payload == nullptr || payloadLen == 0) return;

    // Build and store the MeshMessage
    MeshMessage msg;
    msg.fromNode = from;
    msg.rssi     = rssi;
    msg.snr      = snr;
    msg.valid    = true;
    const size_t copyLen = std::min(payloadLen, sizeof(msg.text) - 1);
    memcpy(msg.text, payload, copyLen);
    msg.text[copyLen] = '\0';

    ESP_LOGI(TAG, "Text from 0x%08" PRIx32 " (rssi=%d snr=%.1f): %s",
             from, rssi, (double)snr, msg.text);

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

    ESP_LOGI(TAG, "Listening on %u Hz", (unsigned)CONFIG_LORA_FREQUENCY_HZ);

    TickType_t lastDiagTick = xTaskGetTickCount();

#if CONFIG_LORA_TX_ENABLED
    // Position broadcast interval — computed once from Kconfig.
    // Initialise _lastPosTxTick to (now - interval) so the first broadcast
    // fires as soon as a fix is available, not after one full interval.
    const TickType_t posTxInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_POSITION_TX_INTERVAL_SEC * 1000UL);
    _lastPosTxTick = xTaskGetTickCount() - posTxInterval;
#endif

    // ── Receive loop ──────────────────────────────────────────────────────
    while (true)
    {
        // Wait for DIO1 IRQ (notified by ISR) or wake on timeout to check state.
        // Timeout of 30 s ensures we re-enter RX even if DIO1 was missed.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30000));

        const uint16_t irq = _getIrqStatus();
        _clearIrq(IRQ_RX_MASK | IRQ_TX_MASK);

        if (irq & IRQ_RX_DONE)
        {
            portENTER_CRITICAL(&_statsLock);
            _stats.rxPackets++;
            portEXIT_CRITICAL(&_statsLock);

                // GetRxBufferStatus — TX=[cmd,NOP,NOP], RX=[status,payloadLen,rxStartPtr]
                // Data at rx[1:2], not rx[2:3].
                uint8_t txs[3] = { CMD_GET_RX_BUF_STATUS, 0, 0 };
                uint8_t rxs[3] = {};
                _transact(txs, rxs, sizeof(txs));
                const uint8_t payloadLen = rxs[1];
                const uint8_t rxPtr     = rxs[2];

            if (payloadLen > 0)
            {
                // GetPacketStatus — TX=[cmd,NOP,NOP,NOP], RX=[status,rssi,snr,signal_rssi]
                // Data at rx[1] and rx[2].
                uint8_t txp[4] = { CMD_GET_PKT_STATUS, 0, 0, 0 };
                uint8_t rxp[4] = {};
                _transact(txp, rxp, sizeof(txp));
                const int16_t rssi = -(static_cast<int16_t>(rxp[1])) / 2;
                const float   snr  = static_cast<float>(static_cast<int8_t>(rxp[2])) / 4.0f;

                // ReadBuffer → [cmd, offset, dummy, data...]
                const size_t txLen = 3 + payloadLen;
                uint8_t txb[3 + 255] = { CMD_READ_BUFFER, rxPtr, 0x00 };
                uint8_t rxb[3 + 255] = {};
                _waitBusy();
                spi_transaction_t t = {};
                t.length    = txLen * 8;
                t.tx_buffer = txb;
                t.rx_buffer = rxb;
                spi_device_polling_transmit(_spi, &t);

                // Data begins at rxb[3] (after cmd, offset, dummy status bytes)
                _processPacket(rxb + 3, payloadLen, rssi, snr);
            }
        }
        else if (irq & IRQ_HEADER_ERR)
        {
            portENTER_CRITICAL(&_statsLock);
            _stats.headerErrors++;
            portEXIT_CRITICAL(&_statsLock);
            ESP_LOGW(TAG, "LoRa header error — IRQ=0x%04x", irq);
        }
        else if (irq & IRQ_CRC_ERROR)
        {
            portENTER_CRITICAL(&_statsLock);
            _stats.crcErrors++;
            portEXIT_CRITICAL(&_statsLock);
            ESP_LOGW(TAG, "LoRa CRC error — IRQ=0x%04x", irq);
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
            const LoRaStats s = stats();
            ESP_LOGI(TAG,
                "DIAG: rx=%" PRIu32 "  crc_err=%" PRIu32
                "  hdr_err=%" PRIu32 "  decrypt_ok=%" PRIu32
                "  text=%" PRIu32 "  tx=%" PRIu32 "  tx_err=%" PRIu32
                "  tx_timeout=%" PRIu32 "  last_rssi=%d  last_snr=%.1f",
                s.rxPackets, s.crcErrors, s.headerErrors,
                s.decryptOk, s.textMessages,
                s.txPackets, s.txErrors, s.txTimeouts,
                s.lastRssi, (double)s.lastSnr);
        }

        // Re-enter RX — required after every RX_DONE or error
        _setRx();

#if CONFIG_LORA_TX_ENABLED
        // ── Periodic GPS position broadcast ───────────────────────────────
        // now was sampled before the diag log check above; re-read to keep
        // the interval accurate regardless of how long processing took.
        if ((xTaskGetTickCount() - _lastPosTxTick) >= posTxInterval)
        {
            _lastPosTxTick = xTaskGetTickCount();
            if (gps.isFixed())
            {
                // HDOP × 100 as a PDOP approximation (we only have HDOP from
                // TinyGPS++; Meshtastic accepts this field as an unsigned int).
                const uint32_t pdop_x100 =
                    static_cast<uint32_t>(gps.hdop() * 100.0f);
                sendPosition(gps.lat(), gps.lng(), gps.altitude(),
                             pdop_x100, gps.satellites());
            }
            else
            {
                ESP_LOGD(TAG, "Position TX skipped — no GPS fix");
            }
        }
#endif
    }
}

/* extern */
LoRa Lora("LoRa", 10240);
