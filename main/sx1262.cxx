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
 * sx1262.cxx — SX1262 SPI hardware driver layer.
 *
 * Implements all low-level SX1262 commands (init, calibration, modulation,
 * frequency, RX/TX control, IRQ handling) and the blocking transmit() method.
 *
 * Higher-level Meshtastic protocol concerns live in meshtastic_proto.cxx.
 * The FreeRTOS task entry point and receive loop live in lora.cxx.
 */

#include "lora.h"
#include "lora_internal.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

static const char* TAG = "lora";

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
// Result: preamble and sync word are detected (they're symmetric /
// correlator-based), but every header CRC fails because the data symbols
// are decoded with inverted chirp direction.  This produces the symptom:
// hdr_err increments, rx=0 forever, RSSI/SNR are stale.
//
// Fix: after any SetPacketParams call, read-modify-write register 0x0736
// to enforce the correct polarity.  Standard IQ: clear bit 2 (val &= ~0x04).
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
// so IRQ_TX_DONE (bit 0) and IRQ_RX_DONE (bit 1) were never visible.
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
//   2 = STBY_RC   3 = STBY_XOSC   4 = FS   5 = RX   6 = TX
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
// TX_DONE fires after the last bit leaves the antenna.
//
// Why polling instead of ISR:
//   ulTaskNotifyTake fires on a rising edge of DIO1.  If DIO1 is already HIGH
//   from an incompletely-cleared prior RX event, ClearIrq may not bring it LOW
//   in time and there is NO rising edge when TX_DONE fires — the ISR never runs
//   and ulTaskNotifyTake times out.
//
// Sequence:
//   1  SetStandby(RC)        — valid from any state, including continuous RX
//   2  ClearDeviceErrors     — clean slate for post-TX diagnostic
//   3  SetRfFrequency        — force PLL recalibration (Rx→Tx errata)
//   4  SetPktParams          — actual payload length
//   5  WriteBuffer           — load payload
//   6  SetDioIrqParams       — TX_DONE | TIMEOUT on DIO1
//   7  Drain stale notifications
//   8  ClearIrq              — bring DIO1 LOW before SetTx
//   9  SetTx(0)              — begin transmission, no hardware timeout
//  10  Poll BUSY HIGH        — confirms chip accepted SetTx
//  11  Poll BUSY LOW         — chip entered TX mode (TCXO + PLL ready)
//  12  Poll chip mode != TX  — TX_DONE (chip returns to STBY_RC)
//  13  Read + clear IRQ, restore RX config, re-enter RX
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

    // 2. Clear any stale device errors from a previous failed TX
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

    // ── Phase B2: confirm chip is actually in TX mode (mode == 6) ────────
    {
        const uint8_t mode = _getChipMode();
        if (mode != 0x06)
        {
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
    // Heltec Wireless Tracker 1.1 uses a 1.8 V TCXO on DIO3 (tcxoVoltage=0x02).
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

    // ── PA config for HP PA on SX1262 ─────────────────────────────────────
    // paDutyCycle=4, hpMax=7, deviceSel=0 (SX1262), paLut=1
    { uint8_t t[] = { CMD_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 };
      _transact(t, nullptr, sizeof(t)); }

    // ── TX output power + ramp time 200 µs ───────────────────────────────
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
    // Must be called after every SetPacketParams.
    _fixIqPolarity();

    // ── RX gain ──────────────────────────────────────────────────────────
    // 0x94 = normal LNA gain (default after reset, handles full input range).
    // 0x96 = boosted LNA gain (+3 dB sensitivity) — NOT used here because
    // it reduces the maximum tolerable input level, causing CRC errors from
    // nearby nodes (e.g. at -29 dBm). With a noise floor of -104 dBm we
    // have >70 dB of link margin — sensitivity boost provides no benefit.
    _writeReg(REG_RX_GAIN, 0x94);

    // ── Buffer base addresses (TX=0, RX=128 — separate regions to prevent
    //    overlap during the TX→RX transition) ────────────────────────────────
    { uint8_t t[] = { CMD_SET_BUF_BASE_ADDR, 0x00, 0x80 };
      _transact(t, nullptr, sizeof(t)); }

    // ── LoRa sync word = Meshtastic (0x2B in RadioLib = 0x24B4 in regs)
    // RadioLib converts 0x2B → MSB=(sw & 0xF0)|0x04=0x24, LSB=((sw & 0x0F)<<4)|0x04=0xB4
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
