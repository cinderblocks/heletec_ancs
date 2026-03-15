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
 * lora.cxx — FreeRTOS task entry point for the LoRa subsystem.
 *
 * Contains:
 *   - LoRa constructor (sets task priority)
 *   - LoRa::run() — SPI init, SX1262 init, DIO1 ISR install, receive loop,
 *                   periodic TX scheduler (position, nodeinfo, telemetry,
 *                   map report)
 *   - extern Lora instance
 *
 * SX1262 hardware layer:   sx1262.cxx
 * Meshtastic protocol:     meshtastic_proto.cxx
 * Shared internal constants: lora_internal.h
 */

#include "lora.h"
#include "lora_internal.h"
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
#include <cinttypes>
#include <ctime>

static const char* TAG = "lora";

// ── Constructor ───────────────────────────────────────────────────────────
LoRa::LoRa(const char* name, uint16_t stackSize)
:   Task(name, stackSize, 4)  // priority 4 — below BLE (5), above draw (3)
{ }

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
    // Position broadcast interval — initialised to (now - interval) so the
    // first broadcast fires as soon as a GPS fix is available, not after one
    // full interval.
    const TickType_t posTxInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_POSITION_TX_INTERVAL_SEC * 1000UL);
    _lastPosTxTick = xTaskGetTickCount() - posTxInterval;

    // NodeInfo interval — boot broadcast: 3 retransmits at 15 s intervals
    // before settling to the configured period.  Gives nearby nodes 3 chances
    // to receive our identity packet in the first 30 s.
    const TickType_t nodeInfoInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_NODEINFO_TX_INTERVAL_SEC * 1000UL);
    sendNodeInfo();
    _nodeInfoBootCount = 1;
    _lastNodeInfoTxTick = xTaskGetTickCount()
                        - nodeInfoInterval
                        + pdMS_TO_TICKS(15 * 1000UL); // next in 15 s

    // Telemetry interval — first TX fires 15 s after boot
    const TickType_t telemetryInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_TELEMETRY_TX_INTERVAL_SEC * 1000UL);
    _lastTelemetryTxTick = xTaskGetTickCount()
                         - telemetryInterval
                         + pdMS_TO_TICKS(15 * 1000UL);

    // MAP_REPORT interval — first TX fires 30 s after boot
    const TickType_t mapReportInterval = pdMS_TO_TICKS(
        (uint32_t)CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC * 1000UL);
    _lastMapReportTxTick = xTaskGetTickCount()
                         - mapReportInterval
                         + pdMS_TO_TICKS(30 * 1000UL);
#endif

    // ── Receive loop ──────────────────────────────────────────────────────
    while (true)
    {
        // Wait for DIO1 IRQ (notified by ISR) or 30 s timeout.
        // The timeout ensures we re-enter RX even if DIO1 was missed.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30000));

        const uint16_t irq = _getIrqStatus();
        _clearIrq(0xFFFF);

        // Count preamble/header-valid events from the global IRQ mask.
        // These don't fire DIO1 but are visible in GetIrqStatus.
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

            // GetRxBufferStatus
            uint8_t txs[4] = { CMD_GET_RX_BUF_STATUS, 0, 0, 0 };
            uint8_t rxs[4] = {};
            _transact(txs, rxs, sizeof(txs));
            const uint8_t payloadLen = rxs[2];
            const uint8_t rxPtr      = rxs[3];

            ESP_LOGI(TAG, "RX_DONE irq=0x%04x len=%u ptr=0x%02x",
                     irq, payloadLen, rxPtr);

            // ── CRC error check (must be inside RX_DONE, not a separate else-if)
            // The SX1262 sets BOTH RX_DONE and CRC_ERROR for packets that
            // complete reception but fail the LoRa CRC.  Without this check,
            // corrupted packets would be fed to _processPacket and AES-CTR
            // would silently "decrypt" garbage.
            if (irq & IRQ_CRC_ERROR)
            {
                portENTER_CRITICAL(&_statsLock);
                _stats.rxPackets--;   // undo the increment above
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
        if (gps.isFixed())
        {
            const TickType_t nowTick   = xTaskGetTickCount();
            const TickType_t sincePos  = nowTick - _lastPosTxTick;
            const TickType_t minPosInterval =
                pdMS_TO_TICKS((uint32_t)MIN_POS_TX_INTERVAL_SEC * 1000UL);

            bool doTx = false;

            if (sincePos >= posTxInterval)
            {
                doTx = true;
            }
            else if (sincePos >= minPosInterval)
            {
                const int32_t cur_lat_i = static_cast<int32_t>(gps.lat() * 1e7);
                const int32_t cur_lon_i = static_cast<int32_t>(gps.lng() * 1e7);
                if (_hasTxPos)
                {
                    // 0.0005° ≈ 55m at equator — reasonable trigger for a moving tracker
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
                    doTx = true; // first fix — send immediately
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
            // the log or immediately fire when fix is later acquired.
            _lastPosTxTick = xTaskGetTickCount();
            ESP_LOGD(TAG, "Position TX skipped — no GPS fix");
        }

        // ── Periodic NodeInfo broadcast ───────────────────────────────────
        if ((xTaskGetTickCount() - _lastNodeInfoTxTick) >= nodeInfoInterval)
        {
            sendNodeInfo();
            _nodeInfoBootCount++;
            // First 3 broadcasts: 15 s retransmit window for reliable boot
            // discovery.  From the 4th onward: normal configured interval.
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

        // ── Periodic MAP_REPORT broadcast ─────────────────────────────────
#if CONFIG_LORA_MAP_REPORT_TX_INTERVAL_SEC > 0
        if ((xTaskGetTickCount() - _lastMapReportTxTick) >= mapReportInterval)
        {
            _lastMapReportTxTick = xTaskGetTickCount();
            sendMapReport(); // silently skips when no GPS fix
        }
#endif
#endif // CONFIG_LORA_TX_ENABLED
    }
}

LoRa Lora("LoRa", 10240);
