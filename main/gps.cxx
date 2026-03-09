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

#include "gps.h"
#include "hardware.h"
#include <esp_log.h>
#include <inttypes.h>
#include <driver/uart.h>
#include <driver/gpio.h>

static const char* TAG = "gps";

// ── Hardware ──────────────────────────────────────────────────────────────
static constexpr uart_port_t GPS_UART = UART_NUM_1;

// ── Baud-rate probe candidates ────────────────────────────────────────────
// Tried in order.  Primary (115200) matches the UC6580 factory default.
// 9600 is the NMEA industry default and covers modules that have been
// factory-reset or that ship at a lower rate.
static constexpr int BAUD_CANDIDATES[]   = {115200, 9600};
static constexpr int NUM_BAUD_CANDIDATES = sizeof(BAUD_CANDIDATES) / sizeof(BAUD_CANDIDATES[0]);

// How long to observe a baud rate before declaring it wrong.
// Needs to be long enough to receive at least a few NMEA sentences (1 Hz).
static constexpr uint32_t BAUD_PROBE_MS    = 15000;

// No characters at all for this long → assume hardware problem → power-cycle.
static constexpr uint32_t WATCHDOG_MS      = 60000;

// A fix is considered stale after this many ms.
static constexpr uint32_t FIX_MAX_AGE_MS   = 3000;

// Periodic health/diagnostic log interval.
static constexpr uint32_t DIAG_INTERVAL_MS = 30000;

// ── Constructor ───────────────────────────────────────────────────────────
GPS::GPS(const char* name, uint16_t stack_size)
:   Task(name, stack_size, 1)
{ }

// ── _installUart ──────────────────────────────────────────────────────────
// Tears down any existing driver and reinstalls at the requested baud rate.
// On success _uartQueue is populated by the IDF driver.
esp_err_t GPS::_installUart(int baud)
{
    if (uart_is_driver_installed(GPS_UART)) {
        uart_driver_delete(GPS_UART);
        _uartQueue = nullptr;
    }

    // Zero-initialise so the flags/backup_before_sleep fields are set,
    // avoiding -Wmissing-field-initializers on IDF v5.5+.
    uart_config_t cfg = {};
    cfg.baud_rate           = baud;
    cfg.data_bits           = UART_DATA_8_BITS;
    cfg.parity              = UART_PARITY_DISABLE;
    cfg.stop_bits           = UART_STOP_BITS_1;
    cfg.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    cfg.rx_flow_ctrl_thresh = 122;
    cfg.source_clk          = UART_SCLK_DEFAULT;

    // Install with a 16-deep event queue so FIFO-overflow and buffer-full
    // events are never dropped.  RX ring buffer is 1024 bytes (~89 ms at
    // 115200 baud — comfortably covers two 1 Hz NMEA bursts).
    esp_err_t err = uart_driver_install(GPS_UART,
                                        /*rx_buf*/           1024,
                                        /*tx_buf*/           0,
                                        /*event_queue_size*/ 16,
                                        &_uartQueue,
                                        /*intr_alloc_flags*/ 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install @ %d baud: %s", baud, esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(GPS_UART, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(GPS_UART, GPS_TX, GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin TX=%d RX=%d: %s", GPS_TX, GPS_RX, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%d ready: TX=GPIO%d  RX=GPIO%d  @ %d baud  (rx_buf=1024 evq=16)",
             (int)GPS_UART, GPS_TX, GPS_RX, baud);
    return ESP_OK;
}

// ── run ───────────────────────────────────────────────────────────────────
void GPS::run(void* /*data*/)
{
    ESP_LOGI(TAG, "GPS task started  RX=GPIO%d  TX=GPIO%d", GPS_RX, GPS_TX);

    // ── Power up VGNSS ────────────────────────────────────────────────────
    gpio_set_direction(static_cast<gpio_num_t>(Hardware::VEXT_CTRL), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(Hardware::VEXT_CTRL), 1);
    vTaskDelay(pdMS_TO_TICKS(500));  // UC6580 needs ~500 ms before UART is ready

    // ── Install UART at primary baud ──────────────────────────────────────
    int  baudIdx    = 0;
    bool baudLocked = false;

    if (_installUart(BAUD_CANDIDATES[baudIdx]) != ESP_OK) {
        ESP_LOGE(TAG, "Fatal: cannot install UART driver — GPS task exiting");
        vTaskDelete(nullptr);
        return;
    }

    // ── Initial state ─────────────────────────────────────────────────────
    TickType_t now            = xTaskGetTickCount();
    TickType_t lastCharTick   = now;  // reset whenever bytes arrive
    TickType_t probeStartTick = now;  // reset on every baud switch
    TickType_t lastDiagTick   = now;
    TickType_t lastHouseTick  = now;

    uint32_t probeStartChars  = 0;
    uint32_t probeStartPassed = 0;

    bool     prevFixed = false;
    uint32_t prevSats  = UINT32_MAX;  // sentinel: "not yet logged"

    // ── Event loop ────────────────────────────────────────────────────────
    while (true)
    {
        uart_event_t event;
        // Block up to 1 s for a UART event.  The 1 s timeout drives all
        // time-based checks (watchdog, baud probe, housekeeping) even when
        // no data is arriving, without any busy-waiting.
        const bool gotEvent =
            (xQueueReceive(_uartQueue, &event, pdMS_TO_TICKS(1000)) == pdTRUE);

        if (gotEvent)
        {
            switch (event.type)
            {
                case UART_DATA:
                {
                    // Drain the entire RX ring buffer in one pass.
                    // 128-byte stack buffer covers a full NMEA sentence per read.
                    uint8_t buf[128];
                    int n;
                    while ((n = uart_read_bytes(GPS_UART, buf, sizeof(buf), 0)) > 0)
                    {
                        for (int i = 0; i < n; i++)
                            _gps.encode(static_cast<char>(buf[i]));
                    }
                    lastCharTick = xTaskGetTickCount();
                    break;
                }

                case UART_FIFO_OVF:
                    // FIFO overflow: data lost.  Flush so the driver can resume.
                    ESP_LOGW(TAG, "UART FIFO overflow — data lost; flushing"
                             "  (baud=%d, possible overload or mismatch)",
                             BAUD_CANDIDATES[baudIdx]);
                    uart_flush_input(GPS_UART);
                    xQueueReset(_uartQueue);
                    break;

                case UART_BUFFER_FULL:
                    // RX ring buffer full — reader isn't keeping up.
                    ESP_LOGW(TAG, "UART RX buffer full — flushing  (baud=%d)",
                             BAUD_CANDIDATES[baudIdx]);
                    uart_flush_input(GPS_UART);
                    xQueueReset(_uartQueue);
                    break;

                case UART_FRAME_ERR:
                    // Start-bit / stop-bit mismatch — almost always a baud mismatch.
                    // The probe logic below will switch automatically; just log here.
                    ESP_LOGW(TAG, "UART frame error — likely baud mismatch  (current: %d)",
                             BAUD_CANDIDATES[baudIdx]);
                    break;

                default:
                    break;
            }
        }

        now = xTaskGetTickCount();

        // ── Watchdog: no chars for WATCHDOG_MS ───────────────────────────
        // Indicates a hardware problem (VGNSS not powered, wiring fault, module
        // crashed).  Power-cycle the VGNSS rail and restart the baud probe.
        if ((now - lastCharTick) >= pdMS_TO_TICKS(WATCHDOG_MS))
        {
            ESP_LOGW(TAG,
                "Watchdog fired: no chars for %u s — power-cycling VGNSS",
                (unsigned)(WATCHDOG_MS / 1000));

            gpio_set_level(static_cast<gpio_num_t>(Hardware::VEXT_CTRL), 0);
            vTaskDelay(pdMS_TO_TICKS(500));   // let capacitors discharge
            gpio_set_level(static_cast<gpio_num_t>(Hardware::VEXT_CTRL), 1);
            vTaskDelay(pdMS_TO_TICKS(500));   // wait for module UART ready

            // Restart probing from the primary baud.
            baudIdx    = 0;
            baudLocked = false;
            _installUart(BAUD_CANDIDATES[baudIdx]);

            now              = xTaskGetTickCount();
            lastCharTick     = now;
            probeStartTick   = now;
            probeStartChars  = _gps.charsProcessed();
            probeStartPassed = _gps.passedChecksum();
            continue;
        }

        // ── Auto-baud probe ───────────────────────────────────────────────
        // Compares snapshots taken at install time against current counters.
        // Three outcomes:
        //   - deltaPassed > 0            → correct baud, lock in
        //   - deltaChars > 0, passed = 0 → bytes arriving but not parseable
        //                                  → wrong baud, try next candidate
        //   - deltaChars = 0             → nothing arriving at this baud either
        //                                  → also try next (watchdog handles HW fault)
        if (!baudLocked)
        {
            const uint32_t deltaChars  = _gps.charsProcessed() - probeStartChars;
            const uint32_t deltaPassed = _gps.passedChecksum()  - probeStartPassed;

            if (deltaPassed > 0)
            {
                baudLocked = true;
                ESP_LOGI(TAG, "Baud locked at %d  (passed=%" PRIu32 ")",
                         BAUD_CANDIDATES[baudIdx], deltaPassed);
            }
            else if ((now - probeStartTick) >= pdMS_TO_TICKS(BAUD_PROBE_MS))
            {
                const int prevBaud = BAUD_CANDIDATES[baudIdx];
                baudIdx = (baudIdx + 1) % NUM_BAUD_CANDIDATES;

                if (deltaChars > 0)
                    ESP_LOGW(TAG,
                        "Baud probe: %" PRIu32 " chars, 0 valid sentences at %d"
                        " — switching to %d",
                        deltaChars, prevBaud, BAUD_CANDIDATES[baudIdx]);
                else
                    ESP_LOGW(TAG,
                        "Baud probe: 0 chars at %d — switching to %d",
                        prevBaud, BAUD_CANDIDATES[baudIdx]);

                _installUart(BAUD_CANDIDATES[baudIdx]);

                now              = xTaskGetTickCount();
                probeStartTick   = now;
                lastCharTick     = now;   // reset watchdog so it doesn't fire immediately
                probeStartChars  = _gps.charsProcessed();
                probeStartPassed = _gps.passedChecksum();
            }
        }

        // ── 30-second diagnostic log ──────────────────────────────────────
        if ((now - lastDiagTick) >= pdMS_TO_TICKS(DIAG_INTERVAL_MS))
        {
            lastDiagTick = now;

            const uint32_t chars  = _gps.charsProcessed();
            const uint32_t passed = _gps.passedChecksum();
            const uint32_t failed = _gps.failedChecksum();
            const uint32_t sats   = _gps.satellites.isValid() ? _gps.satellites.value() : 0u;
            const double   hdop   = _gps.hdop.isValid()       ? _gps.hdop.hdop()        : 99.9;
            const bool     fixed  = _gps.location.isValid() &&
                                    _gps.location.age() < FIX_MAX_AGE_MS;
            size_t rxBuf = 0;
            uart_get_buffered_data_len(GPS_UART, &rxBuf);

            if (chars == 0)
            {
                ESP_LOGW(TAG,
                    "DIAG: 0 chars — check VGNSS (GPIO%d) and UART (RX=%d TX=%d)  rx_buf=%u",
                    Hardware::VEXT_CTRL, GPS_RX, GPS_TX, (unsigned)rxBuf);
            }
            else
            {
                ESP_LOGI(TAG,
                    "DIAG: chars=%" PRIu32 "  ok=%" PRIu32 "  fail=%" PRIu32
                    "  sats=%" PRIu32 "  HDOP=%.1f  fixed=%s  baud=%s%d  rx_buf=%u",
                    chars, passed, failed, sats, hdop,
                    fixed ? "YES" : "no",
                    baudLocked ? "" : "~",  // ~ prefix = still probing
                    BAUD_CANDIDATES[baudIdx],
                    (unsigned)rxBuf);

                if (failed > 0 && !baudLocked)
                    ESP_LOGW(TAG,
                        "DIAG: %" PRIu32 " checksum failure(s) — baud probe in progress",
                        failed);
                if (!fixed && passed > 0)
                    ESP_LOGI(TAG,
                        "DIAG: sentences parsing OK — acquiring satellite fix"
                        " (normal during cold start)");
            }
        }

        // ── 1-second housekeeping: fix state + position log ───────────────
        if ((now - lastHouseTick) >= pdMS_TO_TICKS(1000))
        {
            lastHouseTick = now;

            const bool fixed = _gps.location.isValid() &&
                               _gps.location.age() < FIX_MAX_AGE_MS;

            // Notify the display task on transition.
            if (fixed != prevFixed)
            {
                prevFixed = fixed;
                Heltec.showGpsState(fixed);
                ESP_LOGI(TAG,
                    "GPS fix %s  chars=%" PRIu32 "  ok=%" PRIu32
                    "  fail=%" PRIu32 "  baud=%d",
                    fixed ? "acquired" : "lost",
                    _gps.charsProcessed(), _gps.passedChecksum(),
                    _gps.failedChecksum(), BAUD_CANDIDATES[baudIdx]);
            }

            if (fixed)
            {
                const uint32_t sats = _gps.satellites.isValid()
                                    ? _gps.satellites.value() : 0u;
                const double   hdop = _gps.hdop.isValid()
                                    ? _gps.hdop.hdop() : 99.9;

                // Log satellite count only when it changes.
                if (sats != prevSats)
                {
                    prevSats = sats;
                    ESP_LOGI(TAG, "Satellites: %" PRIu32 "  HDOP: %.1f", sats, hdop);
                }

                // Log position at 1 Hz while HDOP is reasonable.
                if (hdop < 5.0)
                {
                    ESP_LOGI(TAG, "Lat: %.5f  Lng: %.5f  Alt: %.1f m",
                             _gps.location.lat(),
                             _gps.location.lng(),
                             _gps.altitude.isValid() ? _gps.altitude.meters() : 0.0);
                }
            }
        }
    }
}

// ── Diagnostic accessors ──────────────────────────────────────────────────
bool GPS::isFixed() const
{
    return _gps.location.isValid() && _gps.location.age() < FIX_MAX_AGE_MS;
}

uint32_t GPS::satellites()
{
    return _gps.satellites.isValid() ? _gps.satellites.value() : 0u;
}

float GPS::hdop()
{
    return _gps.hdop.isValid() ? static_cast<float>(_gps.hdop.hdop()) : 99.9f;
}

uint32_t GPS::passedChecksum() const
{
    return _gps.passedChecksum();
}

uint32_t GPS::failedChecksum() const
{
    return _gps.failedChecksum();
}

/* extern */
GPS gps("GPS", 8192);
