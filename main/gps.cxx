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

static const char* TAG = "gps";

// Use UART1 for the UC6580.
static constexpr uart_port_t GPS_UART = UART_NUM_1;

GPS::GPS(String const& name, uint16_t stack_size)
:   Task(name, stack_size, 1)
{ }

void GPS::run(void* /*data*/)
{
    ESP_LOGI(TAG, "Starting GPS (RX=%d TX=%d)", GPS_RX, GPS_TX);

    // Power the VGNSS rail — same GPIO as VEXT used by the TFT.
    // TFT::init() has already driven it HIGH, but guard here in case
    // the draw task hasn't started yet on this boot.
    pinMode(Hardware::VEXT_CTRL, OUTPUT);
    digitalWrite(Hardware::VEXT_CTRL, HIGH);

    // UC6580 needs ~500 ms after VGNSS rises before UART is ready.
    vTaskDelay(pdMS_TO_TICKS(500));

    // ── Install IDF UART1 driver directly ────────────────────────────────
    // Using the IDF driver bypasses the Arduino HardwareSerial wrapper and
    // avoids any dual-instance conflict with the global Serial1 object.
    // It also gives us explicit esp_err_t feedback for every step.

    // If the driver was previously installed (e.g. by a prior boot without
    // a full power-cycle), tear it down first so we start clean.
    if (uart_is_driver_installed(GPS_UART)) {
        ESP_LOGW(TAG, "UART%d already installed — deleting and reinstalling", GPS_UART);
        uart_driver_delete(GPS_UART);
    }

    const uart_config_t uart_cfg = {
        .baud_rate           = 115200,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(GPS_UART,
                                        /*rx_buf*/ 512,
                                        /*tx_buf*/ 0,
                                        /*event_queue_size*/ 0,
                                        /*event_queue_handle*/ nullptr,
                                        /*intr_alloc_flags*/ 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "UART%d driver installed (rx_buf=512)", GPS_UART);

    err = uart_param_config(GPS_UART, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
    }

    // uart_set_pin signature: (port, tx_gpio, rx_gpio, rts_gpio, cts_gpio)
    err = uart_set_pin(GPS_UART,
                       GPS_TX, GPS_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin(TX=%d RX=%d) failed: %s",
                 GPS_TX, GPS_RX, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "UART%d pins set: TX=GPIO%d  RX=GPIO%d  @ 115200",
                 GPS_UART, GPS_TX, GPS_RX);
    }

    bool       prevFixed    = false;
    uint32_t   prevSats     = UINT32_MAX; // sentinel: "not yet logged"
    TickType_t lastTick     = xTaskGetTickCount();
    TickType_t lastDiagTick = lastTick;

    // How stale a location fix is allowed to be before we consider it lost.
    static constexpr uint32_t FIX_MAX_AGE_MS   = 3000;
    // Interval for the 1-second housekeeping tick.
    static constexpr uint32_t TICK_INTERVAL_MS  = 1000;
    // Interval for the periodic health/diagnostic log (30 s).
    static constexpr uint32_t DIAG_INTERVAL_MS  = 30000;

    while (true)
    {
        // ── Drain UART RX ring-buffer ─────────────────────────────────────
        // uart_read_bytes with timeout=0 is non-blocking: it returns however
        // many bytes are already in the ring buffer, or 0 if there are none.
        uint8_t byte;
        while (uart_read_bytes(GPS_UART, &byte, 1, 0) == 1)
        {
            _gps.encode(static_cast<char>(byte));
        }

        TickType_t now = xTaskGetTickCount();

        // ── 30-second diagnostic log (unconditional) ──────────────────────
        if ((now - lastDiagTick) >= pdMS_TO_TICKS(DIAG_INTERVAL_MS))
        {
            lastDiagTick = now;
            const uint32_t chars  = _gps.charsProcessed();
            const uint32_t passed = _gps.passedChecksum();
            const uint32_t failed = _gps.failedChecksum();
            const uint32_t sats   = _gps.satellites.isValid() ? _gps.satellites.value() : 0u;
            const double   hdop   = _gps.hdop.isValid()       ? _gps.hdop.hdop()        : 99.9;
            const bool     diag_fixed = _gps.location.isValid() &&
                                        _gps.location.age() < FIX_MAX_AGE_MS;

            // Also query the IDF ring-buffer for any bytes we haven't read yet.
            size_t rx_buffered = 0;
            uart_get_buffered_data_len(GPS_UART, &rx_buffered);

            if (chars == 0)
            {
                ESP_LOGW(TAG,
                    "DIAG: 0 chars processed  rx_buf=%u"
                    " — check VGNSS (GPIO%d) and UART pins (TX=%d RX=%d)",
                    (unsigned)rx_buffered,
                    Hardware::VEXT_CTRL, GPS_TX, GPS_RX);
            }
            else
            {
                ESP_LOGI(TAG,
                    "DIAG: chars=%" PRIu32 "  ok=%" PRIu32 "  fail=%" PRIu32
                    "  sats=%" PRIu32 "  HDOP=%.1f  fixed=%s  rx_buf=%u",
                    chars, passed, failed, sats, hdop,
                    diag_fixed ? "YES" : "no", (unsigned)rx_buffered);

                if (failed > 0)
                    ESP_LOGW(TAG,
                        "DIAG: %" PRIu32 " checksum failure(s) — possible baud mismatch (try 9600)",
                        failed);
                if (!diag_fixed && passed > 0)
                    ESP_LOGI(TAG,
                        "DIAG: sentences parsing OK — still acquiring satellite fix (normal cold start)");
            }
        }

        // ── 1-second housekeeping tick ────────────────────────────────────
        if ((now - lastTick) < pdMS_TO_TICKS(TICK_INTERVAL_MS))
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // short yield, stay responsive
            continue;
        }
        lastTick = now;

        // A fix is considered valid when location is fresh (< FIX_MAX_AGE_MS).
        const bool fixed = _gps.location.isValid() &&
                           _gps.location.age() < FIX_MAX_AGE_MS;

        // ── Update toolbar icon on state change ───────────────────────────
        if (fixed != prevFixed)
        {
            prevFixed = fixed;
            Heltec.showGpsState(fixed);
            ESP_LOGI(TAG, "GPS fix %s (chars=%" PRIu32 "  ok=%" PRIu32 "  fail=%" PRIu32 ")",
                     fixed ? "acquired" : "lost",
                     _gps.charsProcessed(), _gps.passedChecksum(), _gps.failedChecksum());
        }

        // ── Log position info while fixed ─────────────────────────────────
        if (fixed)
        {
            const uint32_t sats = _gps.satellites.isValid()
                                ? _gps.satellites.value() : 0u;
            const double   hdop = _gps.hdop.isValid()
                                ? _gps.hdop.hdop() : 99.9;

            // Log satellite count only when it changes to avoid log spam.
            if (sats != prevSats)
            {
                prevSats = sats;
                ESP_LOGI(TAG, "Satellites: %" PRIu32 "  HDOP: %.1f", sats, hdop);
            }

            // Log position every tick (1 Hz) while HDOP is reasonable.
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

/* extern */
GPS gps("GPS", 6144);
