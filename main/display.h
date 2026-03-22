/**
 * Copyright (c) 2024-2026 Sjofn LLC
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

#pragma once

#include "mesh_codec.h"
#include "tft.h"
#include "util.h"    // conn_state_def

struct notification_def;  // forward declaration — full type in notificationservice.h

/**
 * Display — application-layer screen renderer for the Heltec Wireless
 * Tracker 1.1 ST7735 TFT (160 × 80 px).
 *
 * Owns the TFT driver instance.  All draw methods accept their data as
 * value/const-ref parameters so callers retain full ownership of the
 * underlying state objects.
 *
 * Thread-safety: none.  All methods must be called from the single draw
 * task (Hardware::startDrawing).
 */
class Display {
public:
    // ── Screen layout constants ───────────────────────────────────────────
    static constexpr uint16_t HEADER_COLOR  = 0x3190;
    static constexpr uint8_t  HEADER_HEIGHT = 20;

    /**
     * Construct the display renderer.  Pin numbers match the Heltec
     * Wireless Tracker 1.1 schematic; pass them from Hardware's constants
     * rather than hard-coding here so the class remains board-agnostic.
     */
    Display(int8_t cs, int8_t rst, int8_t dc,
            int8_t sclk, int8_t mosi, int8_t led, int8_t vext);

    /// Initialise the TFT driver and show the boot splash screen.
    void init();

    // ── Header bar (top 20 px) ────────────────────────────────────────────
    /// Fill the top HEADER_HEIGHT rows with HEADER_COLOR.  Call once at boot
    /// before the individual icon methods.
    void paintHeaderBackground();
    void showBLEState(conn_state_def state);
    void showBatteryLevel(uint8_t pct, bool isCharging);
    /// Idempotent: skips the SPI write when state is unchanged.
    void showCallState(bool active);
    void showGpsState(bool fixed);
    void showLoraState(bool connected);
    /// @param ts Null-terminated time string, format "HH:MM" (5 chars).
    void showTime(const char* ts);

    // ── Body area (below header) ──────────────────────────────────────────
    /// Fill the body area with black.
    void blank();
    /**
     * Show the idle / standby screen appropriate for the current BLE state.
     * @param state      Current BLE connection state.
     * @param pairingMsg Passcode or hint string shown during pairing; may be nullptr.
     */
    void standby(conn_state_def state, const char* pairingMsg = nullptr);
    void showNotification(notification_def const& notification);
    void showLoraMessage(MeshMessage const& msg);
    void showPositionMessage(MeshPosition const& pos);
    void showNodeInfoMessage(MeshUser const& user);

    // ── Accessors ─────────────────────────────────────────────────────────
    uint8_t width()  const { return _tft.width();  }
    uint8_t height() const { return _tft.height(); }

private:
    void drawIcon(uint16_t x, uint16_t y, const uint8_t* xbm,
                  uint16_t color = TFT::Color::WHITE);

    TFT  _tft;
    bool _callState = false;  ///< last drawn call state — prevents redundant redraws
};
