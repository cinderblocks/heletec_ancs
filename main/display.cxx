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

#include "display.h"

#include "applist.h"
#include "bitmaps.h"
#include "fonts.h"
#include "notificationservice.h"  // full notification_def type

#include <cinttypes>
#include <cstring>
#include <ctime>
#include <esp_log.h>

static const char* TAG = "display";

// ── Constructor ───────────────────────────────────────────────────────────

Display::Display(int8_t cs, int8_t rst, int8_t dc,
                 int8_t sclk, int8_t mosi, int8_t led, int8_t vext)
: _tft(cs, rst, dc, sclk, mosi, led, vext)
{ }

// ── init ──────────────────────────────────────────────────────────────────

void Display::init()
{
    _tft.init();
    blank();
    _tft.drawXbm(16, 10, 128, 64, Bitmaps::Gnu_128x64, TFT::Color::BLUE);
    ESP_LOGI(TAG, "TFT initialised");
}

// ── drawIcon (private) ────────────────────────────────────────────────────

void Display::drawIcon(uint16_t x, uint16_t y, const uint8_t* xbm, uint16_t color)
{
    _tft.drawXbm(x, y, 16, 16, xbm, color, HEADER_COLOR);
}

// ── Header bar methods ────────────────────────────────────────────────────

void Display::paintHeaderBackground()
{
    _tft.fillRectangle(0, 0, _tft.width(), HEADER_HEIGHT, HEADER_COLOR);
}

void Display::showBLEState(conn_state_def state)
{
    switch (state) {
        case BLE_CONNECTED:
            drawIcon(_tft.width() - 33, 1, Bitmaps::BluetoothRound);
            break;
        case BLE_SERVER_CONNECTED:
        case BLE_PAIRING:
            drawIcon(_tft.width() - 33, 1, Bitmaps::Mqtt);
            break;
        case BLE_DISCONNECTED:
            drawIcon(_tft.width() - 33, 1, Bitmaps::Bluetooth);
            break;
    }
}

void Display::showBatteryLevel(uint8_t percent, bool isCharging)
{
    if (isCharging) {
        // USB VBUS present — TP4054 actively charging.  Yellow lightning bolt;
        // the ADC voltage reading is unreliable while the charger is active.
        drawIcon(_tft.width() - 16, 1, Bitmaps::Battery_Charging, TFT::Color::YELLOW);
        return;
    }
    if (percent > 75) {
        drawIcon(_tft.width() - 16, 1, Bitmaps::Battery_100);
    } else if (percent > 50) {
        drawIcon(_tft.width() - 16, 1, Bitmaps::Battery_66);
    } else if (percent > 25) {
        drawIcon(_tft.width() - 16, 1, Bitmaps::Battery_33);
    } else {
        drawIcon(_tft.width() - 16, 1, Bitmaps::Battery_0, TFT::Color::RED);
    }
}

void Display::showCallState(bool active)
{
    if (active == _callState) { return; }  // idempotent — skip redundant SPI write
    _callState = active;
    if (active) {
        drawIcon(_tft.width() - 84, 1, Bitmaps::PhoneCall, TFT::Color::GREEN);
    } else {
        _tft.fillRectangle(_tft.width() - 84, 1, 16, 16, HEADER_COLOR);
    }
}

void Display::showGpsState(bool fixed)
{
    if (fixed) {
        drawIcon(_tft.width() - 50, 1, Bitmaps::GPS, TFT::Color::WHITE);
    } else {
        _tft.fillRectangle(_tft.width() - 50, 1, 16, 16, HEADER_COLOR);
    }
}

void Display::showLoraState(bool connected)
{
    if (connected) {
        drawIcon(_tft.width() - 67, 1, Bitmaps::LoRaMesh, TFT::Color::WHITE);
    } else {
        _tft.fillRectangle(_tft.width() - 67, 1, 16, 16, HEADER_COLOR);
    }
}

void Display::showTime(const char* ts)
{
    if (ts == nullptr || ts[0] == '\0') { return; }
    _tft.drawStr(0, 6, ts, Font_7x10, TFT::Color::WHITE, HEADER_COLOR);
}

// ── Body area methods ─────────────────────────────────────────────────────

void Display::blank()
{
    _tft.fillRectangle(0, HEADER_HEIGHT, _tft.width(), _tft.height() - HEADER_HEIGHT,
                       TFT::Color::BLACK);
}

void Display::standby(conn_state_def state, const char* pairingMsg)
{
    blank();
    switch (state) {
        case BLE_SERVER_CONNECTED:
            _tft.drawStr(0, 62, "Connected", Font_11x18, TFT::Color::WHITE);
            break;
        case BLE_CONNECTED:
            _tft.drawStr(0, 62, "Standby", Font_11x18, TFT::Color::WHITE);
            break;
        case BLE_PAIRING: {
            const char* msg = (pairingMsg != nullptr) ? pairingMsg : "";
            // "Forget on iOS" = bond-mismatch hint; numeric code = passkey display.
            if (strncmp(msg, "Forget", 6) == 0) {
                _tft.drawStr(0, 28, "Pairing failed:", Font_7x10, TFT::Color::WHITE);
                _tft.drawStr(0, 42, "iOS Settings >",  Font_7x10, TFT::Color::WHITE);
                _tft.drawStr(0, 56, "Bluetooth >",     Font_7x10, TFT::Color::WHITE);
                _tft.drawStr(0, 70, "Forget device",   Font_7x10, TFT::Color::WHITE);
            } else {
                _tft.drawStr(0, 36, "Pairing",  Font_11x18, TFT::Color::WHITE);
                _tft.drawStr(0, 60, msg,         Font_11x18, TFT::Color::WHITE);
            }
            break;
        }
        case BLE_DISCONNECTED:
            _tft.drawStr(0, 62, "Disconnected", Font_11x18, TFT::Color::WHITE);
            break;
    }
}

void Display::showNotification(notification_def const& notification)
{
    char timestamp[8];
    struct tm timeinfo;
    localtime_r(&notification.time, &timeinfo);
    strftime(timestamp, sizeof(timestamp), "%R", &timeinfo);

    blank();
    _tft.fillRectangle(0, HEADER_HEIGHT, _tft.width(), _tft.height() - HEADER_HEIGHT,
                       TFT::Color::WHITE);
    _tft.drawStr(0, 21, AppList.getDisplayName(notification.bundleId),
                 Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    _tft.drawStr(_tft.width() - 38, 21, timestamp,
                 Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    _tft.drawStr(0, 44, notification.title,   Font_11x18, TFT::Color::BLACK, TFT::Color::WHITE);
    _tft.drawStr(0, 62, notification.message, Font_11x18, TFT::Color::BLACK, TFT::Color::WHITE);
}

void Display::showLoraMessage(MeshMessage const& msg)
{
    blank();

    // ── Subheader bar (y=20..31, 12 px) ──────────────────────────────────
    const uint16_t hdrColor = msg.isAlert ? TFT::Color::RED   : TFT::Color::CYAN;
    const uint16_t hdrText  = msg.isAlert ? TFT::Color::WHITE : TFT::Color::BLACK;
    const char*    hdrLabel = msg.isAlert ? "ALERT" : "MESH";
    _tft.fillRectangle(0, HEADER_HEIGHT, _tft.width(), 12, hdrColor);
    _tft.drawStr(0, 21, hdrLabel, Font_7x10, hdrText, hdrColor);

    // Sender: prefer short name; fall back to abbreviated node ID.
    char sender[12] = {};
    if (msg.shortName[0] != '\0') {
        strncpy(sender, msg.shortName, sizeof(sender) - 1);
    } else {
        snprintf(sender, sizeof(sender), "!%06" PRIx32, msg.fromNode & 0xFFFFFF);
    }

    // RSSI right-aligned in subheader.
    char rssiStr[12];
    snprintf(rssiStr, sizeof(rssiStr), "%ddB", static_cast<int>(msg.rssi));
    const uint16_t rssiX = _tft.width() - static_cast<uint16_t>(strlen(rssiStr) * 7);
    _tft.drawStr(rssiX, 21, rssiStr, Font_7x10, hdrText, hdrColor);

    const uint16_t senderX = rssiX - static_cast<uint16_t>(strlen(sender) * 7) - 4;
    if (static_cast<int16_t>(senderX) > 28)
        _tft.drawStr(senderX, 21, sender, Font_7x10, hdrText, hdrColor);

    // ── Message body (y=32..79) ───────────────────────────────────────────
    _tft.fillRectangle(0, 32, _tft.width(), _tft.height() - 32, TFT::Color::WHITE);

    static constexpr int CHARS_PER_LINE = 14;  // floor(160 / 11)
    const char*  text    = msg.text;
    const size_t textLen = strlen(text);

    char line1[CHARS_PER_LINE + 1] = {};
    char line2[CHARS_PER_LINE + 1] = {};

    if (textLen <= static_cast<size_t>(CHARS_PER_LINE)) {
        strncpy(line1, text, CHARS_PER_LINE);
    } else {
        int breakAt = CHARS_PER_LINE;
        for (int b = CHARS_PER_LINE; b >= CHARS_PER_LINE - 4 && b > 0; --b) {
            if (text[b] == ' ') { breakAt = b; break; }
        }
        strncpy(line1, text, breakAt);
        const char* rest = text + breakAt + (text[breakAt] == ' ' ? 1 : 0);
        strncpy(line2, rest, CHARS_PER_LINE);
    }

    _tft.drawStr(0, 33, line1, Font_11x18, TFT::Color::BLACK, TFT::Color::WHITE);
    if (line2[0] != '\0')
        _tft.drawStr(0, 52, line2, Font_11x18, TFT::Color::BLACK, TFT::Color::WHITE);

    // ── Signal quality footer ─────────────────────────────────────────────
    char snrStr[16];
    snprintf(snrStr, sizeof(snrStr), "SNR %.1fdB", static_cast<double>(msg.snr));
    _tft.drawStr(0, 70, snrStr, Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
}

void Display::showPositionMessage(MeshPosition const& pos)
{
    blank();
    _tft.fillRectangle(0, HEADER_HEIGHT, _tft.width(), _tft.height() - HEADER_HEIGHT,
                       TFT::Color::WHITE);

    char nodeStr[12];
    snprintf(nodeStr, sizeof(nodeStr), "%08" PRIx32, pos.fromNode);
    _tft.drawStr(0, 21, "Pos", Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    _tft.drawStr(_tft.width() - 56, 21, nodeStr, Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);

    char latStr[20], lonStr[20];
    snprintf(latStr, sizeof(latStr), "%.4f", static_cast<double>(pos.lat_i) / 1e7);
    snprintf(lonStr, sizeof(lonStr), "%.4f", static_cast<double>(pos.lon_i) / 1e7);
    _tft.drawStr(0, 34, latStr, Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    _tft.drawStr(0, 46, lonStr, Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);

    char altStr[24];
    snprintf(altStr, sizeof(altStr), "%dm  %usat",
             static_cast<int>(pos.alt_m), static_cast<unsigned>(pos.sats));
    _tft.drawStr(0, 58, altStr, Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);

    char sigStr[24];
    snprintf(sigStr, sizeof(sigStr), "%d dBm  %.1f dB",
             static_cast<int>(pos.rssi), static_cast<double>(pos.snr));
    _tft.drawStr(0, 70, sigStr, Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
}

void Display::showNodeInfoMessage(MeshUser const& user)
{
    blank();
    _tft.fillRectangle(0, HEADER_HEIGHT, _tft.width(), _tft.height() - HEADER_HEIGHT,
                       TFT::Color::WHITE);

    _tft.drawStr(0, 21, "Node", Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    _tft.drawStr(_tft.width() - 28, 21, user.shortName,
                 Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);

    _tft.drawStr(0, 34, user.id,       Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
    _tft.drawStr(0, 48, user.longName, Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);

    char sigStr[24];
    snprintf(sigStr, sizeof(sigStr), "%d dBm  %.1f dB",
             static_cast<int>(user.rssi), static_cast<double>(user.snr));
    _tft.drawStr(0, 62, sigStr, Font_7x10, TFT::Color::BLACK, TFT::Color::WHITE);
}
