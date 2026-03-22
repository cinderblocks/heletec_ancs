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

#include "battery_monitor.h"

#include <algorithm>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/task.h>

static const char* TAG = "battery";

// GPIO2: P-FET gate — pull-up enables the 100K/100K voltage divider.
static constexpr gpio_num_t   ADC_CTRL_PIN = GPIO_NUM_2;
// GPIO1 = ADC1_CH0: reads the divider mid-point.
static constexpr adc_channel_t VBAT_ADC_CH = ADC_CHANNEL_0;

// ── Constructor / Destructor ──────────────────────────────────────────────

BatteryMonitor::BatteryMonitor()
{
    _mutex = xSemaphoreCreateMutex();
    if (_mutex == nullptr) {
        // OOM at construction — log deferred; proceed unprotected.
        ESP_EARLY_LOGE(TAG, "xSemaphoreCreateMutex failed — ADC reads unprotected");
    }
}

BatteryMonitor::~BatteryMonitor()
{
    if (_timer)      { xTimerDelete(_timer, 0);             _timer      = nullptr; }
    if (_adcHandle)  { adc_oneshot_del_unit(_adcHandle);    _adcHandle  = nullptr; }
    if (_mutex)      { vSemaphoreDelete(_mutex);             _mutex      = nullptr; }
}

// ── init ──────────────────────────────────────────────────────────────────

void BatteryMonitor::init()
{
    // ADC_CTRL (GPIO2) — initially pull-down (divider disabled, low quiescent draw)
    const gpio_config_t adc_ctrl_conf = {
        .pin_bit_mask  = 1ULL << ADC_CTRL_PIN,
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_ENABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&adc_ctrl_conf);

    // ADC1 oneshot unit — reused for every update() call.
    const adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&adc_cfg, &_adcHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit: %s", esp_err_to_name(err));
    } else {
        const adc_oneshot_chan_cfg_t chan_cfg = {
            .atten    = ADC_ATTEN_DB_12,   // 0–3.9 V input range
            .bitwidth = ADC_BITWIDTH_12,    // 0–4095 raw output
        };
        err = adc_oneshot_config_channel(_adcHandle, VBAT_ADC_CH, &chan_cfg);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "adc_oneshot_config_channel: %s", esp_err_to_name(err));
    }

    // Perform an immediate reading so level()/voltage() are valid before the
    // draw task starts (the draw task reads _battery.level() on startup).
    update();
}

// ── startTimer ───────────────────────────────────────────────────────────

void BatteryMonitor::startTimer(TaskHandle_t drawTask, uint32_t batteryBit, uint32_t chargingBit)
{
    _drawTask    = drawTask;
    _batteryBit  = batteryBit;
    _chargingBit = chargingBit;

    _timer = xTimerCreate("Battery", pdMS_TO_TICKS(30000), pdTRUE, this, _timerCb);
    if (_timer == nullptr) {
        ESP_LOGW(TAG, "Failed to create battery timer");
    } else if (xTimerStart(_timer, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to start battery timer");
    }
}

/* static */
void BatteryMonitor::_timerCb(TimerHandle_t xTimer)
{
    // Post a notification bit to the draw task — the actual 100 ms blocking
    // ADC read happens in the draw task, not here in the timer service task.
    auto* self = static_cast<BatteryMonitor*>(pvTimerGetTimerID(xTimer));
    if (self->_drawTask && self->_batteryBit) {
        xTaskNotify(self->_drawTask, self->_batteryBit, eSetBits);
    }
}

// ── _voltageToPct ─────────────────────────────────────────────────────────
// Piecewise-linear LiPo discharge curve.
//
// Standard LiPo cells have a flat discharge plateau from roughly 4.10 V to
// 3.70 V (representing ~5 % to ~55 % capacity), followed by a steep drop-off
// below 3.70 V.  A single linear formula over-reports capacity during the
// plateau and under-reports near the knee.  Using breakpoints taken from
// empirical discharge data at a typical 0.5 C rate gives ~5 % accuracy.
//
// Breakpoints were compared against the Meshtastic firmware's analogRead()
// table for boards using the same TP4054 + 100K/100K divider topology.

/* static */
uint8_t BatteryMonitor::_voltageToPct(float voltage)
{
    struct Point { float v; uint8_t pct; };
    static constexpr Point kCurve[] = {
        { 4.20f, 100 },
        { 4.10f,  95 },
        { 4.00f,  90 },
        { 3.90f,  80 },
        { 3.80f,  70 },
        { 3.70f,  55 },
        { 3.60f,  35 },
        { 3.50f,  20 },
        { 3.40f,  10 },
        { 3.30f,   5 },
        { 3.20f,   0 },
    };
    static constexpr size_t kN = sizeof(kCurve) / sizeof(kCurve[0]);

    if (voltage >= kCurve[0].v)    return 100;
    if (voltage <= kCurve[kN-1].v) return 0;

    for (size_t i = 0; i + 1 < kN; ++i) {
        if (voltage <= kCurve[i].v && voltage >= kCurve[i+1].v) {
            const float t = (voltage - kCurve[i+1].v) / (kCurve[i].v - kCurve[i+1].v);
            const float pct = static_cast<float>(kCurve[i+1].pct)
                            + t * static_cast<float>(kCurve[i].pct - kCurve[i+1].pct);
            return static_cast<uint8_t>(pct);
        }
    }
    return 0;
}

// ── update ────────────────────────────────────────────────────────────────
// Enable the voltage divider, average 4 ADC samples (reduces noise), compute
// voltage and percentage, then update cached state.
//
// Must NOT be called from an ISR or the timer-service task (vTaskDelay blocks).

void BatteryMonitor::update()
{
    // Serialise concurrent callers (draw task + diag BLE read handler).
    // Null-guard: if OOM prevented mutex creation we proceed unprotected.
    if (_mutex) xSemaphoreTake(_mutex, portMAX_DELAY);

    // Enable voltage divider — pull ADC_CTRL HIGH to turn on the P-FET.
    gpio_set_pull_mode(ADC_CTRL_PIN, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(100));  // wait for rail to settle

    // Average 4 samples to reduce ADC noise.
    int rawSum = 0;
    for (int i = 0; i < 4; ++i) {
        int raw = 0;
        adc_oneshot_read(_adcHandle, VBAT_ADC_CH, &raw);
        rawSum += raw;
    }

    // Disable voltage divider.
    gpio_set_pull_mode(ADC_CTRL_PIN, GPIO_PULLDOWN_ONLY);

    if (_mutex) xSemaphoreGive(_mutex);

    // ── Compute voltage from raw ADC ────────────────────────────────────
    // Linear interpolation between the two calibration endpoints.
    //   raw kAdcRawAtEmpty (680) → kVoltageAtEmpty (3.20 V)
    //   raw kAdcRawAtFull (1023) → kVoltageAtFull  (4.20 V)
    const float rawAvg  = static_cast<float>(rawSum) / 4.0f;
    const float pct01   = std::clamp((rawAvg - kAdcRawAtEmpty) / kAdcRawSpan, 0.f, 1.f);
    const float voltage = kVoltageAtEmpty + pct01 * (kVoltageAtFull - kVoltageAtEmpty);

    // ── Non-linear LiPo curve → percentage ─────────────────────────────
    const uint8_t level = _voltageToPct(voltage);

    // ── Charging state detection ────────────────────────────────────────
    // The TP4054 CHRG pin is wired only to the onboard LED, not to any GPIO.
    // Fallback: battery voltage > 4.15 V means the TP4054 is actively in CC/CV.
    const bool nowCharging = (voltage > kChargingThresholdV);

    _voltage = voltage;
    _level   = level;

    if (nowCharging != _isCharging) {
        _isCharging = nowCharging;
        if (_drawTask && _chargingBit) {
            xTaskNotify(_drawTask, _chargingBit, eSetBits);
        }
        ESP_LOGI(TAG, "Charging state: %s", _isCharging ? "CHARGING" : "NOT CHARGING");
    }

    ESP_LOGD(TAG, "Battery: raw=%.0f  V=%.2fV  level=%u%%  charging=%d",
             static_cast<double>(rawAvg), static_cast<double>(voltage),
             level, static_cast<int>(_isCharging));
}
