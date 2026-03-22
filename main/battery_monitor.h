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

#include <esp_adc/adc_oneshot.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include <cstdint>

/**
 * BatteryMonitor — ADC-based LiPo battery monitor for the Heltec Wireless
 * Tracker 1.1.
 *
 * A P-FET controlled by GPIO2 (ADC_CTRL) enables a 100K/100K voltage-divider
 * between VBAT and GND; GPIO1 (VBAT_READ = ADC1_CH0) reads the mid-point.
 * The divider is only enabled during the 100 ms settle + read window to
 * minimise quiescent drain (~10 µA per 100K resistor while enabled).
 *
 * update() serialises concurrent callers via an internal mutex; it MUST NOT
 * be called from an ISR or the timer-service task (it calls vTaskDelay).
 * The periodic timer only posts a notification bit to the draw task — the
 * actual ADC work always runs in the draw task.
 *
 * Thread-safety: init() must be called once before any task uses level(),
 * voltage(), or isCharging().  After that, those accessors are read-only and
 * safe from any task without additional locking.
 */
class BatteryMonitor {
public:
    // ── Named ADC calibration constants (Heltec Wireless Tracker 1.1) ─────
    /// ADC raw count corresponding to 3.20 V (LiPo fully discharged, cutoff).
    static constexpr float kAdcRawAtEmpty      = 680.0f;
    /// ADC raw count corresponding to 4.20 V (LiPo fully charged).
    static constexpr float kAdcRawAtFull       = 1023.0f;
    /// Span between empty and full raw counts (used for normalisation).
    static constexpr float kAdcRawSpan         = kAdcRawAtFull - kAdcRawAtEmpty;
    /// Battery voltage at the empty (0 %) endpoint.
    static constexpr float kVoltageAtEmpty     = 3.20f;
    /// Battery voltage at the full (100 %) endpoint.
    static constexpr float kVoltageAtFull      = 4.20f;
    /// Voltage threshold above which the TP4054 charger is considered active.
    /// The TP4054 holds VBAT at 4.20 V during CC/CV charging — measurably
    /// above the resting "full" voltage (~4.10–4.15 V).
    /// Same threshold Meshtastic firmware uses as its isVbusIn() fallback.
    static constexpr float kChargingThresholdV = 4.15f;

    BatteryMonitor();
    ~BatteryMonitor();

    /**
     * Initialise ADC hardware and GPIO2 (ADC_CTRL).
     * Does NOT start the periodic timer — call startTimer() after the draw
     * task has been created so the timer has a valid task handle to notify.
     * Performs an immediate blocking read so level()/voltage() return sensible
     * values before the first timer tick.
     */
    void init();

    /**
     * Start the 30-second periodic battery timer.
     * @param drawTask    Task to notify on periodic battery reads.
     * @param batteryBit  Task-notification bit for periodic reads.
     * @param chargingBit Task-notification bit when charging state changes.
     */
    void startTimer(TaskHandle_t drawTask, uint32_t batteryBit, uint32_t chargingBit);

    /// Cached battery percentage (0–100).  Updated by update().
    uint8_t level()     const { return _level;      }
    /// Cached battery voltage in volts.  Updated by update().
    float   voltage()   const { return _voltage;    }
    /// True when the TP4054 charger is believed to be active.
    bool    isCharging() const { return _isCharging; }

    /**
     * Trigger a fresh ADC reading and update all cached values.
     * Blocks ~100 ms while the voltage-divider rail settles.
     * Thread-safe (serialised by internal mutex).
     */
    void update();

private:
    static void _timerCb(TimerHandle_t xTimer);

    /**
     * Convert a measured battery voltage to a percentage using a
     * piecewise-linear approximation of the LiPo discharge curve.
     * More accurate than a single linear mapping because the LiPo curve
     * is flat in the 50–90 % region and steep at both extremes.
     */
    static uint8_t _voltageToPct(float voltage);

    adc_oneshot_unit_handle_t _adcHandle   = nullptr;
    SemaphoreHandle_t         _mutex       = nullptr;
    TimerHandle_t             _timer       = nullptr;
    TaskHandle_t              _drawTask    = nullptr;
    uint32_t                  _batteryBit  = 0;
    uint32_t                  _chargingBit = 0;

    uint8_t  _level      = 0;
    float    _voltage    = 0.0f;
    bool     _isCharging = false;
};
