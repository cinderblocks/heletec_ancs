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

#ifndef BUZZER_H_
#define BUZZER_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>
#include <cstddef>

/**
 * Buzzer — passive buzzer driver (GPIO 45) via LEDC PWM.
 *
 * Call init() once from Hardware::begin().
 * Call play(isCall) when a notification or incoming call arrives.
 * Call stop() to silence immediately (e.g. on BLE disconnect).
 *
 * Two melodies, each exactly 5 000 ms:
 *   isCall = false → ascending notification arpeggio (C6/E6/G6, 5 cycles)
 *   isCall = true  → double-ring ringtone (B5, 3 rings)
 *
 * Playback runs in a dedicated low-priority FreeRTOS task so the draw
 * task is never blocked.  Since each notification display window is 15 s
 * and each melody is 5 s, the task always finishes naturally before the
 * next play() call — stop() is a defensive guard only.
 */
class Buzzer
{
public:
    /// Configure the LEDC peripheral.  Must be called before play() or stop().
    static void init();

    /// Start a 5-second melody.  Stops any current playback first.
    /// @param isCall  true → ringtone; false → notification arpeggio.
    static void play(bool isCall = false);

    /// Silence immediately and cancel any in-progress playback.
    static void stop();

private:
    struct Note { uint32_t freq; uint32_t durationMs; };

    // LEDC configuration
    static constexpr ledc_mode_t    SPEED_MODE  = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_t   TIMER_NUM   = LEDC_TIMER_0;
    static constexpr ledc_channel_t CHANNEL     = LEDC_CHANNEL_0;
    // 10-bit resolution → 50 % duty = 512, which drives a passive buzzer at max volume.
    static constexpr uint32_t       DUTY_RES    = LEDC_TIMER_10_BIT;
    static constexpr uint32_t       DUTY_50PCT  = (1u << (LEDC_TIMER_10_BIT - 1)); // 512

    // Notification arpeggio: ascending C6/E6/G6, 5 cycles = 5 000 ms
    // Cycle timing: 150+50+150+50+250+250 = 900 ms × 4  + 1 400 ms final = 5 000 ms
    static constexpr Note NOTIF_MELODY[] = {
        {1047, 150}, {0,  50}, {1319, 150}, {0,  50}, {1568, 250}, {0, 250},
        {1047, 150}, {0,  50}, {1319, 150}, {0,  50}, {1568, 250}, {0, 250},
        {1047, 150}, {0,  50}, {1319, 150}, {0,  50}, {1568, 250}, {0, 250},
        {1047, 150}, {0,  50}, {1319, 150}, {0,  50}, {1568, 250}, {0, 250},
        {1047, 150}, {0,  50}, {1319, 150}, {0,  50}, {1568, 250}, {0, 750},
    };
    // 4×900 + (150+50+150+50+250+750) = 3 600 + 1 400 = 5 000 ms ✓

    // Ringtone: double-buzz B5, 3 rings = 5 000 ms
    // Ring: 400+100+400+100 = 1 000 ms.  Pauses: 600/600/800.
    static constexpr Note RING_MELODY[] = {
        {988, 400}, {0, 100}, {988, 400}, {0, 100}, {0, 600},
        {988, 400}, {0, 100}, {988, 400}, {0, 100}, {0, 600},
        {988, 400}, {0, 100}, {988, 400}, {0, 100}, {0, 800},
    };
    // 1 600 + 1 600 + 1 800 = 5 000 ms ✓

    struct PlayArgs { const Note* notes; size_t count; };

    static void _silence();
    static void _playTask(void* arg);

    static volatile TaskHandle_t _taskHandle;
};

#endif // BUZZER_H_
