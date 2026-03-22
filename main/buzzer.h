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
 * Call play(type) when a notification, call, or Meshtastic message arrives.
 * Call stop() to silence immediately (e.g. on BLE disconnect).
 *
 * Three melodies, each ≤ 5 000 ms:
 *   NOTIFICATION → ascending arpeggio (C6/E6/G6, 5 cycles, 5 000 ms)
 *   CALL         → double-ring ringtone (B5, 3 rings, 5 000 ms)
 *   LORA         → walkie-talkie "over" chirp (G5/A5, 2 × 1 200 ms, 2 400 ms)
 *                  Short and distinctive — sounds like radio comms, not a phone.
 *
 * Playback runs in a dedicated low-priority FreeRTOS task so the draw
 * task is never blocked.
 */
class Buzzer
{
public:
    /// Sound type selector for play().
    enum class SoundType : uint8_t {
        NOTIFICATION = 0,  ///< BLE notification arpeggio
        CALL         = 1,  ///< incoming call ringtone
        LORA         = 2,  ///< Meshtastic channel message chirp
        ALERT        = 3,  ///< Meshtastic emergency alert — rapid triple-pip alarm
    };

    /// Configure the LEDC peripheral.  Must be called before play() or stop().
    static void init();

    /// Start a melody for the given sound type.  Stops any current playback first.
    static void play(SoundType type = SoundType::NOTIFICATION);

    /// Convenience overload — true selects CALL, false selects NOTIFICATION.
    /// Kept for backward compatibility with existing callers.
    static void play(bool isCall) { play(isCall ? SoundType::CALL : SoundType::NOTIFICATION); }

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

    // Walkie-talkie "over" chirp: G5 double-tap → A5 confirmation, 2× = 2 400 ms.
    // G5 = 784 Hz, A5 = 880 Hz.  Lower and shorter than the BLE arpeggio so it
    // sounds like radio comms, not a phone notification.
    // Cycle: 150+60+150+60+300+480 = 1 200 ms × 2 = 2 400 ms
    static constexpr Note LORA_MELODY[] = {
        {784, 150}, {0, 60}, {784, 150}, {0, 60}, {880, 300}, {0, 480},
        {784, 150}, {0, 60}, {784, 150}, {0, 60}, {880, 300}, {0, 480},
    };
    // 2 × 1 200 = 2 400 ms ✓

    // Emergency alert alarm: rapid triple-pip at A6 (1760 Hz), 3 cycles = 2 580 ms.
    // Each cycle: 80+40+80+40+80+540 = 860 ms.  High pitch + rapid cadence makes
    // this immediately distinct from all other sounds — attention-getting even at
    // low buzzer volumes.
    static constexpr Note ALERT_MELODY[] = {
        {1760, 80}, {0, 40}, {1760, 80}, {0, 40}, {1760, 80}, {0, 540}, // cycle 1
        {1760, 80}, {0, 40}, {1760, 80}, {0, 40}, {1760, 80}, {0, 540}, // cycle 2
        {1760, 80}, {0, 40}, {1760, 80}, {0, 40}, {1760, 80}, {0, 540}, // cycle 3
    };
    // 3 × 860 = 2 580 ms ✓

    struct PlayArgs { const Note* notes; size_t count; };

    static void _silence();
    static void _playTask(void* arg);

    static volatile TaskHandle_t _taskHandle;
};

#endif // BUZZER_H_
