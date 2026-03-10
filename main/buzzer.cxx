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

#include "buzzer.h"
#include "hardware.h"   // Hardware::BUZZER (GPIO 45)
#include "sdkconfig.h"

#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "buzzer";

// ── Static member definitions ─────────────────────────────────────────────
// constexpr arrays defined in the header require out-of-line definitions
// in C++14 when ODR-used (passed by pointer to _playTask).
constexpr Buzzer::Note Buzzer::NOTIF_MELODY[];
constexpr Buzzer::Note Buzzer::RING_MELODY[];

volatile TaskHandle_t Buzzer::_taskHandle = nullptr;

// ── _silence ──────────────────────────────────────────────────────────────
void Buzzer::_silence()
{
    ledc_set_duty(SPEED_MODE, CHANNEL, 0);
    ledc_update_duty(SPEED_MODE, CHANNEL);
}

// ── _playTask ─────────────────────────────────────────────────────────────
// Iterates the note table, driving LEDC at each note's frequency for its
// duration, then self-deletes.  Runs at priority 2 (below draw task at 3).
void Buzzer::_playTask(void* arg)
{
    const PlayArgs* pa    = static_cast<const PlayArgs*>(arg);
    const Note*     notes = pa->notes;
    const size_t    count = pa->count;

    for (size_t i = 0; i < count; i++)
    {
        if (notes[i].freq == 0)
        {
            // Silence interval — duty 0, then delay.
            ledc_set_duty(SPEED_MODE, CHANNEL, 0);
            ledc_update_duty(SPEED_MODE, CHANNEL);
        }
        else
        {
            // Change frequency, then set 50 % duty to drive the buzzer.
            ledc_set_freq(SPEED_MODE, TIMER_NUM, notes[i].freq);
            ledc_set_duty(SPEED_MODE, CHANNEL, DUTY_50PCT);
            ledc_update_duty(SPEED_MODE, CHANNEL);
        }
        vTaskDelay(pdMS_TO_TICKS(notes[i].durationMs));
    }

    _silence();
    _taskHandle = nullptr;
    vTaskDelete(nullptr);
}

// ── init ──────────────────────────────────────────────────────────────────
void Buzzer::init()
{
#if !CONFIG_BUZZER_ENABLED
    // Ensure the pin is driven low so it doesn't float and click.
    gpio_set_direction(static_cast<gpio_num_t>(Hardware::BUZZER), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(Hardware::BUZZER), 0);
    ESP_LOGI(TAG, "Buzzer disabled by config — GPIO%d driven low", Hardware::BUZZER);
    return;
#endif

    // Configure LEDC timer.  Use zero-init + explicit field assignment to
    // silence -Wmissing-field-initializers on IDF v5.5 struct additions.
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode      = SPEED_MODE;
    timer_cfg.duty_resolution = static_cast<ledc_timer_bit_t>(DUTY_RES);
    timer_cfg.timer_num       = TIMER_NUM;
    timer_cfg.freq_hz         = 1000;   // initial freq; overridden per note
    timer_cfg.clk_cfg         = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config: %s", esp_err_to_name(err));
        return;
    }

    // Configure LEDC channel — initially silent (duty = 0).
    ledc_channel_config_t chan_cfg = {};
    chan_cfg.gpio_num  = Hardware::BUZZER;
    chan_cfg.speed_mode = SPEED_MODE;
    chan_cfg.channel   = CHANNEL;
    chan_cfg.intr_type = LEDC_INTR_DISABLE;
    chan_cfg.timer_sel = TIMER_NUM;
    chan_cfg.duty      = 0;
    chan_cfg.hpoint    = 0;

    err = ledc_channel_config(&chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Buzzer ready on GPIO%d (LEDC timer%d ch%d, 10-bit)",
             Hardware::BUZZER, (int)TIMER_NUM, (int)CHANNEL);
}

// ── stop ─────────────────────────────────────────────────────────────────
void Buzzer::stop()
{
#if !CONFIG_BUZZER_ENABLED
    return;
#endif
    TaskHandle_t h = _taskHandle;
    if (h != nullptr) {
        _taskHandle = nullptr;
        vTaskDelete(h);
    }
    _silence();
}

// ── play ─────────────────────────────────────────────────────────────────
void Buzzer::play(bool isCall)
{
#if !CONFIG_BUZZER_ENABLED
    return;
#endif
    // Stop any melody that might still be running (defensive — under normal
    // operation the 5 s melody always finishes within the 15 s display window).
    stop();

    // Static args — safe to pass as a pointer because _playTask reads them
    // before this function returns (the task starts immediately and the
    // static lifetime outlasts any task).
    static const PlayArgs notifArgs{ NOTIF_MELODY, sizeof(NOTIF_MELODY) / sizeof(NOTIF_MELODY[0]) };
    static const PlayArgs ringArgs { RING_MELODY,  sizeof(RING_MELODY)  / sizeof(RING_MELODY[0])  };

    const PlayArgs* args = isCall ? &ringArgs : &notifArgs;

    // Pin to core 0 — the same core as the draw task (which is pinned to core 0
    // via xTaskCreatePinnedToCore in Hardware::begin).  Both tasks share one core,
    // so their accesses to the static _taskHandle are strictly sequential and the
    // "nullptr written by _playTask then read stale on core 0" cache-coherency race
    // that exists with xTaskCreate (no affinity, may land on core 1) is eliminated.
    //
    // Use a plain local for the out-parameter: xTaskCreatePinnedToCore takes
    // TaskHandle_t* (non-volatile), so we cannot pass &_taskHandle directly.
    // The assignment into the volatile member happens after the call.
    TaskHandle_t handle = nullptr;
    BaseType_t rc = xTaskCreatePinnedToCore(
        _playTask,
        "buzzer",
        /*stack*/ 2048,
        const_cast<PlayArgs*>(args),
        /*priority*/ 2,   // below draw task (3), above idle (0/1)
        &handle,
        /*core*/ 0);

    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(buzzer) failed");
        _taskHandle = nullptr;
    } else {
        _taskHandle = handle;
    }
}
