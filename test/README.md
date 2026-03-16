# heltec_ancs — Host Unit Tests

Pure-C++ tests that run on macOS/Linux without any hardware or ESP-IDF toolchain.
Uses the [Unity](https://github.com/ThrowTheSwitch/Unity) test framework (fetched
automatically by CMake FetchContent).

## Structure

```
test/
  CMakeLists.txt            # standalone CMake project, fetches Unity
  stubs/                    # minimal ESP-IDF / FreeRTOS header shims
    freertos/
      FreeRTOS.h            # TickType_t, BaseType_t, configTICK_RATE_HZ
      portmacro.h           # portMUX_TYPE, portENTER/EXIT_CRITICAL
      semphr.h              # SemaphoreHandle_t, xSemaphoreCreateMutex …
      queue.h               # QueueHandle_t, xQueueCreate/Send/Receive …
      task.h                # TaskHandle_t, TimerHandle_t, vTaskDelay …
    esp_log.h               # ESP_LOGI/W/E/D → no-op macros
    nvs_flash.h             # nvs_flash_init, nvs_open → stub (NOT_FOUND)
    nvs.h                   # re-exports nvs_flash.h stubs
  test_mesh_codec.cxx       # 41 tests — varint, zigzag, all en/decoders
  test_applist.cxx          # 27 tests — built-in lookup, custom entry management
  test_notification_def.cxx # 22 tests — notification_def struct logic
```

## Building and running

```bash
cd test
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

Or run individual test binaries directly for verbose output:

```bash
./build/test_mesh_codec
./build/test_applist
./build/test_notification_def
```

## What is tested

### `test_mesh_codec` (primary focus)

Tests `mesh_codec.cxx` — the platform-free Meshtastic protobuf codec that
contains all the logic previously inlined inside the `LoRa` class.

| Group | Tests |
|---|---|
| `mc_pbVarint` | 0, 1, 127, 128, 300, UINT32_MAX |
| `mc_pbZigzag` | 0, ±1, ±2, INT32_MIN, INT32_MAX |
| `mc_parseData` | portnum, payload, want_response, truncated, empty |
| `mc_parsePosition` | **field-number regressions** (see below) |
| `mc_parseUser` | id, long_name, public_key 32B / wrong-length, truncation |
| Encode round-trips | Data→Position, Telemetry byte layout |
| `mc_encodeData` | **request_id field 6 (0x35) not field 7 (0x3D)** |

#### Key regression tests for known bugs

**Position — `time` at field 4, not field 9**

`test_parse_position_time_from_field4_not_field9` crafts a Position proto
with the correct time at field 4 (`sfixed32`, tag `0x25`), a `pos_flags`
varint at field 9 (tag `0x48`), a `google_plus_code` string at field 7 (tag
`0x3A` — must be **skipped** not stored as sats), and sats at field 14 (tag
`0x70`).

`test_parse_position_field9_sfixed32_not_stored_as_time` encodes time at the
*old wrong* field 9 sfixed32 position (tag `0x4D`) and verifies the decoder
**does not** store it in `pos.unixTime`.

**Data — `request_id` at field 6 (tag `0x35`), never field 7 (`0x3D`)**

`test_encode_data_request_id_uses_field6_tag_0x35` scans the encoded bytes
for tag `0x35` (✓ required) and `0x3D` (✗ must be absent).

### `test_applist`

Tests the pure logic of `ApplicationList` using in-memory state only (NVS
stubs return `NOT_FOUND` so `_loadFromNvs` exits immediately):

- Built-in bundle ID lookup (`isAllowedApplication`, `getApplicationId`)
- Display name resolution by enum and by bundle ID string
- Custom entry add / remove / reset
- Duplicate, built-in-override, and overflow guards

### `test_notification_def`

Tests `notification_def` — the POD struct that carries notification data
between BLE callbacks and the draw task:

- Default field values (all zero/empty/false)
- `reset()` clears string and flag fields but preserves `key`
- `isCall()` returns true only for `APP_PHONE` and `APP_FACETIME`
- `ATTR_*` bitmask arithmetic and tracking logic
- Buffer size guarantees (title=64, message=128, bundleId=64)
