# heltec_ancs — Host Unit Tests

Pure-C++ tests that run on macOS/Linux without any hardware or ESP-IDF toolchain.
Uses the [Unity](https://github.com/ThrowTheSwitch/Unity) test framework (fetched
automatically by CMake FetchContent).

`test_mesh_crypto` requires **mbedtls**.  Two sources are supported, checked in
order:

1. **ESP-IDF bundled mbedtls** (preferred) — automatically built from source when
   `IDF_PATH` is set.  Guarantees the exact same mbedtls version the firmware
   links against.
2. **System mbedtls** (fallback) — used when `IDF_PATH` is not set:
   ```bash
   brew install mbedtls             # macOS
   apt-get install libmbedtls-dev   # Debian/Ubuntu
   ```

## Structure

```
test/
  CMakeLists.txt            # standalone CMake project, fetches Unity
  stubs/                    # minimal ESP-IDF / FreeRTOS header shims
    freertos/
      FreeRTOS.h            # TickType_t, pdMS_TO_TICKS, configTICK_RATE_HZ
      portmacro.h           # portMUX_TYPE, portENTER/EXIT_CRITICAL
      semphr.h              # SemaphoreHandle_t stub
      queue.h               # QueueHandle_t stub
      task.h                # TaskHandle_t, TimerHandle_t, vTaskDelay …
    esp_log.h               # ESP_LOGI/W/E/D → no-ops
    nvs_flash.h             # nvs_open → ESP_ERR_NVS_NOT_FOUND stub
    nvs.h                   # re-exports nvs_flash.h stubs
  test_mesh_codec.cxx       # 41 tests — varint, zigzag, all en/decoders
  test_mesh_crypto.cxx      # 47 tests — AES-CTR, X25519, PKC DM, OTA frames
  test_applist.cxx          # 27 tests — built-in lookup, custom entry mgmt
  test_notification_def.cxx # 22 tests — notification_def struct logic
```

## Building and running

With ESP-IDF installed (recommended — uses the same mbedtls as the firmware):
```bash
cd test
cmake -B build -DCMAKE_BUILD_TYPE=Debug    # picks up $IDF_PATH automatically
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

Without ESP-IDF (falls back to system mbedtls):
```bash
brew install mbedtls                        # macOS, one-time
cd test
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

Run a specific suite verbosely:
```bash
./build/test_mesh_crypto
./build/test_mesh_codec
./build/test_applist
./build/test_notification_def
```

## What is tested

### `test_mesh_crypto` (47 tests) — primary crypto focus

Tests `mesh_crypto.cxx` — the platform-free layer holding all Meshtastic
cryptographic primitives (mbedtls only, no ESP-IDF). The firmware delegates
every crypto call here; these tests verify correctness against published standards.

| Group | Count | Reference |
|---|---|---|
| `mc_buildNonce` | 6 | Meshtastic `CryptoEngine::initCounter()` |
| `mc_aes128ctr` | 6 | NIST SP 800-38A Appendix F.5.1 |
| `mc_aes256ctr` | 3 | NIST SP 800-38A Appendix F.5.5 |
| `mc_channelCrypt` | 6 | Meshtastic DEFAULT_PSK round-trips |
| `mc_x25519PublicKey` | 4 | RFC 7748 §6.1 known key derivations |
| `mc_x25519SharedSecret` | 5 | RFC 7748 §6.1 known shared secrets |
| `mc_pkcCrypt` | 5 | PKC DM encrypt/decrypt round-trips |
| Full OTA frame | 2 | Channel text message + PKC DM end-to-end |
| Channel hash | 3 | Verify DEFAULT_CHAN_HASH = 0x08 |

#### Key regression tests for known PKC bugs

**RFC 7748 Alice→Bob shared secret**
`test_x25519_shared_secret_alice_to_bob` pins the exact 32-byte output of
`mc_x25519SharedSecret` to the RFC 7748 §6.1 vector. Wrong LE↔BE byte reversal
or wrong mbedtls ECDH call path causes a concrete hex-diff failure.

**ECDH commutativity**
`test_x25519_shared_secret_commutativity` verifies `ECDH(a,B) == ECDH(b,A)`.
If mbedtls Curve25519 behaves differently from Arduino Crypto `Curve25519::eval()`
for the Meshtastic PKC use-case, this test catches it.

**Full PKC OTA frame round-trip**
`test_full_ota_pkc_direct_message` constructs a complete Meshtastic OTA packet
(16-byte header + AES-256-CTR Data proto, `chanHash=0x00`), encrypts as Alice,
decrypts as Bob, and parses the result. Highest-fidelity test of the PKC path
without hardware.

**Channel hash = 0x08**
`test_channel_hash_combined` verifies `xorHash("LongFast") ^ xorHash(DEFAULT_PSK)`
equals `0x08`. PSK or channel-name drift here explains decrypt failures on hardware.

### `test_mesh_codec` (41 tests)

Tests `mesh_codec.cxx` — platform-free Meshtastic protobuf codec.

- Field-number regression: `time` at field 4, `sats_in_view` at field 14
- `request_id` at tag `0x35` (field 6), never `0x3D` (field 7)

### `test_applist` (27 tests)

ApplicationList: built-in lookups, custom add/remove, overflow and duplicate guards.

### `test_notification_def` (22 tests)

notification_def struct: defaults, `reset()`, `isCall()`, ATTR_* bitmasks, buffer sizes.
