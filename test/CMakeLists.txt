cmake_minimum_required(VERSION 3.16)
project(heltec_ancs_tests CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD   11)

# ── Fetch Unity test framework ────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(unity)

# ── Paths ─────────────────────────────────────────────────────────────────
set(MAIN_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../main)
set(STUB_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/stubs)

# ── Common compile options ────────────────────────────────────────────────
# Warnings help catch real bugs; treat format issues as errors.
set(COMMON_FLAGS
    -Wall -Wextra -Wno-unused-parameter
)

# ── Helper: add a test executable ────────────────────────────────────────
# Usage: add_firmware_test(name src1 src2 ...)
function(add_firmware_test name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE
        ${MAIN_DIR}   # production headers (mesh_codec.h, applist.h, …)
        ${STUB_DIR}   # fake ESP-IDF / FreeRTOS headers (shadow real ones)
    )
    target_compile_options(${name} PRIVATE ${COMMON_FLAGS})
    # Unity is a C library; link it to every test binary.
    target_link_libraries(${name} PRIVATE unity)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

enable_testing()

# ── test_mesh_codec ───────────────────────────────────────────────────────
# Tests mc_pbVarint, mc_pbZigzag, mc_parseData, mc_parsePosition,
# mc_parseUser, and all the encode/decode round-trips.
# mesh_codec.cxx has ZERO platform dependencies — no stubs required.
add_firmware_test(test_mesh_codec
    test_mesh_codec.cxx
    ${MAIN_DIR}/mesh_codec.cxx
)

# ── test_applist ──────────────────────────────────────────────────────────
# Tests ApplicationList built-in lookups, custom entry management,
# display-name resolution, and duplicate/overflow guards.
# applist.cxx needs FreeRTOS semaphore + NVS + ESP log stubs — all provided
# by the headers in stubs/ which shadow the real ESP-IDF paths.
add_firmware_test(test_applist
    test_applist.cxx
    ${MAIN_DIR}/applist.cxx
)

# ── test_notification_def ─────────────────────────────────────────────────
# Tests the pure notification_def struct: default values, reset(), isCall(),
# and the ATTR_* bitmask constants.  No NimBLE or task code compiled.
# notificationservice.h forward-declares NimBLERemoteCharacteristic so no
# NimBLE stub is needed — the test file provides a matching forward declaration.
add_firmware_test(test_notification_def
    test_notification_def.cxx
)
