cmake_minimum_required(VERSION 3.16)
project(heltec_ancs_tests CXX C)

set(CMAKE_CXX_STANDARD 20)
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

# ── Crypto library detection (mbedtls only) ───────────────────────────────
#
# Two-tier strategy:
#
#   Priority 1 — ESP-IDF bundled source (IDF_PATH is set).
#     add_subdirectory() the upstream subtree that ESP-IDF vendors inside its
#     components/ directory.
#     mbedtls:   $IDF_PATH/components/mbedtls/mbedtls/
#
#   Priority 2 — System library (IDF_PATH not set, or upstream dir absent).
#     pkg-config first, then manual Homebrew/apt fallback paths.
#     Install: brew install mbedtls          (macOS)
#              apt-get install libmbedtls-dev (Debian/Ubuntu)

find_package(PkgConfig QUIET)
set(MBEDTLS_FOUND  FALSE)

# ── mbedtls ───────────────────────────────────────────────────────────────
if(DEFINED ENV{IDF_PATH})
    set(_IDF_MBEDTLS "$ENV{IDF_PATH}/components/mbedtls/mbedtls")
    if(EXISTS "${_IDF_MBEDTLS}/CMakeLists.txt")
        set(ENABLE_PROGRAMS      OFF CACHE BOOL "" FORCE)
        set(ENABLE_TESTING       OFF CACHE BOOL "" FORCE)
        set(MBEDTLS_FATAL_WARNINGS OFF CACHE BOOL "" FORCE)
        add_subdirectory("${_IDF_MBEDTLS}"
                         "${CMAKE_CURRENT_BINARY_DIR}/mbedtls_idf"
                         EXCLUDE_FROM_ALL)
        # The upstream mbedtls CMakeLists.txt creates a target named "mbedcrypto".
        set(MBEDTLS_LINK_TARGET mbedcrypto)
        set(MBEDTLS_FOUND TRUE)
        message(STATUS "Using ESP-IDF mbedtls: ${_IDF_MBEDTLS}")
    endif()
endif()

if(NOT MBEDTLS_FOUND)
    if(PkgConfig_FOUND)
        pkg_check_modules(MBEDCRYPTO QUIET IMPORTED_TARGET mbedcrypto)
        if(MBEDCRYPTO_FOUND)
            set(MBEDTLS_LINK_TARGET PkgConfig::MBEDCRYPTO)
            set(MBEDTLS_FOUND TRUE)
            message(STATUS "Found mbedtls via pkg-config")
        endif()
    endif()
endif()

if(NOT MBEDTLS_FOUND)
    find_library(MBEDTLS_CRYPTO_LIB
        NAMES mbedcrypto
        HINTS
            /opt/homebrew/opt/mbedtls/lib
            /opt/homebrew/lib
            /usr/local/opt/mbedtls/lib
            /usr/local/lib
            /usr/lib
            /usr/lib/x86_64-linux-gnu
            /usr/lib/aarch64-linux-gnu
    )
    find_path(MBEDTLS_INCLUDE_DIR
        NAMES mbedtls/aes.h
        HINTS
            /opt/homebrew/opt/mbedtls/include
            /opt/homebrew/include
            /usr/local/opt/mbedtls/include
            /usr/local/include
            /usr/include
    )
    if(MBEDTLS_CRYPTO_LIB AND MBEDTLS_INCLUDE_DIR)
        add_library(system_mbedcrypto UNKNOWN IMPORTED)
        set_target_properties(system_mbedcrypto PROPERTIES
            IMPORTED_LOCATION             "${MBEDTLS_CRYPTO_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${MBEDTLS_INCLUDE_DIR}"
        )
        set(MBEDTLS_LINK_TARGET system_mbedcrypto)
        set(MBEDTLS_FOUND TRUE)
        message(STATUS "Found system mbedtls: ${MBEDTLS_CRYPTO_LIB}")
        message(STATUS "  includes:           ${MBEDTLS_INCLUDE_DIR}")
    endif()
endif()

if(NOT MBEDTLS_FOUND)
    message(WARNING "mbedtls not found — set IDF_PATH or install: "
                    "brew install mbedtls  /  apt-get install libmbedtls-dev")
    message(WARNING "test_mesh_crypto will be skipped.")
endif()

# ── Paths ─────────────────────────────────────────────────────────────────
set(MAIN_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../main)
set(STUB_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/stubs)

# ── Common compile options ────────────────────────────────────────────────
set(COMMON_FLAGS -Wall -Wextra -Wno-unused-parameter)

# ── Helper: add a test executable ────────────────────────────────────────
function(add_firmware_test name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE
        ${MAIN_DIR}   # production headers
        ${STUB_DIR}   # ESP-IDF / FreeRTOS shims
    )
    target_compile_options(${name} PRIVATE ${COMMON_FLAGS})
    target_link_libraries(${name} PRIVATE unity)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

enable_testing()

# ── test_mesh_codec ───────────────────────────────────────────────────────
add_firmware_test(test_mesh_codec
    test_mesh_codec.cxx
    ${MAIN_DIR}/mesh_codec.cxx
)

# ── test_mesh_crypto ──────────────────────────────────────────────────────
# Tests AES-128-CTR, AES-256-CTR, AES-256-CCM, SHA-256, X25519 key
# derivation, X25519 ECDH, and PKC DM round-trips using NIST SP 800-38A
# and RFC 7748 known test vectors.
# Requires mbedtls (AES-CTR + AES-CCM + SHA-256).
if(MBEDTLS_FOUND)
    add_executable(test_mesh_crypto
        test_mesh_crypto.cxx
        ${MAIN_DIR}/mesh_crypto.cxx
        ${MAIN_DIR}/mesh_codec.cxx   # mc_encodeData / mc_parseData used in OTA tests
    )
    target_include_directories(test_mesh_crypto PRIVATE
        ${MAIN_DIR}
        ${STUB_DIR}
    )
    target_compile_options(test_mesh_crypto PRIVATE ${COMMON_FLAGS})
    target_link_libraries(test_mesh_crypto PRIVATE
        unity ${MBEDTLS_LINK_TARGET})
    add_test(NAME test_mesh_crypto COMMAND test_mesh_crypto)
else()
    message(STATUS "Skipping test_mesh_crypto — mbedtls not found")
endif()

# ── test_applist ──────────────────────────────────────────────────────────
add_firmware_test(test_applist
    test_applist.cxx
    ${MAIN_DIR}/applist.cxx
)

# ── test_notification_def ─────────────────────────────────────────────────
add_firmware_test(test_notification_def
    test_notification_def.cxx
)

# ── test_battery_monitor ──────────────────────────────────────────────────
# Tests the piecewise-linear LiPo discharge curve (BatteryMonitor::_voltageToPct).
# Platform-free: the curve logic is replicated inline, so no firmware stubs needed.
add_firmware_test(test_battery_monitor
    test_battery_monitor.cxx
)
