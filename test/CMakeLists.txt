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

# ── Locate mbedtls (required for test_mesh_crypto) ────────────────────────
# Priority 1: ESP-IDF bundled mbedtls — built from source via add_subdirectory
#             to guarantee version consistency with the firmware.
# Priority 2: System mbedtls (brew install mbedtls / apt-get install libmbedtls-dev).
#
# Both paths produce a CMake target in MBEDTLS_LINK_TARGET that exports the
# correct include directories and link libraries, so downstream targets only
# need target_link_libraries(... ${MBEDTLS_LINK_TARGET}).
set(MBEDTLS_FOUND FALSE)

if(DEFINED ENV{IDF_PATH})
    set(_IDF_MBEDTLS "$ENV{IDF_PATH}/components/mbedtls/mbedtls")
    if(EXISTS "${_IDF_MBEDTLS}/CMakeLists.txt")
        # Suppress optional mbedtls targets we don't need.
        set(ENABLE_PROGRAMS      OFF CACHE BOOL "" FORCE)
        set(ENABLE_TESTING       OFF CACHE BOOL "" FORCE)
        set(MBEDTLS_FATAL_WARNINGS OFF CACHE BOOL "" FORCE)
        add_subdirectory("${_IDF_MBEDTLS}"
                         "${CMAKE_CURRENT_BINARY_DIR}/mbedtls"
                         EXCLUDE_FROM_ALL)
        set(MBEDTLS_FOUND TRUE)
        # The mbedcrypto target exports its include dirs automatically.
        set(MBEDTLS_LINK_TARGET mbedcrypto)
        message(STATUS "Using ESP-IDF mbedtls: ${_IDF_MBEDTLS}")
    endif()
endif()

if(NOT MBEDTLS_FOUND)
    # Fall back to system mbedtls.
    # Homebrew (macOS): brew install mbedtls
    #   Apple Silicon: /opt/homebrew/opt/mbedtls/
    #   Intel:         /usr/local/opt/mbedtls/
    # Debian/Ubuntu:   apt-get install libmbedtls-dev
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
        # Wrap system library in an imported target for uniform linking.
        add_library(system_mbedcrypto UNKNOWN IMPORTED)
        set_target_properties(system_mbedcrypto PROPERTIES
            IMPORTED_LOCATION "${MBEDTLS_CRYPTO_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${MBEDTLS_INCLUDE_DIR}"
        )
        set(MBEDTLS_LINK_TARGET system_mbedcrypto)
        set(MBEDTLS_FOUND TRUE)
        message(STATUS "Using system mbedtls: ${MBEDTLS_CRYPTO_LIB}")
        message(STATUS "  includes:           ${MBEDTLS_INCLUDE_DIR}")
    else()
        message(WARNING
            "mbedtls not found — test_mesh_crypto will be skipped.\n"
            "Set IDF_PATH to use ESP-IDF's bundled mbedtls, or install:\n"
            "  brew install mbedtls              (macOS)\n"
            "  apt-get install libmbedtls-dev    (Debian/Ubuntu)")
    endif()
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
# Tests AES-128-CTR, AES-256-CTR, X25519 key derivation, X25519 ECDH, and
# PKC DM round-trips using NIST SP 800-38A and RFC 7748 known test vectors.
# Requires mbedtls — prefers ESP-IDF's bundled copy (IDF_PATH), falls back
# to system mbedtls.
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
    target_link_libraries(test_mesh_crypto PRIVATE unity ${MBEDTLS_LINK_TARGET})
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
