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

#ifndef APP_LIST_H_
#define APP_LIST_H_

#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

enum application_def
{
    APP_UNKNOWN = 0,
    APP_SMS,
    APP_PHONE,
    APP_FACETIME,
    APP_MESSENGER,
    APP_PINGER,
    APP_TEXTNOW,
    APP_KEYBASE,
    APP_SIGNAL,
    APP_HONK,
    APP_GHRN,
    APP_TOWBOOK,
};

struct AppMapping {
    const char* bundleId;
    application_def appId;
};

struct AppDisplayName {
    application_def appId;
    const char* displayName;
};

/**
 * ApplicationList
 *
 * Two-layer design:
 *  1. Built-in entries — compiled into flash (.rodata), never modified.
 *  2. Custom entries  — stored in NVS (namespace "applist"), loaded at boot,
 *                       managed at runtime via addEntry() / removeEntry() / resetToDefaults().
 *
 * Custom entries always have type APP_UNKNOWN; their display name is stored
 * directly alongside the bundle ID and retrieved via getDisplayName(bundleId).
 */
class ApplicationList
{
public:
    static constexpr size_t MAX_CUSTOM_ENTRIES = 16;
    static constexpr size_t BUNDLE_ID_MAX      = 64;
    static constexpr size_t DISPLAY_NAME_MAX   = 32;

    struct CustomEntry {
        char bundleId[BUNDLE_ID_MAX]       = {};
        char displayName[DISPLAY_NAME_MAX] = {};
    };

    /// Constructor — loads custom entries from NVS.
    ApplicationList();
    ~ApplicationList();

    // ── Query ─────────────────────────────────────────────────────────────
    bool isAllowedApplication(const char* bundleId) const;
    application_def getApplicationId(const char* bundleId) const;

    /// Look up display name by enum (built-in apps only).
    const char* getDisplayName(application_def appId) const;

    /// Look up display name by bundle ID (checks custom entries first, then
    /// built-ins, then returns the bundle ID itself as a fallback).
    const char* getDisplayName(const char* bundleId) const;

    bool isBuiltIn(const char* bundleId) const;

    // ── Management (persisted to NVS) ─────────────────────────────────────
    /// Add a custom entry.  Returns false if the list is full or the bundle ID
    /// already exists (either as a built-in or an existing custom entry).
    bool addEntry(const char* bundleId, const char* displayName);

    /// Remove a custom entry.  Returns false if not found, or it is a built-in.
    bool removeEntry(const char* bundleId);

    /// Clear all custom entries and erase the NVS partition.
    void resetToDefaults();

    // ── Iteration (for BLE listing) ───────────────────────────────────────
    size_t getCustomCount() const;
    bool   getCustomEntry(size_t idx, CustomEntry& out) const;

private:
    // ── Built-in tables (flash) ───────────────────────────────────────────
    static constexpr AppMapping _builtinMappings[] = {
        {"com.apple.MobileSMS",           APP_SMS},
        {"com.apple.mobilephone",         APP_PHONE},
        {"com.apple.facetime",            APP_FACETIME},
        {"com.facebook.Messenger",        APP_MESSENGER},
        {"com.pinger.textfreeWithVoice",  APP_PINGER},
        {"com.tinginteractive.usms",      APP_TEXTNOW},
        {"keybase.ios",                   APP_KEYBASE},
        {"org.whispersystems.signal",     APP_SIGNAL},
        {"com.honkforhelp.driver",        APP_HONK},
        {"com.arity.rescuer",             APP_GHRN},
        {"com.towbook.mobile",            APP_TOWBOOK},
    };

    static constexpr AppDisplayName _builtinNames[] = {
        {APP_SMS,       "iMessage"},
        {APP_PHONE,     "Call"},
        {APP_FACETIME,  "Facetime"},
        {APP_MESSENGER, "Facebook"},
        {APP_PINGER,    "Pinger"},
        {APP_TEXTNOW,   "TextNow"},
        {APP_KEYBASE,   "Keybase"},
        {APP_SIGNAL,    "Signal"},
        {APP_HONK,      "Honk"},
        {APP_GHRN,      "GHRN"},
        {APP_TOWBOOK,   "Towbook"},
    };

    static constexpr size_t _builtinCount     = sizeof(_builtinMappings) / sizeof(_builtinMappings[0]);
    static constexpr size_t _builtinNameCount = sizeof(_builtinNames)    / sizeof(_builtinNames[0]);

    // ── Runtime custom entries (DRAM) ─────────────────────────────────────
    CustomEntry       _custom[MAX_CUSTOM_ENTRIES];
    size_t            _customCount = 0;
    SemaphoreHandle_t _mutex;

    void _loadFromNvs();
    void _saveToNvs() const;
};

extern ApplicationList AppList;

#endif /* APP_LIST_H_ */