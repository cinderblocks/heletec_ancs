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

#include "applist.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <cstring>
#include <esp_log.h>

static const char* TAG = "applist";

// ---------------------------------------------------------------------------
// Scoped mutex guard — matches the pattern in notificationservice.cxx.
// Null-safe: no-ops when the handle is nullptr (OOM at construction time).
// ---------------------------------------------------------------------------
struct ScopedLock {
    explicit ScopedLock(SemaphoreHandle_t m) : _m(m) { if (_m) xSemaphoreTake(_m, portMAX_DELAY); }
    ~ScopedLock() { if (_m) xSemaphoreGive(_m); }
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
private:
    SemaphoreHandle_t _m;
};

// NVS namespace and key names.
static constexpr char NVS_NAMESPACE[] = "applist";
static constexpr char NVS_KEY_COUNT[] = "cnt";
// Entry keys: "e0" … "e15"

// Define static constexpr arrays (required by C++14; ODR-used in loops).
constexpr AppMapping     ApplicationList::_builtinMappings[];
constexpr AppDisplayName ApplicationList::_builtinNames[];

// NVS entry key table — one key per slot, matching MAX_CUSTOM_ENTRIES (16).
// Using a static table avoids snprintf format-truncation warnings: the compiler
// cannot statically bound a loop index, but a direct array lookup is always safe.
static constexpr const char* NVS_ENTRY_KEYS[] = {
    "e0","e1","e2", "e3", "e4", "e5", "e6",  "e7",
    "e8","e9","e10","e11","e12","e13","e14","e15",
};
static_assert(
    sizeof(NVS_ENTRY_KEYS)/sizeof(NVS_ENTRY_KEYS[0]) >= ApplicationList::MAX_CUSTOM_ENTRIES,
    "NVS_ENTRY_KEYS must have at least MAX_CUSTOM_ENTRIES entries");

// ── Constructor / destructor ──────────────────────────────────────────────
ApplicationList::ApplicationList()
    : _customCount(0)
    , _mutex(xSemaphoreCreateMutex())
{
    _loadFromNvs();
}

ApplicationList::~ApplicationList()
{
    if (_mutex) vSemaphoreDelete(_mutex);
}

// ── NVS persistence ───────────────────────────────────────────────────────
void ApplicationList::_loadFromNvs()
{
    // This may be called from the global constructor before app_main, so NVS
    // may not have been initialised yet.  nvs_flash_init() is idempotent —
    // calling it here is safe even if app_main calls it again later.
    esp_err_t initErr = nvs_flash_init();
    if (initErr == ESP_ERR_NVS_NO_FREE_PAGES ||
        initErr == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        initErr = nvs_flash_init();
    }
    if (initErr != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_init: %s — skipping NVS load",
                 esp_err_to_name(initErr));
        return;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return;  // namespace not yet created — first boot, nothing to load
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open(r): %s", esp_err_to_name(err));
        return;
    }

    uint8_t count = 0;
    nvs_get_u8(handle, NVS_KEY_COUNT, &count);
    if (count > MAX_CUSTOM_ENTRIES) count = MAX_CUSTOM_ENTRIES;

    _customCount = 0;
    for (uint8_t i = 0; i < count; i++) {
        size_t sz = sizeof(CustomEntry);
        CustomEntry entry = {};
        if (nvs_get_blob(handle, NVS_ENTRY_KEYS[i], &entry, &sz) == ESP_OK) {
            entry.bundleId[BUNDLE_ID_MAX - 1]       = '\0';
            entry.displayName[DISPLAY_NAME_MAX - 1] = '\0';
            _custom[_customCount++] = entry;
        }
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Loaded %zu custom entr%s from NVS",
             _customCount, _customCount == 1 ? "y" : "ies");
}

void ApplicationList::_saveToNvs() const
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(rw): %s", esp_err_to_name(err));
        return;
    }

    nvs_set_u8(handle, NVS_KEY_COUNT, (uint8_t)_customCount);
    for (size_t i = 0; i < _customCount; i++) {
        nvs_set_blob(handle, NVS_ENTRY_KEYS[i], &_custom[i], sizeof(CustomEntry));
    }
    nvs_commit(handle);
    nvs_close(handle);
}

// ── Query ─────────────────────────────────────────────────────────────────
bool ApplicationList::isAllowedApplication(const char* bundleId) const
{
    // Built-ins (no lock needed — const data in flash)
    for (size_t i = 0; i < _builtinCount; i++) {
        if (strcmp(bundleId, _builtinMappings[i].bundleId) == 0)
            return true;
    }
    // Custom entries
    ScopedLock lock(_mutex);
    for (size_t i = 0; i < _customCount; i++) {
        if (strcmp(bundleId, _custom[i].bundleId) == 0)
            return true;
    }
    return false;
}

application_def ApplicationList::getApplicationId(const char* bundleId) const
{
    for (size_t i = 0; i < _builtinCount; i++) {
        if (strcmp(bundleId, _builtinMappings[i].bundleId) == 0)
            return _builtinMappings[i].appId;
    }
    // Custom entries are all APP_UNKNOWN — caller uses bundleId for display.
    return APP_UNKNOWN;
}

const char* ApplicationList::getDisplayName(application_def appId) const
{
    for (size_t i = 0; i < _builtinNameCount; i++) {
        if (_builtinNames[i].appId == appId)
            return _builtinNames[i].displayName;
    }
    return "";
}

const char* ApplicationList::getDisplayName(const char* bundleId) const
{
    // 1. Check custom entries first so the user can override a built-in name.
    // Copy the display name under the lock to avoid holding it across the
    // built-in and return paths below.
    {
        ScopedLock lock(_mutex);
        for (size_t i = 0; i < _customCount; i++) {
            if (strcmp(bundleId, _custom[i].bundleId) == 0)
                return _custom[i].displayName;
        }
    } // lock released here

    // 2. Built-in lookup via enum.
    for (size_t i = 0; i < _builtinCount; i++) {
        if (strcmp(bundleId, _builtinMappings[i].bundleId) == 0)
            return getDisplayName(_builtinMappings[i].appId);
    }

    // 3. Unknown — return the bundle ID itself as a readable fallback.
    return bundleId;
}

bool ApplicationList::isBuiltIn(const char* bundleId) const
{
    for (size_t i = 0; i < _builtinCount; i++) {
        if (strcmp(bundleId, _builtinMappings[i].bundleId) == 0)
            return true;
    }
    return false;
}

// ── Management ────────────────────────────────────────────────────────────
bool ApplicationList::addEntry(const char* bundleId, const char* displayName)
{
    if (!bundleId || bundleId[0] == '\0') return false;
    if (!displayName) displayName = "";

    // Reject built-in IDs (already allowed).
    if (isBuiltIn(bundleId)) {
        ESP_LOGW(TAG, "addEntry: '%s' is a built-in entry", bundleId);
        return false;
    }

    ScopedLock lock(_mutex);

    // Reject duplicates.
    for (size_t i = 0; i < _customCount; i++) {
        if (strcmp(bundleId, _custom[i].bundleId) == 0) {
            ESP_LOGW(TAG, "addEntry: '%s' already exists", bundleId);
            return false;
        }
    }

    if (_customCount >= MAX_CUSTOM_ENTRIES) {
        ESP_LOGW(TAG, "addEntry: custom list full (%zu entries)", MAX_CUSTOM_ENTRIES);
        return false;
    }

    CustomEntry& e = _custom[_customCount++];
    strncpy(e.bundleId,    bundleId,    BUNDLE_ID_MAX    - 1); e.bundleId[BUNDLE_ID_MAX - 1]       = '\0';
    strncpy(e.displayName, displayName, DISPLAY_NAME_MAX - 1); e.displayName[DISPLAY_NAME_MAX - 1] = '\0';

    _saveToNvs();

    ESP_LOGI(TAG, "Added '%s' (\"%s\")  total custom: %zu", bundleId, displayName, _customCount);
    return true;
}

bool ApplicationList::removeEntry(const char* bundleId)
{
    if (isBuiltIn(bundleId)) {
        ESP_LOGW(TAG, "removeEntry: '%s' is a built-in and cannot be removed", bundleId);
        return false;
    }

    ScopedLock lock(_mutex);

    bool found = false;
    for (size_t i = 0; i < _customCount; i++) {
        if (strcmp(bundleId, _custom[i].bundleId) == 0) {
            // Compact: shift tail entries down one slot.
            memmove(&_custom[i], &_custom[i + 1],
                    (_customCount - i - 1) * sizeof(CustomEntry));
            _customCount--;
            found = true;
            break;
        }
    }

    if (found) {
        _saveToNvs();
        ESP_LOGI(TAG, "Removed '%s'  remaining custom: %zu", bundleId, _customCount);
    } else {
        ESP_LOGW(TAG, "removeEntry: '%s' not found", bundleId);
    }

    return found;
}

void ApplicationList::resetToDefaults()
{
    ScopedLock lock(_mutex);
    _customCount = 0;

    // Erase all keys from the NVS namespace.
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_erase_all(handle);
        nvs_commit(handle);
        nvs_close(handle);
    }

    ESP_LOGI(TAG, "Custom list cleared — reverted to built-in defaults");
}

// ── Iteration ─────────────────────────────────────────────────────────────
size_t ApplicationList::getCustomCount() const
{
    ScopedLock lock(_mutex);
    return _customCount;
}

bool ApplicationList::getCustomEntry(size_t idx, CustomEntry& out) const
{
    ScopedLock lock(_mutex);
    bool ok = (idx < _customCount);
    if (ok) out = _custom[idx];
    return ok;
}

/* extern */
ApplicationList AppList;
