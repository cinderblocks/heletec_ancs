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

#include "bleservice.h"
#include <esp_log.h>
#include "sdkconfig.h"

#include "ancs.h"
#include "hardware.h"
#include "notificationservice.h"
#include <NimBLEHIDDevice.h>
#include <host/ble_gap.h>
#include <host/ble_store.h>
#include <sys/time.h>
#include <climits>

static const char* TAG = "heltec";

// Bluetooth SIG generic-display appearance value (0x0080).
static constexpr uint16_t BLE_APPEARANCE_GENERIC_DISPLAY = 0x0080;

static const auto ANCS_SERVICE_UUID            = NimBLEUUID("7905F431-B5CE-4E99-A40F-4B1E122D00D0");
static const auto NOTIFICATION_SOURCE_CHR_UUID = NimBLEUUID("9FBF120D-6301-42D9-8C58-25E699A21DBD");
static const auto CONTROL_POINT_CHR_UUID       = NimBLEUUID("69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9");
static const auto DATA_SOURCE_CHR_UUID         = NimBLEUUID("22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB");

// Bluetooth SIG Current Time Service (CTS) — UUID 0x1805
// iOS exposes this service on an encrypted link to allow accessories to sync their clocks.
static const auto CTS_SERVICE_UUID          = NimBLEUUID((uint16_t)0x1805);
static const auto CURRENT_TIME_CHR_UUID     = NimBLEUUID((uint16_t)0x2A2B); // Exact Time 256 + adjust reason
static const auto LOCAL_TIME_INFO_CHR_UUID  = NimBLEUUID((uint16_t)0x2A0F); // Timezone + DST offset

// Map Kconfig CHOICE (CONFIG_NIMBLE_LOG_LEVEL_*) to esp_log_level_t.
// Exactly one of these symbols is defined by the build system.
#if   defined(CONFIG_NIMBLE_LOG_LEVEL_NONE)
#  define _NIMBLE_LOG_LEVEL ESP_LOG_NONE
#elif defined(CONFIG_NIMBLE_LOG_LEVEL_ERROR)
#  define _NIMBLE_LOG_LEVEL ESP_LOG_ERROR
#elif defined(CONFIG_NIMBLE_LOG_LEVEL_INFO)
#  define _NIMBLE_LOG_LEVEL ESP_LOG_INFO
#elif defined(CONFIG_NIMBLE_LOG_LEVEL_DEBUG)
#  define _NIMBLE_LOG_LEVEL ESP_LOG_DEBUG
#elif defined(CONFIG_NIMBLE_LOG_LEVEL_VERBOSE)
#  define _NIMBLE_LOG_LEVEL ESP_LOG_VERBOSE
#else  // CONFIG_NIMBLE_LOG_LEVEL_WARN (default) or undefined
#  define _NIMBLE_LOG_LEVEL ESP_LOG_WARN
#endif

BleService::BleService(const NotificationCallback notificationSourceCallback, const NotificationCallback dataSourceCallback)
:   _notificationSourceCallback(notificationSourceCallback)
,   _dataSourceCallback(dataSourceCallback)
{ }

/* virtual */
BleService::~BleService()
{
    if (_hidDevice != nullptr) {
        delete _hidDevice;
        _hidDevice = nullptr;
    }
}

void BleService::startServer(const char* appName)
{
    // Initialize device
    NimBLEDevice::init(appName);

    // Apply NimBLE log level selected in menuconfig (BLE Configuration menu).
    esp_log_level_set("NimBLE", _NIMBLE_LOG_LEVEL);

    NimBLEDevice::setPower(9);  // +9 dBm (max for ESP32-S3)

    // Security: bonding + MITM + secure connections, display-only IO capability
    NimBLEDevice::setSecurityAuth(true, true, true);  // bonding, mitm, sc
    // DISPLAY_YES_NO = numeric comparison: both devices show the same 6-digit
    // number and the user taps YES/NO on each side to confirm they match.
    // (DISPLAY_ONLY would require the user to type the number into iOS.)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO);
    NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    NimBLEDevice::setSecurityPasskey(420420);

    // Log stored bonds for diagnostics — confirms NVS persistence is working.
    ESP_LOGI(TAG, "Stored bonds: %d", NimBLEDevice::getNumBonds());

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallback(this));
    // Advertising restart is driven by the client task after full cleanup.
    pServer->advertiseOnDisconnect(false);

    _hidDevice = new NimBLEHIDDevice(pServer);
    _hidDevice->setManufacturer("Sjofn LLC");
    _hidDevice->setPnp(0x00, 0x00C3, 0xffff, 0x0001);
    _hidDevice->setHidInfo(0x00, 0x01);
    _hidDevice->setBatteryLevel(100);
    _hidDevice->startServices();

    _restartAdvertising();
}

void BleService::_restartAdvertising()
{
    // ANCS UUID: 7905F431-B5CE-4E99-A40F-4B1E122D00D0 in little-endian byte order
    // AD type 0x15 = List of 128-bit Service Solicitation UUIDs
    // length=17 (1 type byte + 16 UUID bytes)
    //
    // *** This MUST be in the PRIMARY advertisement packet, NOT the scan response. ***
    // iOS in background mode does passive scanning (no SCAN_REQ), so scan response
    // packets are never received. Without the solicitation in the primary packet,
    // iOS will not activate ANCS for this device.
    static const uint8_t ANCS_SOLICIT[] = {
        17, 0x15,                                // length=17, AD type: 128-bit solicitation
        0xD0, 0x00, 0x2D, 0x12,                 // UUID bytes 0-3  (little-endian)
        0x1E, 0x4B, 0x0F, 0xA4,                 // bytes 4-7
        0x99, 0x4E, 0xCE, 0xB5,                 // bytes 8-11
        0x31, 0xF4, 0x05, 0x79                  // bytes 12-15
    };

    // Primary advertisement: flags + appearance + ANCS solicitation = 25 bytes (fits in 31)
    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN);
    advData.setAppearance(BLE_APPEARANCE_GENERIC_DISPLAY);
    advData.addData(ANCS_SOLICIT, sizeof(ANCS_SOLICIT));

    // Scan response: HID service UUID + preferred connection parameters = 10 bytes
    // Seen by iOS only when it actively scans (e.g. Settings > Bluetooth is open)
    NimBLEAdvertisementData scanData;
    if (_hidDevice != nullptr) {
        scanData.addServiceUUID(_hidDevice->getHidService()->getUUID());
    }
    scanData.setPreferredParams(0x06, 0x10);    // 7.5 ms min, 20 ms max

    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    // enableScanResponse first: sets m_scanResp=true and resets m_advDataSet=false
    // so that start() will re-push both adv data and scan response to the host on
    // every call (including after a host reset / onHostSync).
    pAdv->enableScanResponse(true);
    // Store the payloads in m_advData / m_scanData so onHostSync can recover them.
    // setAdvertisementData also calls ble_gap_adv_set_data immediately.
    pAdv->setAdvertisementData(advData);
    // setScanResponseData calls ble_gap_adv_rsp_set_data immediately and stores in m_scanData.
    pAdv->setScanResponseData(scanData);
    // start() sees m_advDataSet=true (set by setAdvertisementData above), skips the
    // re-push block, and calls ble_gap_adv_start() directly. The host already has the
    // correct adv and scan-response data from the explicit calls above.
    pAdv->start();
}
void BleService::startClient(void *data)
{
    if (data == nullptr) { vTaskDelete(nullptr); return; }
    ESP_LOGI(TAG, "Starting client task");
    auto clientParam = static_cast<ClientParameter *>(data);
    BleService *bleService = clientParam->bleService;
    uint16_t connHandle = clientParam->connHandle;

    // ── Step 1: Obtain the client for the existing server connection ──────────
    //
    // NimBLEServer::getClient(connHandle) is the correct API for the dual-role
    // (peripheral + central) scenario. It bypasses connect() entirely and directly
    // sets m_connHandle on the NimBLEClient, reusing the existing GAP link.
    // The client is owned by the server and must be freed with getServer()->deleteClient().
    NimBLEClient *pClient = NimBLEDevice::getServer()->getClient(connHandle);
    if (pClient == nullptr)
    {
        ESP_LOGW(TAG, "getClient() returned null — connection already gone");
        goto EndTask;
    }
    bleService->_pClient = pClient;

    // ── Step 2: Wait for the link to be encrypted ─────────────────────────────
    //
    // Bonded reconnect: iOS fires LL_START_ENC_REQ automatically within ~300 ms
    // and encryption completes before our 1500 ms Security-Request timer fires.
    //
    // Stale-bond (ESP32 has no matching LTK): NimBLE sends LTK_Request_Negative_
    // Reply → controller sends LL_REJECT_EXT_IND(PIN_or_Key_Missing) →
    // BLE_GAP_EVENT_ENC_CHANGE(fail) → onAuthenticationComplete(fail) deletes the
    // local bond entry and returns WITHOUT calling startSecurity().  The connection
    // stays alive.  At T=1500 ms the wait loop below sends the Security Request
    // from the client task context (safe — not inside the NimBLE host callback).
    // iOS, having received PIN_or_Key_Missing, has deleted its own LTK and will
    // respond with a fresh Pairing Request → iOS pairing dialog appears.
    //
    // Fresh pairing (no bond on either side): iOS never sends LL_START_ENC_REQ.
    // At T=1500 ms we send the Security Request → iOS pairing dialog appears.
    //
    // IMPORTANT: do NOT call startSecurity() from onAuthenticationComplete.
    // That callback runs on the NimBLE host task while SMP is still unwinding
    // the failed encryption.  Calling ble_gap_security_initiate() re-enters the
    // SMP state machine mid-transition, causing NimBLE to terminate the
    // connection with a MIC failure (reason 573).  iOS treats MIC failure as our
    // error, retains its bond, and retries LL_START_ENC_REQ on every reconnect.
    {
        static constexpr uint32_t SECURITY_REQUEST_DELAY_MS = 1500;
        ESP_LOGI(TAG, "Waiting for encryption (up to 30 s)...");
        ble_gap_conn_desc authDesc{};
        const TickType_t encWaitStart  = xTaskGetTickCount();
        const TickType_t deadline      = encWaitStart + pdMS_TO_TICKS(30000);
        bool encrypted         = false;
        bool securityRequested = false;
        while (xTaskGetTickCount() < deadline)
        {
            // Notification from onDisconnect — exit immediately.
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) != 0)
            {
                ESP_LOGW(TAG, "Connection lost while waiting for encryption");
                goto EndTask;
            }
            if (ble_gap_conn_find(connHandle, &authDesc) != 0)
            {
                ESP_LOGW(TAG, "Connection lost while waiting for encryption");
                goto EndTask;
            }
            if (authDesc.sec_state.encrypted)
            {
                encrypted = true;
                break;
            }
            // iOS has not started encryption — send a Security Request to prompt
            // the iOS pairing dialog (covers fresh pairing and stale-bond cases).
            if (!securityRequested &&
                (xTaskGetTickCount() - encWaitStart) >= pdMS_TO_TICKS(SECURITY_REQUEST_DELAY_MS))
            {
                ESP_LOGI(TAG, "iOS has not encrypted — sending Security Request");
                NimBLEDevice::startSecurity(connHandle);
                securityRequested = true;
            }
        }
        if (!encrypted)
        {
            ESP_LOGW(TAG, "Timed out waiting for encryption");
            goto EndTask;
        }
        ESP_LOGI(TAG, "Link encrypted — beginning ANCS discovery");
    }

    // ── Step 3: Discover ANCS service and characteristics ────────────────────
    {
    NimBLERemoteService *pAncsService = pClient->getService(ANCS_SERVICE_UUID);
    if (pAncsService == nullptr)
    {
        ESP_LOGW(TAG, "Failed to find ANCS service on peer");
        goto EndTask;
    }

    // Discover ALL characteristics at once (refresh=true) so m_vChars is populated
    // in ascending handle order. If we instead call getCharacteristic(uuid) individually,
    // each call appends to m_vChars in query order (not handle order). That breaks
    // retrieveDescriptors() which computes endHandle = next_chr.val_handle - 1 by
    // iterating the vector — an out-of-order vector yields endHandle < val_handle,
    // no CCCD is found, subscribe() silently returns true without writing the CCCD,
    // and iOS never receives the Notification Source subscription that triggers the
    // ANCS permission dialog.
    pAncsService->getCharacteristics(true);

    bleService->_notificationSourceCharacteristic =
        pAncsService->getCharacteristic(NOTIFICATION_SOURCE_CHR_UUID);
    if (bleService->_notificationSourceCharacteristic == nullptr)
    {
        ESP_LOGW(TAG, "Failed to find notification source characteristic");
        goto EndTask;
    }

    bleService->_controlPointCharacteristic =
        pAncsService->getCharacteristic(CONTROL_POINT_CHR_UUID);
    if (bleService->_controlPointCharacteristic == nullptr)
    {
        ESP_LOGW(TAG, "Failed to find control point characteristic");
        goto EndTask;
    }

    bleService->_dataSourceCharacteristic =
        pAncsService->getCharacteristic(DATA_SOURCE_CHR_UUID);
    if (bleService->_dataSourceCharacteristic == nullptr)
    {
        ESP_LOGW(TAG, "Failed to find datasource characteristic");
        goto EndTask;
    }

    // subscribe() writes 0x0001 to the CCCD. The Notification Source subscription
    // is the specific write that triggers the iOS "Allow Notifications Access?" dialog.
    auto nsCallback = bleService->_notificationSourceCallback;
    if (!bleService->_notificationSourceCharacteristic->subscribe(
        true,
        [nsCallback](NimBLERemoteCharacteristic *pChr, uint8_t *pData, size_t length, bool isNotify) {
            nsCallback(pChr, pData, length, isNotify);
        }))
    {
        ESP_LOGW(TAG, "Failed to subscribe to Notification Source — CCCD write rejected or not found");
        goto EndTask;
    }

    auto dsCallback = bleService->_dataSourceCallback;
    if (!bleService->_dataSourceCharacteristic->subscribe(
        true,
        [dsCallback](NimBLERemoteCharacteristic *pChr, uint8_t *pData, size_t length, bool isNotify) {
            dsCallback(pChr, pData, length, isNotify);
        }))
    {
        ESP_LOGW(TAG, "Failed to subscribe to Data Source — CCCD write rejected or not found");
        goto EndTask;
    }

    ESP_LOGI(TAG, "ANCS subscriptions active");
    } // end discovery block

    // ── Step 4: Discover Current Time Service (CTS) ───────────────────────────
    //
    // iOS exposes CTS (0x1805) on an encrypted link, so we can sync the ESP32
    // system clock and optionally receive automatic time-change notifications.
    // This step is non-fatal; a missing or unreadable CTS is logged and skipped.
    {
        NimBLERemoteService *pCtsService = pClient->getService(CTS_SERVICE_UUID);
        if (pCtsService != nullptr)
        {
            // ── 4a: Read Local Time Information (0x2A0F) for UTC offset ──────
            // Byte 0: Time Zone in ×15-minute units from UTC (int8, -128=unknown).
            // Byte 1: DST Offset — specific constants (0=no DST, 2=+30 min, 4=+60 min,
            //         8=+120 min, 0xFF=unknown).  Each unit represents 15 minutes.
            NimBLERemoteCharacteristic *pLtiChr =
                pCtsService->getCharacteristic(LOCAL_TIME_INFO_CHR_UUID);
            if (pLtiChr != nullptr && pLtiChr->canRead())
            {
                NimBLEAttValue ltiVal = pLtiChr->readValue();
                if (ltiVal.size() >= 2)
                {
                    int8_t  tzUnits  = static_cast<int8_t>(ltiVal[0]);
                    uint8_t dstUnits = ltiVal[1];
                    int32_t totalMin = static_cast<int32_t>(tzUnits) * 15;
                    if (dstUnits != 0xFF) totalMin += static_cast<int32_t>(dstUnits) * 15;
                    bleService->_utcOffsetSeconds = totalMin * 60;
                    ESP_LOGI(TAG, "CTS: UTC offset %+d min (tz=%d, dst=%u)",
                        static_cast<int>(totalMin), static_cast<int>(tzUnits), dstUnits);
                }
            }

            // ── 4b: Discover and read Current Time characteristic (0x2A2B) ───
            bleService->_currentTimeCharacteristic =
                pCtsService->getCharacteristic(CURRENT_TIME_CHR_UUID);
            if (bleService->_currentTimeCharacteristic != nullptr)
            {
                if (bleService->_currentTimeCharacteristic->canRead())
                {
                    NimBLEAttValue ctVal = bleService->_currentTimeCharacteristic->readValue();
                    if (ctVal.size() >= 9)
                    {
                        BleService::_applyCurrentTime(ctVal.data(), ctVal.size(),
                            bleService->_utcOffsetSeconds, bleService->_timeCallback);
                    }
                }

                // ── 4c: Subscribe to time-change notifications ───────────────
                if (bleService->_currentTimeCharacteristic->canNotify())
                {
                    auto tcb    = bleService->_timeCallback;
                    auto utcOff = bleService->_utcOffsetSeconds;
                    bleService->_currentTimeCharacteristic->subscribe(
                        true,
                        [tcb, utcOff](NimBLERemoteCharacteristic * /*pChr*/,
                                      uint8_t *pData, size_t length, bool /*isNotify*/)
                        {
                            BleService::_applyCurrentTime(pData, length, utcOff, tcb);
                        });
                    ESP_LOGI(TAG, "CTS: subscribed to time-change notifications");
                }
            }
            else
            {
                ESP_LOGW(TAG, "CTS: Current Time characteristic (0x2A2B) not found");
            }
        }
        else
        {
            ESP_LOGI(TAG, "CTS: service not found on peer — time sync unavailable");
        }
    } // end CTS block

    // ── Step 5: Keep-alive loop ───────────────────────────────────────────────
    //
    // Woken by xTaskNotify from ServerCallback::onDisconnect.
    // 200 ms poll via ble_gap_conn_find() catches any disconnect that doesn't
    // deliver a GAP event (e.g. sudden link loss / supervision timeout).
    while (true)
    {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200)) != 0)
            break;  // server-side disconnect notified us

        ble_gap_conn_desc pollDesc{};
        if (ble_gap_conn_find(connHandle, &pollDesc) != 0)
            break;  // link is gone
    }

EndTask:
    ESP_LOGI(TAG, "Ending ClientTask");

    // Clear characteristic pointers before releasing the client so any
    // concurrent retrieveNotificationData() call sees nullptr immediately.
    bleService->_notificationSourceCharacteristic = nullptr;
    bleService->_controlPointCharacteristic       = nullptr;
    bleService->_dataSourceCharacteristic         = nullptr;
    bleService->_currentTimeCharacteristic        = nullptr;
    bleService->_utcOffsetSeconds                 = INT32_MIN;

    // Release the server-owned client. This deletes the NimBLEClient object
    // and all cached service / characteristic objects.
    NimBLEDevice::getServer()->deleteClient();
    bleService->_pClient = nullptr;

    // Clear task handle and param before advertising so a fast reconnect
    // cannot create a second task while this one is still in flight.
    bleService->_clientTaskHandle    = nullptr;
    bleService->_currentClientParam  = nullptr;
    delete clientParam;

    // Only restart advertising if onConnect did NOT spawn a new client task
    // while we were cleaning up.  If _clientTaskHandle is non-null here, it
    // means onConnect fired during EndTask (after we set it null above) and
    // already created a new task for the reconnection — restarting advertising
    // in that case is harmless but skipping it avoids a redundant GAP call.
    if (bleService->_clientTaskHandle == nullptr)
    {
        ESP_LOGI(TAG, "Restarting advertising after disconnect");
        bleService->_restartAdvertising();
    }
    else
    {
        ESP_LOGI(TAG, "New connection already handled — skipping advertising restart");
    }
    vTaskDelete(nullptr);
}

void BleService::retrieveNotificationData(uint32_t notifyUUID) const
{
    if (_controlPointCharacteristic == nullptr) { return; }

    uint8_t uuid[4];
    uuid[0] = notifyUUID;
    uuid[1] = notifyUUID >> 8;
    uuid[2] = notifyUUID >> 16;
    uuid[3] = notifyUUID >> 24;
    uint8_t vIdentifier[] = {0x0, uuid[0], uuid[1], uuid[2], uuid[3], ANCS::NotificationAttributeIDAppIdentifier};
    _controlPointCharacteristic->writeValue(vIdentifier, 6, true);
    uint8_t vTitle[] = {0x0, uuid[0], uuid[1], uuid[2], uuid[3], ANCS::NotificationAttributeIDTitle, 0x0, 0x10};
    _controlPointCharacteristic->writeValue(vTitle, 8, true);
    uint8_t vMessage[] = {0x0, uuid[0], uuid[1], uuid[2], uuid[3], ANCS::NotificationAttributeIDMessage, 0x0, 0x10};
    _controlPointCharacteristic->writeValue(vMessage, 8, true);
    uint8_t vDate[] = {0x0, uuid[0], uuid[1], uuid[2], uuid[3], ANCS::NotificationAttributeIDDate};
    _controlPointCharacteristic->writeValue(vDate, 6, true);
}

void BleService::setServerCallback(ANCSServiceServerCallback *serverCallback)
{
    _serverCallback = serverCallback;
}

void BleService::setClientCallback(ANCSServiceClientCallback *clientCallback)
{
    _clientCallback = clientCallback;
}

void BleService::setBatteryLevel(uint8_t level)
{
    if (_hidDevice != nullptr) {
        _hidDevice->setBatteryLevel(level);
    }
}

void BleService::setTimeCallback(TimeCallback cb)
{
    _timeCallback = cb;
}

/* static */
void BleService::_applyCurrentTime(const uint8_t *pData, size_t length,
                                   int32_t utcOffsetSec, TimeCallback cb)
{
    // Current Time characteristic layout (Bluetooth SIG 0x2A2B, 10 bytes):
    //   Bytes 0–1 : Year (uint16 LE, e.g. 2025)
    //   Byte  2   : Month (1–12)
    //   Byte  3   : Day   (1–31)
    //   Byte  4   : Hours (0–23)
    //   Byte  5   : Minutes (0–59)
    //   Byte  6   : Seconds (0–59)
    //   Byte  7   : Day of Week (1=Mon … 7=Sun, 0=Unknown)
    //   Byte  8   : Fractions256 (1/256 of a second, ignored here)
    //   Byte  9   : Adjust Reason (bitmask, optional)
    if (length < 9)
    {
        ESP_LOGW(TAG, "CTS: data too short (%d bytes, need 9)", static_cast<int>(length));
        return;
    }

    uint16_t year  = static_cast<uint16_t>(pData[0] | (pData[1] << 8));
    uint8_t  month = pData[2];
    uint8_t  day   = pData[3];
    uint8_t  hours = pData[4];
    uint8_t  mins  = pData[5];
    uint8_t  secs  = pData[6];

    // Build a struct tm representing iOS local time (tz + DST already applied by iOS)
    struct tm localTm{};
    localTm.tm_year  = static_cast<int>(year) - 1900;
    localTm.tm_mon   = static_cast<int>(month) - 1;   // tm_mon is 0-based
    localTm.tm_mday  = static_cast<int>(day);
    localTm.tm_hour  = static_cast<int>(hours);
    localTm.tm_min   = static_cast<int>(mins);
    localTm.tm_sec   = static_cast<int>(secs);
    localTm.tm_isdst = -1;

    ESP_LOGI(TAG, "CTS: iOS local time %04d-%02d-%02d %02d:%02d:%02d",
        static_cast<int>(year), static_cast<int>(month), static_cast<int>(day),
        static_cast<int>(hours), static_cast<int>(mins), static_cast<int>(secs));

    // Convert to epoch and subtract the UTC offset to get UTC epoch.
    // mktime() treats its argument as *local* time; since ESP-IDF has no timezone
    // database, TZ is effectively UTC, so mktime() here gives "local time as if UTC".
    time_t localEpoch = mktime(&localTm);
    if (localEpoch == static_cast<time_t>(-1))
    {
        ESP_LOGW(TAG, "CTS: mktime() failed — clock not updated");
        return;
    }

    time_t utcEpoch = (utcOffsetSec != INT32_MIN)
                    ? (localEpoch - static_cast<time_t>(utcOffsetSec))
                    : localEpoch;   // fallback: treat local time as UTC

    struct timeval tv{ .tv_sec = utcEpoch, .tv_usec = 0 };
    settimeofday(&tv, nullptr);
    ESP_LOGI(TAG, "CTS: system clock synced — UTC epoch %lld (offset %+d s)",
        static_cast<long long>(utcEpoch), utcOffsetSec != INT32_MIN ? utcOffsetSec : 0);

    if (cb != nullptr)
    {
        cb(&localTm, utcOffsetSec);
    }
}


////// Server callbacks ///////

void BleService::ServerCallback::onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
{
    ESP_LOGI(TAG, "Device connected (ota=%s, id=%s, bonded=%d, bonds_stored=%d)",
             connInfo.getAddress().toString().c_str(),
             connInfo.getIdAddress().toString().c_str(),
             connInfo.isBonded(),
             NimBLEDevice::getNumBonds());
    ancsService->_isConnected.store(true);
    ancsService->_everEncrypted.store(false); // reset per-connection encryption flag

    if (ancsService->_serverCallback != nullptr)
    {
        ancsService->_serverCallback->onConnect();
    }

    // Do NOT call startSecurity() here.
    //
    // For a bonded reconnect (the common case — iOS came back in range), iOS
    // fires LL_START_ENC_REQ automatically within ~100-300 ms of connecting.
    // Sending an SMP Security Request at the same moment races with iOS's own
    // encryption initiation, corrupting the SMP state machine.  iOS responds
    // with TERMINATE_IND (disconnect reason 531 = BLE_ERR_REM_USER_CONN_TERM)
    // before encryption can complete, and onAuthenticationComplete never fires.
    //
    // The client task (startClient) will send a Security Request after a short
    // delay if iOS has NOT started encryption on its own — that covers the
    // fresh (un-bonded) connection case where iOS needs to be prompted.

    // Spawn the GATT client task, passing the conn handle so it can call
    // getServer()->getClient(connHandle) to attach to this GAP connection.
    if (ancsService->_clientTaskHandle == nullptr)
    {
        ancsService->_currentClientParam = new ClientParameter(connInfo.getConnHandle(), ancsService);
        xTaskCreatePinnedToCore(&BleService::startClient, "ClientTask", 10000,
            ancsService->_currentClientParam, 5,
            &ancsService->_clientTaskHandle, 0);
    }
}

void BleService::ServerCallback::onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason)
{
    ESP_LOGI(TAG, "Device disconnected (reason=%d)", reason);

    ancsService->_isConnected.store(false);

    // Drain stale pending notifications so they are not replayed on the next connection.
    Notifications.clearPendingNotifications();

    if (ancsService->_clientTaskHandle != nullptr)
    {
        // Wake the client task so it exits its keep-alive loop and cleans up.
        xTaskNotify(ancsService->_clientTaskHandle, 1, eSetValueWithOverwrite);
    }
    else
    {
        // No client task running — restart advertising directly.
        ESP_LOGI(TAG, "No client task - restarting advertising directly");
        ancsService->_restartAdvertising();
    }

    if (ancsService->_serverCallback != nullptr)
    {
        ancsService->_serverCallback->onDisconnect();
    }
}

uint32_t BleService::ServerCallback::onPassKeyDisplay()
{
    uint32_t passkey = NimBLEDevice::getSecurityPasskey();
    ESP_LOGI(TAG, "Passkey display: %06lu", (unsigned long)passkey);
    char buf[7];
    snprintf(buf, sizeof(buf), "%06lu", (unsigned long)passkey);
    Heltec.pairing(buf);
    return passkey;
}

void BleService::ServerCallback::onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t pin)
{
    ESP_LOGI(TAG, "Confirm passkey: %06lu", (unsigned long)pin);
    char buf[7];
    snprintf(buf, sizeof(buf), "%06lu", (unsigned long)pin);
    Heltec.pairing(buf);
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
}

void BleService::ServerCallback::onAuthenticationComplete(NimBLEConnInfo &connInfo)
{
    if (!connInfo.isEncrypted())
    {
        ESP_LOGW(TAG, "Encryption failed (conn_handle=%d, peer_ota=%s, peer_id=%s)",
                 connInfo.getConnHandle(),
                 connInfo.getAddress().toString().c_str(),
                 connInfo.getIdAddress().toString().c_str());

        // Do NOT call NimBLEDevice::deleteBond() here.
        // deleteBond() calls ble_gap_unpair() which TERMINATES the active
        // connection immediately (hci_reason=19).  The encryption wait loop
        // then sees "connection lost" and exits — no chance to retry pairing.
        //
        // NimBLEServer already handles BLE_GAP_EVENT_REPEAT_PAIRING by
        // deleting the old bond and returning RETRY, so fresh SMP will work
        // on the next Security Request if iOS cooperates.
        Heltec.setBLEConnectionState(BLE_PAIRING);
        return;
    }

    ESP_LOGI(TAG, "Authentication successful (encrypted=%d, authenticated=%d, bonded=%d)",
        connInfo.isEncrypted(), connInfo.isAuthenticated(), connInfo.isBonded());
    ancsService->_everEncrypted.store(true);
    ancsService->resetAuthStreak();
    Heltec.setBLEConnectionState(BLE_CONNECTED);
}

/* extern */
BleService Ble = BleService(&NotificationService::NotificationSourceNotifyCallback, &NotificationService::DataSourceNotifyCallback);
