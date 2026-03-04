/**
 * Copyright (c) 2024-2025 Sjofn LLC
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

#include "ancs.h"
#include "hardware.h"
#include "notificationservice.h"
#include "security.h"
#include <NimBLEHIDDevice.h>
#include <host/ble_gap.h>
#include <host/ble_store.h>

static auto TAG = "heltec";

// Bluetooth SIG generic-display appearance value (0x0080).
static constexpr uint16_t BLE_APPEARANCE_GENERIC_DISPLAY = 0x0080;

static const auto ANCS_SERVICE_UUID            = NimBLEUUID("7905F431-B5CE-4E99-A40F-4B1E122D00D0");
static const auto NOTIFICATION_SOURCE_CHR_UUID = NimBLEUUID("9FBF120D-6301-42D9-8C58-25E699A21DBD");
static const auto CONTROL_POINT_CHR_UUID       = NimBLEUUID("69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9");
static const auto DATA_SOURCE_CHR_UUID         = NimBLEUUID("22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB");

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

void BleService::startServer(String const& appName)
{
    // Initialize device
    NimBLEDevice::init(std::string(appName.c_str()));
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
    // iOS will not expose ANCS until the BLE link is encrypted (MITM bonded).
    // onAuthenticationComplete fires from the NimBLE host task; we poll
    // ble_gap_conn_find() so we don't block that task with a semaphore.
    {
        ESP_LOGI(TAG, "Waiting for encryption (up to 10 s)...");
        ble_gap_conn_desc authDesc{};
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
        bool encrypted = false;
        while (xTaskGetTickCount() < deadline)
        {
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
            vTaskDelay(pdMS_TO_TICKS(100));
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

    // ── Step 4: Keep-alive loop ───────────────────────────────────────────────
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

    // Release the server-owned client. This deletes the NimBLEClient object
    // and all cached service / characteristic objects.
    NimBLEDevice::getServer()->deleteClient();
    bleService->_pClient = nullptr;

    // Clear task handle and param before advertising so a fast reconnect
    // cannot create a second task while this one is still in flight.
    bleService->_clientTaskHandle    = nullptr;
    bleService->_currentClientParam  = nullptr;
    delete clientParam;

    ESP_LOGI(TAG, "Restarting advertising after disconnect");
    bleService->_restartAdvertising();
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


////// Server callbacks ///////

void BleService::ServerCallback::onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
{
    ESP_LOGI(TAG, "Device connected %s", connInfo.getAddress().toString().c_str());
    ancsService->_isConnected.store(true);

    if (ancsService->_serverCallback != nullptr)
    {
        ancsService->_serverCallback->onConnect();
    }

    // Send a BLE Security Request to iOS so it initiates SMP pairing immediately.
    // Without this, neither side starts the security handshake and ANCS (which
    // requires an encrypted link) is never exposed by iOS.
    NimBLEDevice::startSecurity(connInfo.getConnHandle());

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
    Heltec.pairing(String(passkey));
    return passkey;
}

void BleService::ServerCallback::onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t pin)
{
    ESP_LOGI(TAG, "Confirm passkey: %06lu", (unsigned long)pin);
    Heltec.pairing(String(pin));
    // With DISPLAY_YES_NO both sides show the same 6-digit number and the user
    // taps YES on iOS. The ESP32 auto-confirms here; iOS handles the user prompt.
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
}

void BleService::ServerCallback::onAuthenticationComplete(NimBLEConnInfo &connInfo)
{
    if (!connInfo.isEncrypted())
    {
        ESP_LOGW(TAG, "Encryption failed — disconnecting (conn_handle=%d)", connInfo.getConnHandle());

        // Only wipe stored bonds after CONSECUTIVE failures reach the threshold.
        // Wiping on every single failure is the root cause of "can't reconnect after
        // going out of range": a transient SMP hiccup on the first reconnect nukes the
        // ESP32 bond while iOS still holds its LTK → every subsequent reconnect gets
        // rejected at the LL layer (iOS sends HCI 0x13 = Remote User Terminated).
        if (ancsService->noteAuthFail())
        {
            int rc = ble_store_clear();
            ESP_LOGW(TAG, "Auth fail threshold reached — cleared all bonds (%d)", rc);
        }

        // Always update the display regardless of streak.
        Heltec.setBLEConnectionState(BLE_DISCONNECTED);
        NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
        return;
    }

    ESP_LOGI(TAG, "Authentication successful (encrypted=%d, authenticated=%d, bonded=%d)",
        connInfo.isEncrypted(), connInfo.isAuthenticated(), connInfo.isBonded());
    ancsService->resetAuthStreak();
    Heltec.setBLEConnectionState(BLE_CONNECTED);
}

bool BleService::noteAuthFail()
{
    int streak = _authFailStreak.fetch_add(1) + 1;
    ESP_LOGW(TAG, "Auth fail streak: %d / %d", streak, AUTH_FAIL_PAIRING_THRESHOLD);

    if (streak >= AUTH_FAIL_PAIRING_THRESHOLD)
    {
        ESP_LOGW(TAG, "MITM pairing requires user action: go to iOS Settings > Bluetooth");
        Heltec.pairing("iOS Settings");
        return true;
    }
    return false;
}

/* extern */
BleService Ble = BleService(&NotificationService::NotificationSourceNotifyCallback, &NotificationService::DataSourceNotifyCallback);
