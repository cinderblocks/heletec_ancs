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
#include "notificationservice.h"
#include "security.h"
#include <BLE2902.h>
#include <BLEHIDDevice.h>

static auto TAG = "heltec";

static const auto ANCS_SERVICE_UUID = BLEUUID("7905F431-B5CE-4E99-A40F-4B1E122D00D0");
static const auto NOTIFICATION_SOURCE_CHR_UUID = BLEUUID("9FBF120D-6301-42D9-8C58-25E699A21DBD");
static const auto CONTROL_POINT_CHR_UUID = BLEUUID("69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9");
static const auto DATA_SOURCE_CHR_UUID = BLEUUID("22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB");

//static const auto ANS_SERVICE_UUID = BLEUUID(static_cast<uint16_t>(0x1811));

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
    if (_clientCb != nullptr) {
        delete _clientCb;
        _clientCb = nullptr;
    }
    if (_gattcDoneSem != nullptr) {
        vSemaphoreDelete(_gattcDoneSem);
        _gattcDoneSem = nullptr;
    }
}


void BleService::startServer(String const& appName)
{
    // Initialize device
    BLEDevice::init(appName);
    BLEDevice::setPower(ESP_PWR_LVL_P20);

    // Create the semaphore used to gate startAdvertising() until the GATTC app has been
    // unregistered.  ClientCallback::onDisconnect gives it; the client task takes it.
    if (_gattcDoneSem == nullptr) {
        _gattcDoneSem = xSemaphoreCreateBinary();
    }
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallback(this));
    // Advertising restart is handled by the client task after proper GATTC cleanup.
    // advertiseOnDisconnect(true) fires esp_ble_gap_start_advertising() directly
    // from inside ESP_GATTS_DISCONNECT_EVT on the BTC task, but at that point
    // ESP_GATTC_DISCONNECT_EVT has not yet fired, so the GATTC interface is still
    // registered and the advertising call returns ESP_ERR_INVALID_STATE.
    pServer->advertiseOnDisconnect(false);
#if defined(CONFIG_BLUEDROID_ENABLED)
    BLESecurity::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
#endif //defined(CONFIG_BLUEDROID_ENABLED)
    BLEDevice::setSecurityCallbacks(&Security);

    _hidDevice = new BLEHIDDevice(pServer);
    _hidDevice->manufacturer()->setValue("Sjofn LLC");
    _hidDevice->outputReport(0x01);
    _hidDevice->inputReport(0x02);
    _hidDevice->pnp(0x00, 0x00C3 ,0xffff, 0x0001);
    _hidDevice->hidInfo(0x00, 0x01);
    _hidDevice->startServices();
    _hidDevice->setBatteryLevel(100);

    // Soliciting ANCS
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAppearance(ESP_BLE_APPEARANCE_GENERIC_DISPLAY);
    pAdvertising->addServiceUUID(_hidDevice->deviceInfo()->getUUID());
    pAdvertising->addServiceUUID(_hidDevice->batteryService()->getUUID());
    pAdvertising->addServiceUUID(_hidDevice->hidService()->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // 7.5ms
    pAdvertising->setMaxPreferred(0x10); // 15ms

    //Start advertising
    pAdvertising->start();

    // Create the client callback once - reused for every connection
    _clientCb = new ClientCallback(this);
}

/* static */
void BleService::startClient(void *data)
{
    if (data == nullptr) { return; }
    ESP_LOGI(TAG, "Starting client");
    uint8_t v[] = {0x1, 0x0};
    auto clientParam = static_cast<ClientParameter *>(data);

#if defined(CONFIG_BLUEDROID_ENABLED)
    BLESecurity::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
#endif //defined(CONFIG_BLUEDROID_ENABLED)
    BLEDevice::setSecurityCallbacks(&Security);
#if defined(CONFIG_BLUEDROID_ENABLED)
    // Use bool overload - this is the one that sets m_securityEnabled=true
    // The uint8_t overload does NOT set m_securityEnabled, so startSecurity() never runs
    BLESecurity::setAuthenticationMode(true, true, true); // bonding=true, mitm=true, sc=true
    BLESecurity::setCapability(ESP_IO_CAP_IO);
    BLESecurity::setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    BLESecurity::setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
#elif defined(CONFIG_NIMBLE_ENABLED)
    BLESecurity::setAuthenticationMode(true, true, true);
    BLESecurity::setCapability(BLE_HS_IO_DISPLAY_ONLY);
    BLESecurity::setInitEncryptionKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    BLESecurity::setRespEncryptionKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
#endif

    BLEClient *pClient = BLEDevice::createClient();
    clientParam->bleService->_pClient = pClient;
    pClient->setClientCallbacks(clientParam->bleService->_clientCb);
    pClient->connect(clientParam->bleAddress);

    BLERemoteService *pAncsService = pClient->getService(ANCS_SERVICE_UUID);
    if (pAncsService == nullptr)
    {
        ESP_LOGW(TAG, "Failed to find ANCS service on peer");
        goto EndTask;
    }
    clientParam->bleService->_notificationSourceCharacteristic =
        pAncsService->getCharacteristic(NOTIFICATION_SOURCE_CHR_UUID);
    if (clientParam->bleService->_notificationSourceCharacteristic == nullptr)
    {
        ESP_LOGW(TAG, "Failed to find notification source characteristic");
        goto EndTask;
    }
    clientParam->bleService->_controlPointCharacteristic =
        pAncsService->getCharacteristic(CONTROL_POINT_CHR_UUID);
    if (clientParam->bleService->_controlPointCharacteristic == nullptr)
    {
        ESP_LOGW(TAG, "Failed to find our control point characteristic");
        goto EndTask;
    }
    clientParam->bleService->_dataSourceCharacteristic =
        pAncsService->getCharacteristic(DATA_SOURCE_CHR_UUID);
    if (clientParam->bleService->_dataSourceCharacteristic == nullptr)
    {
        ESP_LOGW(TAG, "Failed to find datasource characteristic");
        goto EndTask;
    }
    clientParam->bleService->_notificationSourceCharacteristic->registerForNotify(
        clientParam->bleService->_notificationSourceCallback);
    clientParam->bleService->_notificationSourceCharacteristic->getDescriptor(
        BLEUUID(static_cast<uint16_t>(0x2902)))->writeValue(v, 2, true);
    clientParam->bleService->_dataSourceCharacteristic->registerForNotify(
        clientParam->bleService->_dataSourceCallback);
    clientParam->bleService->_dataSourceCharacteristic->getDescriptor(
        BLEUUID(static_cast<uint16_t>(0x2902)))->writeValue(v, 2, true);

    // Keep-alive loop.  Woken by xTaskNotify from either ServerCallback::onDisconnect
    // or ClientCallback::onDisconnect, whichever fires first.  100 ms poll catches any
    // disconnect that slips through without a notification (e.g. link-loss timeout).
    while (true)
    {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) != 0)
            break;  // woken by a disconnect notification
        if (!pClient->isConnected())
            break;  // polled disconnect
    }

EndTask:
    ESP_LOGI(TAG, "Ending ClientTask");

    // Capture the service pointer now – we will free clientParam before advertising.
    BleService *bleService = clientParam->bleService;

    // If the peer was still connected when we fell out of the loop (e.g. goto EndTask
    // from a setup failure), disconnect now.  This queues ESP_GATTC_DISCONNECT_EVT,
    // which calls esp_ble_gattc_app_unregister() and then gives _gattcDoneSem via
    // ClientCallback::onDisconnect.
    if (pClient->isConnected())
    {
        pClient->disconnect();
    }

    // Wait until ClientCallback::onDisconnect signals that esp_ble_gattc_app_unregister()
    // has been posted to the BTC task queue.  This guarantees that when we post
    // START_ADV to the BTC queue below, UNREGISTER_APP is already ahead of it, so
    // esp_ble_gap_start_advertising() will not see ESP_ERR_INVALID_STATE on either core.
    // 3 s timeout is a safety net; in practice the event arrives within tens of ms.
    if (bleService->_gattcDoneSem != nullptr)
    {
        if (xSemaphoreTake(bleService->_gattcDoneSem, pdMS_TO_TICKS(3000)) != pdTRUE)
        {
            ESP_LOGW(TAG, "Timed out waiting for GATTC disconnect event");
        }
    }

    // Now it is safe to delete the BLEClient (GATTC app is already unregistered).
    // Null the characteristic pointers FIRST so that any concurrent call to
    // retrieveNotificationData() that slipped past the _isConnected check (set
    // in ServerCallback::onDisconnect, which fires before this point) will see
    // nullptr and return immediately rather than accessing freed memory.
    bleService->_notificationSourceCharacteristic = nullptr;
    bleService->_controlPointCharacteristic = nullptr;
    bleService->_dataSourceCharacteristic = nullptr;
    if (bleService->_pClient != nullptr)
    {
        delete pClient;
        bleService->_pClient = nullptr;
    }

    // Clear the task handle and param *before* startAdvertising() so a fast reconnect
    // cannot create a second client task while we are still running.
    bleService->_clientTaskHandle = nullptr;
    bleService->_currentClientParam = nullptr;
    delete clientParam;
    clientParam = nullptr;

    // Restart advertising.  _restartAdvertising() calls reset() first so any stuck
    // m_advConfiguring flag left by a previous interrupted config chain is cleared.
    ESP_LOGI(TAG, "Restarting advertising after disconnect");
    bleService->_restartAdvertising();
    vTaskDelete(nullptr);
}

void BleService::_restartAdvertising()
{
    // BLEAdvertising::start() silently returns true (only log_w) when m_advConfiguring
    // is stuck as true — e.g. when an async config chain was interrupted by a fast
    // reconnect during the initial advertising setup.  reset() is the only public API
    // that forcefully clears both m_advConfiguring and m_advDataSet.
    //
    // reset() does NOT clear m_serviceUUIDs, so the UUIDs added in startServer() are
    // reused by start() automatically via the full async config chain:
    //   esp_ble_gap_config_adv_data → ADV_DATA_SET_COMPLETE
    //   → esp_ble_gap_config_adv_data(scanResp) → SCAN_RSP_DATA_SET_COMPLETE
    //   → esp_ble_gap_start_advertising
    //
    // This must be called AFTER _gattcDoneSem has been taken, which guarantees that
    // esp_ble_gattc_app_unregister() is already in the BTC task queue ahead of our
    // GAP commands, preventing ESP_ERR_INVALID_STATE on ESP_GAP_BLE_ADV_START_COMPLETE.
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->reset();                                       // clears stuck state
    pAdvertising->setAppearance(ESP_BLE_APPEARANCE_GENERIC_DISPLAY);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);                         // re-apply after reset
    pAdvertising->setMaxPreferred(0x10);
    pAdvertising->start();
}

void BleService::retrieveNotificationData(uint32_t notifyUUID) const
{
        // Belt-and-suspenders: callers should already check isConnected(), but guard
        // here too in case _controlPointCharacteristic was nulled by EndTask.
        if (_controlPointCharacteristic == nullptr) { return; }

        uint8_t uuid[4];
        uuid[0] = notifyUUID;
        uuid[1] = notifyUUID >> 8;
        uuid[2] = notifyUUID >> 16;
        uuid[3] = notifyUUID >> 24;
        uint8_t vIdentifier[] = {0x0,uuid[0],uuid[1],uuid[2],uuid[3], ANCS::NotificationAttributeIDAppIdentifier};
        _controlPointCharacteristic->writeValue(vIdentifier, 6, true);
        uint8_t vTitle[] = {0x0,uuid[0],uuid[1],uuid[2],uuid[3], ANCS::NotificationAttributeIDTitle, 0x0, 0x10};
        _controlPointCharacteristic->writeValue(vTitle, 8, true);
        uint8_t vMessage[] = {0x0,uuid[0],uuid[1],uuid[2],uuid[3], ANCS::NotificationAttributeIDMessage, 0x0, 0x10};
        _controlPointCharacteristic->writeValue(vMessage, 8, true);
        uint8_t vDate[] = {0x0,uuid[0],uuid[1],uuid[2],uuid[3], ANCS::NotificationAttributeIDDate};
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


////// Callbacks ///////

#if defined(CONFIG_BLUEDROID_ENABLED)
void BleService::ServerCallback::onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *desc)
#elif defined(CONFIG_NIMBLE_ENABLED)
void BleService::ServerCallback::onConnect(BLEServer *pServer, ble_gap_conn_desc *desc)
#endif
{
#if defined(CONFIG_BLUEDROID_ENABLED)
    const auto peerAddress = BLEAddress(desc->connect.remote_bda);
#elif defined(CONFIG_NIMBLE_ENABLED)
    auto peerAddress = BLEAddress(desc->peer_ota_addr);
#endif
    ESP_LOGI(TAG, "Device connected %s", peerAddress.toString().c_str());
    ancsService->_isConnected.store(true);
    // Drain any stale semaphore signal left over from the previous disconnect cycle.
    if (ancsService->_gattcDoneSem != nullptr) {
        xSemaphoreTake(ancsService->_gattcDoneSem, 0);
    }
    if (ancsService->_serverCallback != nullptr)
    {
        ancsService->_serverCallback->onConnect();
    }
    if (ancsService->_clientTaskHandle == nullptr)
    {
        ancsService->_currentClientParam = new ClientParameter(peerAddress, ancsService);
        xTaskCreatePinnedToCore(&BleService::startClient, "ClientTask", 10000,
            ancsService->_currentClientParam, 5,
            &ancsService->_clientTaskHandle, 0);
    }
}

void BleService::ServerCallback::onDisconnect(BLEServer *pServer)
{
    ESP_LOGI(TAG, "Device disconnected");

    // Mark as disconnected FIRST so NotificationDescription::run() immediately stops
    // calling retrieveNotificationData().  This is the primary guard against the
    // use-after-free of _controlPointCharacteristic that was crashing the BTC task.
    ancsService->_isConnected.store(false);

    // Drain any notifications that were queued while connected so they are not
    // processed on the next connection with stale UUIDs.
    Notifications.clearPendingNotifications();

    // Wake the client task so it exits its keep-alive loop.  The task will wait for
    // ClientCallback::onDisconnect (which signals that esp_ble_gattc_app_unregister()
    // is in the BTC queue) before calling startAdvertising().
    //
    // IMPORTANT: do NOT null _clientTaskHandle here.  Nulling prematurely would:
    //   (a) allow onConnect to create a new task while the old one is still cleaning up,
    //   (b) cause the else-branch below to call startAdvertising() while GATTC is still
    //       registered, returning ESP_ERR_INVALID_STATE on the dual-core ESP32-S3.
    // The task owns _clientTaskHandle and nulls it only after startAdvertising() returns.
    if (ancsService->_clientTaskHandle != nullptr)
    {
        xTaskNotify(ancsService->_clientTaskHandle, 1, eSetValueWithOverwrite);
    }
    else if (ancsService->_pClient == nullptr)
    {
        // No task and no GATTC client — the GATTC app is already unregistered, so
        // it is safe to restart advertising directly from this BTC task callback.
        ESP_LOGI(TAG, "No client task - restarting advertising directly");
        ancsService->_restartAdvertising();
    }
    // else: _pClient != null but no task handle — the task exited the keep-alive loop
    // via the polling path and is currently in EndTask cleaning up.  It will call
    // startAdvertising() itself after taking _gattcDoneSem.

    if (ancsService->_serverCallback != nullptr)
    {
        ancsService->_serverCallback->onDisconnect();
    }
}

void BleService::ClientCallback::onConnect(BLEClient *pClient)
{
    if (pClient != nullptr) {
        ESP_LOGI(TAG, "Device client connected %s", pClient->getPeerAddress().toString().c_str());
    }
    if (ancsService->_clientCallback != nullptr)
    {
        ancsService->_clientCallback->onConnect();
    }
}

void BleService::ClientCallback::onDisconnect(BLEClient *pClient)
{
    if (pClient != nullptr) {
        ESP_LOGI(TAG, "Device Client disconnected %s", pClient->getPeerAddress().toString().c_str());
    }

    // This callback fires from inside ESP_GATTC_DISCONNECT_EVT, which means
    // esp_ble_gattc_app_unregister() has already been posted to the BTC task queue.
    // Signal the client task that it is now safe to call startAdvertising(): by the
    // time the task posts START_ADV to the BTC queue, UNREGISTER_APP is already ahead
    // of it in the queue, guaranteeing correct ordering on both single- and dual-core.
    if (ancsService->_gattcDoneSem != nullptr) {
        xSemaphoreGive(ancsService->_gattcDoneSem);
    }
    // Also wake the keep-alive loop in case ServerCallback::onDisconnect hasn't fired yet.
    if (ancsService->_clientTaskHandle != nullptr) {
        xTaskNotify(ancsService->_clientTaskHandle, 1, eSetValueWithOverwrite);
    }

    if (ancsService->_clientCallback != nullptr)
    {
        ancsService->_clientCallback->onDisconnect();
    }
}

/* extern */
BleService Ble = BleService(&NotificationService::NotificationSourceNotifyCallback, &NotificationService::DataSourceNotifyCallback);
