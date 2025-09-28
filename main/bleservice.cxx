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
}


void BleService::startServer(String const& appName)
{
    // Initialize device
    BLEDevice::init(appName);
    BLEDevice::setPower(ESP_PWR_LVL_P9);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallback(this));
#if defined(CONFIG_BLUEDROID_ENABLED)
    BLESecurity::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
#endif //defined(CONFIG_BLUEDROID_ENABLED)
    pServer->advertiseOnDisconnect(true);
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
    BLEDevice::startAdvertising();

    // Set security
#if defined(CONFIG_BLUEDROID_ENABLED)
    BLESecurity::setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
    BLESecurity::setCapability(ESP_IO_CAP_IO);
    BLESecurity::setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    BLESecurity::setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
#elif defined(CONFIG_NIMBLE_ENABLED)
    BLESecurity::setAuthenticationMode(BLE_SM_PAIR_AUTHREQ_SC);
    BLESecurity::setCapability(BLE_HS_IO_DISPLAY_ONLY);
    BLESecurity::setInitEncryptionKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    BLESecurity::setRespEncryptionKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
#endif

    //Start advertising
    pAdvertising->start();
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
    BLESecurity::setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
    BLESecurity::setCapability(ESP_IO_CAP_IO);
    BLESecurity::setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    BLESecurity::setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
#elif defined(CONFIG_NIMBLE_ENABLED)
    BLESecurity::setAuthenticationMode(BLE_SM_PAIR_AUTHREQ_SC);
    BLESecurity::setCapability(BLE_HS_IO_DISPLAY_ONLY);
    BLESecurity::setInitEncryptionKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    BLESecurity::setRespEncryptionKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
#endif

    BLEClient *pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallback(clientParam->bleService));
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
    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
EndTask:
    ESP_LOGI(TAG, "Ending ClientTask");
    clientParam->bleService->_clientTaskHandle = nullptr;
    delete clientParam;
    vTaskDelete(nullptr);
}

void BleService::retrieveNotificationData(uint32_t notifyUUID) const
{
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
    if (ancsService->_serverCallback != nullptr)
    {
        ancsService->_serverCallback->onConnect();
    }
    if (ancsService->_clientTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(&BleService::startClient, "ClientTask", 10000,
            new ClientParameter(peerAddress, ancsService), 5,
            &ancsService->_clientTaskHandle, 0);
    }
}

void BleService::ServerCallback::onDisconnect(BLEServer *pServer)
{
    if (ancsService->_clientTaskHandle != nullptr)
    {
        ::vTaskDelete(ancsService->_clientTaskHandle);
        ancsService->_clientTaskHandle = nullptr;
    }
    ESP_LOGI(TAG, "Device disconnected");
    if (ancsService->_serverCallback != nullptr)
    {
        ancsService->_serverCallback->onDisconnect();
    }
    BLEDevice::startAdvertising();
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
        ESP_LOGI(TAG, "Device Client disconnected %s", pClient->getPeerAddress().toString().c_str() );
    }
    if (ancsService->_clientCallback != nullptr)
    {
        ancsService->_clientCallback->onDisconnect();
    }
    BLEDevice::startAdvertising();
}

/* extern */
BleService Ble = BleService(&NotificationService::NotificationSourceNotifyCallback, &NotificationService::DataSourceNotifyCallback);
