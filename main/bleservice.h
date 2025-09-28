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

#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <BLEDevice.h>

class BLEHIDDevice;

typedef void (*NotificationCallback)(BLERemoteCharacteristic *pBLERemoteCharacteristic,
              uint8_t *pData, size_t length, bool isNotify);

class ANCSServiceServerCallback
{
    public:
        virtual void onConnect() = 0;
        virtual void onDisconnect() = 0;
};

class ANCSServiceClientCallback
{
    public:
        virtual void onConnect() = 0;
        virtual void onDisconnect() = 0;
};

class BleService
{
    public:
        BleService(NotificationCallback notificationSourceCallback, NotificationCallback dataSourceCallback);
        virtual ~BleService();
        void startServer(String const& appName);
        void retrieveNotificationData(uint32_t notifyUUID) const;
        void setServerCallback(ANCSServiceServerCallback *serverCallback);
        void setClientCallback(ANCSServiceClientCallback *clientCallback);
        void setBatteryLevel(uint8_t level);

    private:
        // ANCS
        BLERemoteCharacteristic *_notificationSourceCharacteristic = nullptr;
        BLERemoteCharacteristic *_controlPointCharacteristic = nullptr;
        BLERemoteCharacteristic *_dataSourceCharacteristic = nullptr;
        // Battery
        BLECharacteristic *_batteryLevelCharacteristic = nullptr;

        NotificationCallback _notificationSourceCallback;
        NotificationCallback _dataSourceCallback;
        ANCSServiceServerCallback *_serverCallback = nullptr;
        ANCSServiceClientCallback *_clientCallback = nullptr;
        TaskHandle_t _clientTaskHandle = nullptr;
        BLEHIDDevice *_hidDevice = nullptr;

        static void startClient(void *data);

        class ServerCallback final : public BLEServerCallbacks
        {
            friend class ANCSService;
            public:
                explicit ServerCallback(BleService *ancsService) : ancsService(ancsService) { }
#if defined(CONFIG_BLUEDROID_ENABLED)
                void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *desc) override;
#elif defined(CONFIG_NIMBLE_ENABLED)
                void onConnect(BLEServer *pServer, ble_gap_conn_desc *desc) override;
#endif
                void onDisconnect(BLEServer *pServer) override;

            private:
                BleService *ancsService;
        };

        class ClientCallback final : public BLEClientCallbacks
        {
            friend class BleService;
            public:
                explicit ClientCallback(BleService *ancsService) : ancsService(ancsService) { }
                void onConnect(BLEClient *pClient) override;
                void onDisconnect(BLEClient *pClient) override;
            private:
                BleService *ancsService;
        };

        struct ClientParameter
        {
            BLEAddress bleAddress;
            BleService *bleService;
            ClientParameter(BLEAddress const& bleaddress, BleService *bleservice)
            : bleAddress(bleaddress), bleService(bleservice) { };
        };
};

extern BleService Ble;

#endif /* BLE_SERVICE_H */
