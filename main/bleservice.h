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

#include <atomic>
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <freertos/semphr.h>

class NimBLEHIDDevice;

typedef void (*NotificationCallback)(NimBLERemoteCharacteristic *pBLERemoteCharacteristic,
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
        bool isConnected() const { return _isConnected.load(); }
        bool noteAuthFail();
        void resetAuthStreak() { _authFailStreak.store(0); }

    private:
        // ANCS remote characteristics (discovered on the iPhone)
        NimBLERemoteCharacteristic *_notificationSourceCharacteristic = nullptr;
        NimBLERemoteCharacteristic *_controlPointCharacteristic = nullptr;
        NimBLERemoteCharacteristic *_dataSourceCharacteristic = nullptr;

        NotificationCallback _notificationSourceCallback;
        NotificationCallback _dataSourceCallback;
        ANCSServiceServerCallback *_serverCallback = nullptr;
        ANCSServiceClientCallback *_clientCallback = nullptr;
        TaskHandle_t _clientTaskHandle = nullptr;
        NimBLEHIDDevice *_hidDevice = nullptr;

        // Pointer to the server-owned GATT client (obtained via getServer()->getClient()).
        // Null when no client task is running.
        NimBLEClient *_pClient = nullptr;

        static void startClient(void *data);

        class ServerCallback final : public NimBLEServerCallbacks
        {
            friend class ANCSService;
            public:
                explicit ServerCallback(BleService *ancsService) : ancsService(ancsService) { }
                void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override;
                void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override;
                void onAuthenticationComplete(NimBLEConnInfo &connInfo) override;
                uint32_t onPassKeyDisplay() override;
                void onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t pin) override;

            private:
                BleService *ancsService;
        };

        struct ClientParameter
        {
            uint16_t connHandle;  // GAP connection handle from the server's onConnect
            BleService *bleService;
            ClientParameter(uint16_t handle, BleService *bleservice)
            : connHandle(handle), bleService(bleservice) { };
        };

        ClientParameter *_currentClientParam = nullptr;

        std::atomic<bool> _isConnected{false};
        std::atomic<int>  _authFailStreak{0};
        static constexpr int AUTH_FAIL_PAIRING_THRESHOLD = 2;

        void _restartAdvertising();
};

extern BleService Ble;

#endif /* BLE_SERVICE_H */
