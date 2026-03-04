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

#ifndef NOTIFICATION_SERVICE_H_
#define NOTIFICATION_SERVICE_H_

#include "applist.h"
#include "task.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <time.h>

class NimBLERemoteCharacteristic;

/**
 * Combined event type posted to mEventQueue from BLE callbacks and addPendingNotification.
 * Using one queue (instead of two + a QueueSet) lets us call xQueueReset() safely on
 * disconnect and keeps the BLE callbacks non-blocking — they only memcpy + xQueueSend.
 */
struct ancs_event_t {
    enum : uint8_t { EVT_PENDING_UUID = 0, EVT_DATA_SOURCE = 1, EVT_NOTIFY_SOURCE = 2 } type;
    uint8_t  length;    // valid bytes in data[]
    uint8_t  data[64];  // LE UUID (4 bytes) for EVT_PENDING_UUID; raw ANCS packet for others
};

struct notification_def
{
    String title = "";
    String message = "";
    time_t time = 0;
    application_def type = APP_UNKNOWN;
    uint32_t key = 0;
    boolean showed = false;
    boolean isComplete = false;
    uint8_t receivedAttributes = 0; // bitmask: bit0=Title, bit1=Message, bit2=Date
    TickType_t fetchStartTime = 0;  // set when fetch begins; used to detect stuck fetches

    static constexpr uint8_t ATTR_TITLE   = (1u << 0);
    static constexpr uint8_t ATTR_MESSAGE = (1u << 1);
    static constexpr uint8_t ATTR_DATE    = (1u << 2);
    static constexpr uint8_t ATTR_ALL     = ATTR_TITLE | ATTR_MESSAGE | ATTR_DATE;

    bool isCall() const { return type == APP_PHONE || type == APP_FACETIME; }

    void reset()
    {
        title = "";
        message = "";
        time = 0;
        showed = false;
        isComplete = false;
        receivedAttributes = 0;
        fetchStartTime = 0;
    }
};

class NotificationService
{
public:
    NotificationService();
    ~NotificationService();

    // Called from NotificationDescription task — blocks until an event is available.
    void processNextEvent();

    void addPendingNotification(uint32_t uuid);
    void clearPendingNotifications();
    void addNotification(notification_def const& notification, bool isCalling);
    void setNotificationAttribute(uint32_t uuid, uint8_t attributeId, String const& value);
    bool resetForUpdate(uint32_t uuid);
    bool removeIfCall(uint32_t uuid);
    bool removeNotification(uint32_t uuid);
    void removeCallNotification();
    void addCancelledUUID(uint32_t uuid);
    bool consumeCancelledUUID(uint32_t uuid);
    void markFetchStart(uint32_t uuid);
    void resetIfStale(uint32_t uuid, TickType_t timeoutTicks);
    bool exists(uint32_t uuid) const;
    [[nodiscard]] bool isCallingNotification() const;
    bool takeCallingNotification(notification_def& out);
    notification_def* getNotification(uint32_t uuid);
    size_t getNotificationCount() const;
    notification_def* getNotificationByIndex(size_t index);
    bool takeNotificationByIndex(size_t index, notification_def& out);

    // BLE callbacks — must return immediately; do NOT take any mutex or allocate heap.
    static void DataSourceNotifyCallback(NimBLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify);
    static void NotificationSourceNotifyCallback(NimBLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify);

private:
    static constexpr size_t notificationListSize = 16;
    // Combined queue: capacity for pending UUID bursts (64) plus concurrent ANCS attribute
    // responses (up to 4 per UUID) and notification-source events.
    static constexpr size_t eventQueueSize       = 96;
    static constexpr size_t cancelledSetSize     = 16;

    notification_def notificationList[notificationListSize];
    size_t notificationCount;
    QueueHandle_t mEventQueue;
    notification_def callingNotification;
    mutable SemaphoreHandle_t mMutex;

    uint32_t cancelledUUIDs[cancelledSetSize];
    size_t   cancelledCount;

    int  findNotificationIndex(uint32_t uuid) const;
    void handleDataSourceEvent(const uint8_t* data, uint8_t length);
    void handleNotificationSourceEvent(const uint8_t* data, uint8_t length);
};

class NotificationDescription final : public Task
{
public:
    NotificationDescription(String const& name, uint16_t stack_size);
private:
    void run(void *data) override;
};

extern NotificationService Notifications;
extern NotificationDescription NotificationReceiver;

#endif /* NOTIFICATION_SERVICE_H_ */