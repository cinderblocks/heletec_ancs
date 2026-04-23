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

#ifndef NOTIFICATION_SERVICE_H_
#define NOTIFICATION_SERVICE_H_

#include "applist.h"
#include "task.h"
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
    // EVT_PENDING_UUID layout:
    //   data[0..3] : notification UID (LE uint32)
    //   data[4]    : ANCS CategoryID from the NotificationSource packet
    // EVT_DATA_SOURCE / EVT_NOTIFY_SOURCE:
    //   data[0..length-1] : raw ANCS packet bytes
    uint8_t  data[64];
};

struct notification_def
{
    char title[64]    = {};
    char message[128] = {};
    char bundleId[64] = {};  ///< iOS bundle ID — used to resolve display name for custom apps
    time_t time = 0;
    application_def type = APP_UNKNOWN;
    uint32_t key = 0;
    /// ANCS CategoryID from the NotificationSource packet (e.g. CategoryIDIncomingCall=1,
    /// CategoryIDMissedCall=2).  Used to distinguish a ringing incoming call from a
    /// "missed call" entry — both share the com.apple.mobilephone bundle ID.
    uint8_t categoryId = 0;
    bool showed = false;
    bool isComplete = false;
    uint8_t receivedAttributes = 0;
    TickType_t fetchStartTime = 0;

    static constexpr uint8_t ATTR_TITLE   = (1u << 0);
    static constexpr uint8_t ATTR_MESSAGE = (1u << 1);
    static constexpr uint8_t ATTR_DATE    = (1u << 2);
    static constexpr uint8_t ATTR_ALL     = ATTR_TITLE | ATTR_MESSAGE | ATTR_DATE;

    /// Returns true ONLY for an actively ringing incoming call.
    /// Missed call entries (CategoryIDMissedCall) from the same bundle ID must NOT
    /// trigger the call screen — they are regular notifications.
    bool isCall() const
    {
        return (type == APP_PHONE || type == APP_FACETIME)
               && categoryId == 1u; // ANCS::CategoryIDIncomingCall
    }

    void reset()
    {
        title[0]    = '\0';
        message[0]  = '\0';
        bundleId[0] = '\0';
        time = 0;
        categoryId = 0;
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

    /// @param categoryId  ANCS CategoryID from the NotificationSource packet.
    ///                    Stored in EVT_PENDING_UUID data[4] so it survives until
    ///                    handleDataSourceEvent() creates the notification_def.
    void addPendingNotification(uint32_t uuid, uint8_t categoryId = 0);
    void clearPendingNotifications();
    void addNotification(notification_def const& notification, bool isCalling);
    void setNotificationAttribute(uint32_t uuid, uint8_t attributeId, const char* value);
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
    size_t getNotificationCount() const;
    bool takeNotificationByIndex(size_t index, notification_def& out);

    /**
     * Atomically copy all pending (unshowed, complete) non-call notifications
     * into buf[] and mark each as showed, under a single mutex acquisition.
     * Returns the number of notifications copied (≤ maxCount).
     * Used by DrawTask to replace N separate takeNotificationByIndex() calls,
     * reducing mutex churn from O(N) to O(1) and lowering context-switch
     * pressure on CPU 0.
     */
    size_t takeAllPendingNotifications(notification_def* buf, size_t maxCount);

    // BLE callbacks — must return immediately; do NOT take any mutex or allocate heap.
    static void DataSourceNotifyCallback(NimBLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify);
    static void NotificationSourceNotifyCallback(NimBLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify);

private:
    static constexpr size_t notificationListSize = 16;
    // Combined queue: capacity for pending UUID bursts (64) plus concurrent ANCS attribute
    // responses (up to 4 per UUID) and notification-source events.
    // Sized generously: 20 pre-existing notifications × (1 EVT_PENDING_UUID + 4 EVT_DATA_SOURCE)
    // = 100 events; add headroom for simultaneous NotificationSource events.
    static constexpr size_t eventQueueSize       = 160;
    static constexpr size_t cancelledSetSize     = 16;

    notification_def notificationList[notificationListSize];
    size_t notificationCount;
    QueueHandle_t mEventQueue;
    notification_def callingNotification;
    mutable SemaphoreHandle_t mMutex;

    uint32_t cancelledUUIDs[cancelledSetSize];
    size_t   cancelledCount;

    // ── Pending CategoryID map ────────────────────────────────────────────────
    // Maps a notification UUID to its ANCS CategoryID so that handleDataSourceEvent()
    // can correctly classify incoming calls vs. missed calls vs. other notifications
    // even though CategoryID is only available in the NotificationSource packet.
    struct PendingCategoryEntry { uint32_t uuid; uint8_t categoryId; };
    static constexpr size_t pendingCategoryMapSize = 32;
    PendingCategoryEntry pendingCategoryMap[pendingCategoryMapSize];
    size_t pendingCategoryCount = 0;

    /// Store (uuid → categoryId) before the GATT fetch begins.
    void storePendingCategory(uint32_t uuid, uint8_t categoryId);
    /// Remove and return the stored categoryId for uuid (returns 0 if not found).
    uint8_t consumePendingCategory(uint32_t uuid);

    int  findNotificationIndex(uint32_t uuid) const;
    void handleDataSourceEvent(const uint8_t* data, uint8_t length);
    void handleNotificationSourceEvent(const uint8_t* data, uint8_t length);

    // ── Unsafe raw-pointer accessors (internal use only) ─────────────────
    // These return a pointer into the notification list WITHOUT holding mMutex.
    // Callers MUST hold mMutex for the entire lifetime of the returned pointer.
    // Prefer the safe copy-out APIs (takeAllPendingNotifications, takeCallingNotification).
    notification_def* getNotification(uint32_t uuid);
    notification_def* getNotificationByIndex(size_t index);
};

class NotificationDescription final : public Task
{
public:
    NotificationDescription(const char* name, uint16_t stack_size);
private:
    void run(void *data) override;
};

extern NotificationService Notifications;
extern NotificationDescription NotificationReceiver;

#endif /* NOTIFICATION_SERVICE_H_ */