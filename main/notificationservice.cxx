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

#include "notificationservice.h"

#include "ancs.h"
#include "bleservice.h"
#include "hardware.h"
#include <NimBLERemoteCharacteristic.h>
#include <cinttypes>
#include <cstring>

static const char* TAG = "notify";

NotificationService::NotificationService()
:   notificationCount(0)
,   mEventQueue(nullptr)
,   mMutex(xSemaphoreCreateMutex())
,   cancelledCount(0)
{
    for (size_t i = 0; i < notificationListSize; i++)
    {
        notificationList[i].key = 0;
    }

    // One queue for all ANCS events: pending UUID fetches, DataSource packets, and
    // NotificationSource packets.  A single queue allows xQueueReset() on disconnect
    // without the QueueSet-ownership complications of two separate queues.
    mEventQueue = xQueueCreate(eventQueueSize, sizeof(ancs_event_t));
}

NotificationService::~NotificationService()
{
    if (mEventQueue != nullptr)
    {
        vQueueDelete(mEventQueue);
    }
    if (mMutex != nullptr)
    {
        vSemaphoreDelete(mMutex);
    }
}

int NotificationService::findNotificationIndex(uint32_t uuid) const
{
    for (size_t i = 0; i < notificationListSize; i++)
    {
        if (notificationList[i].key == uuid)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

/* static */
void NotificationService::DataSourceNotifyCallback(NimBLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    // This runs on the BTC task. It MUST return immediately so the GATT indication
    // ACK is sent without delay. Do nothing except copy the raw packet to the event
    // queue — all processing happens in the NotificationDescription task.
    if (length < 6 || Notifications.mEventQueue == nullptr) { return; }
    ancs_event_t event;
    event.type   = ancs_event_t::EVT_DATA_SOURCE;
    event.length = (length < sizeof(event.data)) ? (uint8_t)length : (uint8_t)sizeof(event.data);
    memcpy(event.data, pData, event.length);
    xQueueSend(Notifications.mEventQueue, &event, 0); // non-blocking; drop if full
}

/* static */
void NotificationService::NotificationSourceNotifyCallback(NimBLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length,bool isNotify)
{
    // This runs on the BTC task. Must return immediately — see DataSourceNotifyCallback.
    if (length < 8 || Notifications.mEventQueue == nullptr) { return; }
    ancs_event_t event;
    event.type   = ancs_event_t::EVT_NOTIFY_SOURCE;
    event.length = (length < sizeof(event.data)) ? (uint8_t)length : (uint8_t)sizeof(event.data);
    memcpy(event.data, pData, event.length);
    xQueueSend(Notifications.mEventQueue, &event, 0); // non-blocking; drop if full
}

void NotificationService::setNotificationAttribute(uint32_t uuid, uint8_t attributeId, String const& value)
{
    bool shouldNotify = false;

    xSemaphoreTake(mMutex, portMAX_DELAY);
    // Locate the notification — could be in the call slot or the regular list
    notification_def *notification = nullptr;
    if (callingNotification.key == uuid) {
        notification = &callingNotification;
    } else {
        int index = findNotificationIndex(uuid);
        if (index != -1) { notification = &notificationList[index]; }
    }

    if (notification != nullptr) {
        switch (attributeId) {
            case ANCS::NotificationAttributeIDTitle:
                notification->title = value;
                notification->receivedAttributes |= notification_def::ATTR_TITLE;
                Serial.println(value.c_str());
                break;
            case ANCS::NotificationAttributeIDMessage:
                notification->message = value;
                notification->receivedAttributes |= notification_def::ATTR_MESSAGE;
                Serial.println(value.c_str());
                break;
            case ANCS::NotificationAttributeIDDate: {
                tm t = {};
                strptime(value.c_str(), "%Y%m%dT%H%M%S", &t);
                notification->time = mktime(&t);
                notification->receivedAttributes |= notification_def::ATTR_DATE;
                break;
            }
            default:
                break;
        }
        // Mark complete once all three requested attributes have arrived.
        // Using a received-bitmask instead of isEmpty()/time!=0 correctly handles
        // notifications whose message body is intentionally empty (calls, timers, etc.).
        if (!notification->isComplete &&
            (notification->receivedAttributes & notification_def::ATTR_ALL) == notification_def::ATTR_ALL)
        {
            notification->isComplete = true;
            shouldNotify = true;
        }
    }
    xSemaphoreGive(mMutex);

    // Notify outside the mutex — xTaskNotify is safe without a lock
    if (shouldNotify) {
        Heltec.notifyDraw(Hardware::DRAW_NOTIFY);
    }
}

bool NotificationService::removeIfCall(uint32_t uuid)
{
    bool wasCall = false;
    xSemaphoreTake(mMutex, portMAX_DELAY);
    if (callingNotification.key == uuid) {
        callingNotification.key = 0;
        wasCall = true;
    }
    xSemaphoreGive(mMutex);

    if (wasCall) {
        Heltec.notifyDraw(Hardware::DRAW_NOTIFY);
    }
    return wasCall;
}

bool NotificationService::resetForUpdate(uint32_t uuid)
{
    // Returns true (and resets fields) ONLY if the notification was already complete.
    // If isComplete is false, a re-fetch is already in flight — returning false prevents
    // adding the same UUID to the pending queue again (key deduplication for iOS
    // EventIDNotificationModified floods, e.g. active call duration updates).
    xSemaphoreTake(mMutex, portMAX_DELAY);
    bool shouldRequeue = false;
    notification_def *notification = nullptr;
    if (callingNotification.key == uuid) {
        notification = &callingNotification;
    } else {
        int index = findNotificationIndex(uuid);
        if (index != -1) { notification = &notificationList[index]; }
    }
    if (notification != nullptr && notification->isComplete) {
        bool wasCall = notification->isCall(); // type survives reset()
        notification->reset();
        // For active calls iOS fires EventIDNotificationModified every second.
        // Preserve showed=true after each re-fetch so the caller-ID screen is
        // only raised once (when the call first arrives) and not every second.
        if (wasCall) { notification->showed = true; }
        shouldRequeue = true;
    }
    xSemaphoreGive(mMutex);
    return shouldRequeue;
}

void NotificationService::addPendingNotification(uint32_t uuid)
{
    if (mEventQueue == nullptr) { return; }
    ancs_event_t event;
    event.type   = ancs_event_t::EVT_PENDING_UUID;
    event.length = 4;
    event.data[0] = (uint8_t)(uuid);
    event.data[1] = (uint8_t)(uuid >> 8);
    event.data[2] = (uint8_t)(uuid >> 16);
    event.data[3] = (uint8_t)(uuid >> 24);
    xQueueSend(mEventQueue, &event, 0);
}

void NotificationService::clearPendingNotifications()
{
    if (mEventQueue != nullptr)
    {
        xQueueReset(mEventQueue);
    }
}

void NotificationService::addNotification(notification_def const& notification, bool isCalling)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    if (isCalling)
    {
        callingNotification = notification;
        callingNotification.fetchStartTime = xTaskGetTickCount();
    }
    else
    {
        int targetIndex = -1;

        // Pass 1: find an empty slot (key == 0)
        for (size_t i = 0; i < notificationListSize; i++)
        {
            if (notificationList[i].key == 0)
            {
                targetIndex = static_cast<int>(i);
                break;
            }
        }

        if (targetIndex != -1)
        {
            notificationCount++;
        }
        else
        {
            // Pass 2: prefer evicting an already-showed notification (safe to discard).
            // This avoids clobbering a not-yet-displayed notification with a new one.
            for (size_t i = 0; i < notificationListSize; i++)
            {
                if (notificationList[i].showed)
                {
                    targetIndex = static_cast<int>(i);
                    break;
                }
            }
            // Pass 3: no showed slot — fall back to slot 0 (oldest by position).
            if (targetIndex == -1)
            {
                targetIndex = 0;
            }
            // Eviction: count stays the same (replacing one entry with another).
        }

        notificationList[targetIndex] = notification;
        notificationList[targetIndex].fetchStartTime = xTaskGetTickCount();
    }
    xSemaphoreGive(mMutex);
}

size_t NotificationService::getNotificationCount() const
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    size_t count = notificationCount;
    xSemaphoreGive(mMutex);
    return count;
}

notification_def* NotificationService::getNotificationByIndex(size_t index)
{
    size_t currentIndex = 0;
    for (size_t i = 0; i < notificationListSize; i++)
    {
        if (notificationList[i].key != 0)
        {
            if (currentIndex == index)
            {
                return &notificationList[i];
            }
            currentIndex++;
        }
    }
    return nullptr;
}

bool NotificationService::takeNotificationByIndex(size_t index, notification_def& out)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    bool found = false;
    size_t currentIndex = 0;
    for (size_t i = 0; i < notificationListSize; i++)
    {
        if (notificationList[i].key != 0)
        {
            if (currentIndex == index)
            {
                if (!notificationList[i].showed && notificationList[i].isComplete)
                {
                    out = notificationList[i];
                    notificationList[i].showed = true;
                    found = true;
                }
                break;
            }
            currentIndex++;
        }
    }
    xSemaphoreGive(mMutex);
    return found;
}

bool NotificationService::exists(uint32_t uuid) const
{
    if (callingNotification.key == uuid)
    {
        return true;
    }
    return findNotificationIndex(uuid) != -1;
}

notification_def* NotificationService::getNotification(uint32_t uuid)
{
    if (callingNotification.key == uuid)
    {
        return &callingNotification;
    }
    
    int index = findNotificationIndex(uuid);
    if (index != -1)
    {
        return &notificationList[index];
    }
    return nullptr;
}

void NotificationService::removeCallNotification()
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    callingNotification.key = 0;
    xSemaphoreGive(mMutex);
}

bool NotificationService::isCallingNotification() const
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    bool result = callingNotification.key != 0;
    xSemaphoreGive(mMutex);
    return result;
}

bool NotificationService::takeCallingNotification(notification_def& out)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    bool found = false;
    if (callingNotification.key != 0 &&
        callingNotification.isComplete &&
        !callingNotification.showed)
    {
        out = callingNotification;
        callingNotification.showed = true;
        found = true;
    }
    xSemaphoreGive(mMutex);
    return found;
}

bool NotificationService::removeNotification(uint32_t uuid)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    int index = findNotificationIndex(uuid);
    if (index != -1)
    {
        notificationList[index].reset();
        notificationList[index].key = 0;
        notificationList[index].type = APP_UNKNOWN;
        notificationCount--;
    }
    xSemaphoreGive(mMutex);
    return index != -1;
}

void NotificationService::addCancelledUUID(uint32_t uuid)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    // Deduplicate — don't add if already present.
    bool found = false;
    for (size_t i = 0; i < cancelledCount; i++) {
        if (cancelledUUIDs[i] == uuid) { found = true; break; }
    }
    if (!found && cancelledCount < cancelledSetSize) {
        cancelledUUIDs[cancelledCount++] = uuid;
    } else if (!found) {
        // Set is full — overwrite the oldest entry (index 0) by rotating.
        // In practice this should never happen (size 16 is very generous).
        memmove(&cancelledUUIDs[0], &cancelledUUIDs[1], (cancelledSetSize - 1) * sizeof(uint32_t));
        cancelledUUIDs[cancelledSetSize - 1] = uuid;
        ESP_LOGW(TAG, "cancelledUUIDs set full — oldest entry evicted");
    }
    xSemaphoreGive(mMutex);
}

bool NotificationService::consumeCancelledUUID(uint32_t uuid)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    bool found = false;
    for (size_t i = 0; i < cancelledCount; i++) {
        if (cancelledUUIDs[i] == uuid) {
            // Shift remaining entries down to fill the gap.
            memmove(&cancelledUUIDs[i], &cancelledUUIDs[i + 1],
                    (cancelledCount - i - 1) * sizeof(uint32_t));
            cancelledCount--;
            found = true;
            break;
        }
    }
    xSemaphoreGive(mMutex);
    return found;
}

void NotificationService::markFetchStart(uint32_t uuid)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    notification_def *notification = nullptr;
    if (callingNotification.key == uuid) {
        notification = &callingNotification;
    } else {
        int index = findNotificationIndex(uuid);
        if (index != -1) { notification = &notificationList[index]; }
    }
    if (notification != nullptr) {
        notification->fetchStartTime = xTaskGetTickCount();
    }
    xSemaphoreGive(mMutex);
}

void NotificationService::resetIfStale(uint32_t uuid, TickType_t timeoutTicks)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    notification_def *notification = nullptr;
    if (callingNotification.key == uuid) {
        notification = &callingNotification;
    } else {
        int index = findNotificationIndex(uuid);
        if (index != -1) { notification = &notificationList[index]; }
    }
    if (notification != nullptr && !notification->isComplete &&
        notification->fetchStartTime != 0 &&
        (xTaskGetTickCount() - notification->fetchStartTime) > timeoutTicks)
    {
        ESP_LOGW(TAG, "Fetch for UUID %08lx timed out — resetting for re-fetch", uuid);
        notification->reset();
    }
    xSemaphoreGive(mMutex);
}


NotificationDescription::NotificationDescription(String const& name, uint16_t stack_size)
:   Task(name, stack_size)
{ }

// ---------------------------------------------------------------------------
// Private helpers — called from processNextEvent(), NOT from BLE callbacks.
// It is safe to take the mutex and allocate heap here because we are in the
// NotificationDescription task, not the BTC task.
// ---------------------------------------------------------------------------

void NotificationService::handleDataSourceEvent(const uint8_t* pData, uint8_t length)
{
    // Need at minimum: command(1) + uid(4) + attrId(1) = 6 bytes
    if (length < 6) { return; }

    uint32_t messageId = (uint32_t)pData[1]
                       | ((uint32_t)pData[2] << 8)
                       | ((uint32_t)pData[3] << 16)
                       | ((uint32_t)pData[4] << 24);

    // Attribute data starts at byte 8; value may be empty if length <= 8
    String message;
    for (uint8_t i = 8; i < length; i++)
    {
        message += static_cast<char>(pData[i]);
    }

    if (exists(messageId))
    {
        setNotificationAttribute(messageId, pData[5], message);
    }
    else if (pData[5] == ANCS::NotificationAttributeIDAppIdentifier)
    {
        if (AppList.isAllowedApplication(message))
        {
            const application_def applicationType = AppList.getApplicationId(message);
            notification_def notification;
            notification.type = applicationType;
            notification.key  = messageId;
            addNotification(notification, notification.isCall());
            ESP_LOGI(TAG, "Message from %s added", message.c_str());
        }
        else
        {
            ESP_LOGI(TAG, "Message from %s suppressed", message.c_str());
        }
    }
}

void NotificationService::handleNotificationSourceEvent(const uint8_t* pData, uint8_t length)
{
    if (length < 8) { return; }

    uint32_t messageId = (uint32_t)pData[4]
                       | ((uint32_t)pData[5] << 8)
                       | ((uint32_t)pData[6] << 16)
                       | ((uint32_t)pData[7] << 24);

    if (pData[0] == ANCS::EventIDNotificationRemoved)
    {
        // If the UUID is not in any list it's still in the event queue waiting to be
        // fetched.  Record it as cancelled so handleDataSourceEvent doesn't re-add it
        // after the eventual fetch runs.
        if (!removeIfCall(messageId) && !removeNotification(messageId))
        {
            addCancelledUUID(messageId);
        }
    }
    else if (pData[0] == ANCS::EventIDNotificationModified)
    {
        // Only re-queue if the previous fetch completed. If isComplete is false a
        // re-fetch is already in flight — skip to avoid flooding the queue (iOS sends
        // Modified every second for active calls, call timers, etc.).
        if (resetForUpdate(messageId))
        {
            addPendingNotification(messageId);
        }
    }
    else if (pData[0] == ANCS::EventIDNotificationAdded)
    {
        addPendingNotification(messageId);
    }
}

void NotificationService::processNextEvent()
{
    static constexpr TickType_t FETCH_TIMEOUT = pdMS_TO_TICKS(30000);

    if (mEventQueue == nullptr) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }

    ancs_event_t event;
    if (xQueueReceive(mEventQueue, &event, portMAX_DELAY) != pdTRUE) { return; }

    switch (event.type)
    {
        case ancs_event_t::EVT_PENDING_UUID:
        {
            uint32_t uuid = (uint32_t)event.data[0]
                          | ((uint32_t)event.data[1] << 8)
                          | ((uint32_t)event.data[2] << 16)
                          | ((uint32_t)event.data[3] << 24);

            if (consumeCancelledUUID(uuid))
            {
                ESP_LOGI(TAG, "Skipping cancelled UUID %08" PRIx32, uuid);
                break;
            }
            if (Ble.isConnected() && uuid != 0)
            {
                resetIfStale(uuid, FETCH_TIMEOUT);
                markFetchStart(uuid);
                Ble.retrieveNotificationData(uuid);
            }
            break;
        }
        case ancs_event_t::EVT_DATA_SOURCE:
            handleDataSourceEvent(event.data, event.length);
            break;
        case ancs_event_t::EVT_NOTIFY_SOURCE:
            handleNotificationSourceEvent(event.data, event.length);
            break;
        default:
            break;
    }
}

void NotificationDescription::run(void *data)
{
    // Dispatch ANCS events one at a time. processNextEvent() blocks until an event
    // is available — no busy-waiting, no polling delay, no mutex held across a sleep.
    while (true)
    {
        Notifications.processNextEvent();
    }
}

/* extern */
NotificationService Notifications;
NotificationDescription NotificationReceiver("NotificationReceiver", 50000);