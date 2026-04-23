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
#include "util.h"
#include <NimBLERemoteCharacteristic.h>
#include <algorithm>
#include <cinttypes>
#include <cstring>

static const char* TAG = "notify";

NotificationService::NotificationService()
:   notificationCount(0)
,   mEventQueue(nullptr)
,   mMutex(xSemaphoreCreateMutex())
,   cancelledCount(0)
,   pendingCategoryCount(0)
{
    for (size_t i = 0; i < notificationListSize; i++)
    {
        notificationList[i].key = 0;
    }
    for (size_t i = 0; i < pendingCategoryMapSize; i++)
    {
        pendingCategoryMap[i] = {0, 0};
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

void NotificationService::setNotificationAttribute(uint32_t uuid, uint8_t attributeId, const char* value)
{
    bool shouldNotify = false;
    {
        ScopedLock lock(mMutex);
        notification_def *notification = nullptr;
        if (callingNotification.key == uuid) {
            notification = &callingNotification;
        } else {
            int index = findNotificationIndex(uuid);
            if (index != -1) { notification = &notificationList[index]; }
        }

        if (notification != nullptr) {
            switch (attributeId) {
                case ANCS::NotificationAttributeIDAppIdentifier:
                    // Repopulate bundleId after a reset()-triggered re-fetch.
                    // reset() clears bundleId; without this the display name would
                    // be blank for custom (APP_UNKNOWN) apps after a modification.
                    strncpy(notification->bundleId, value, sizeof(notification->bundleId) - 1);
                    notification->bundleId[sizeof(notification->bundleId) - 1] = '\0';
                    break;
                case ANCS::NotificationAttributeIDTitle:
                    strncpy(notification->title, value, sizeof(notification->title) - 1);
                    notification->title[sizeof(notification->title) - 1] = '\0';
                    notification->receivedAttributes |= notification_def::ATTR_TITLE;
                    ESP_LOGI(TAG, "Title: %s", notification->title);
                    break;
                case ANCS::NotificationAttributeIDMessage:
                    strncpy(notification->message, value, sizeof(notification->message) - 1);
                    notification->message[sizeof(notification->message) - 1] = '\0';
                    notification->receivedAttributes |= notification_def::ATTR_MESSAGE;
                    ESP_LOGI(TAG, "Message: %s", notification->message);
                    break;
                case ANCS::NotificationAttributeIDDate: {
                    tm t = {};
                    strptime(value, "%Y%m%dT%H%M%S", &t);
                    notification->time = mktime(&t);
                    notification->receivedAttributes |= notification_def::ATTR_DATE;
                    break;
                }
                default:
                    break;
            }
            if (!notification->isComplete &&
                (notification->receivedAttributes & notification_def::ATTR_ALL) == notification_def::ATTR_ALL)
            {
                notification->isComplete = true;
                // Pre-existing notifications (present on iOS before this BLE connection)
                // must not be displayed or buzzed — they are not new to the user.
                // Mark showed=true here so takeAllPendingNotifications skips them, and
                // suppress DRAW_NOTIFY entirely to avoid standby-screen flicker during
                // the reconnect burst of potentially dozens of pre-existing fetches.
                if (notification->preExisting) {
                    notification->showed = true;
                    // shouldNotify intentionally left false
                } else {
                    shouldNotify = true;
                }
            }
        }
    } // lock released here

    if (shouldNotify) {
        Heltec.notifyDraw(Hardware::DRAW_NOTIFY);
    }
}

bool NotificationService::removeIfCall(uint32_t uuid)
{
    bool wasCall = false;
    {
        ScopedLock lock(mMutex);
        if (callingNotification.key == uuid) {
            // If the call was fully fetched but not yet shown (e.g., the caller
            // hung up before the draw task woke up), demote it to the regular
            // notification list so it isn't silently lost.  The draw task will
            // then show it via takeAllPendingNotifications as a normal entry
            // (buzzer plays notification tone, not call ringtone, because
            // isCall() will now return false for a demoted entry once
            // categoryId != CategoryIDIncomingCall or we leave it as-is and
            // the list shows it without call-screen treatment).
            if (callingNotification.isComplete && !callingNotification.showed) {
                int targetIndex = -1;
                for (size_t i = 0; i < notificationListSize; i++) {
                    if (notificationList[i].key == 0) { targetIndex = static_cast<int>(i); break; }
                }
                if (targetIndex != -1) {
                    notificationList[targetIndex] = callingNotification;
                    // Force isCall() to return false so the demoted entry is
                    // treated as a regular notification and doesn't re-enter
                    // the call-screen path.
                    notificationList[targetIndex].categoryId = 0;
                    notificationCount++;
                    ESP_LOGI(TAG, "Demoted unshown call %08lx to regular notification list",
                             static_cast<unsigned long>(uuid));
                } else {
                    ESP_LOGW(TAG, "No free slot to demote call %08lx — notification lost",
                             static_cast<unsigned long>(uuid));
                }
            }
            callingNotification.key = 0;
            wasCall = true;
        }
    }
    if (wasCall) {
        Heltec.notifyDraw(Hardware::DRAW_NOTIFY);
    }
    return wasCall;
}

bool NotificationService::resetForUpdate(uint32_t uuid)
{
    ScopedLock lock(mMutex);
    bool shouldRequeue = false;
    notification_def *notification = nullptr;
    if (callingNotification.key == uuid) {
        notification = &callingNotification;
    } else {
        int index = findNotificationIndex(uuid);
        if (index != -1) { notification = &notificationList[index]; }
    }
    if (notification != nullptr && notification->isComplete) {
        bool wasCall = notification->isCall();

        // Re-validate non-call notifications against the current whitelist before
        // re-queuing.  Only custom (user-added) entries can be removed at runtime,
        // so built-in (hardcoded) entries must ALWAYS be allowed through — never
        // discard them, even if bundleId is transiently stale or empty.
        // Guard against an empty bundleId too: if it's blank for any reason, give
        // the re-fetch a chance to repopulate it rather than wrongly suppressing
        // a notification we already accepted once.
        if (!wasCall &&
            notification->bundleId[0] != '\0' &&
            !AppList.isBuiltIn(notification->bundleId) &&
            !AppList.isAllowedApplication(notification->bundleId))
        {
            // Custom app is no longer whitelisted — discard the notification entirely.
            notification->reset();
            notification->key = 0;
            if (notificationCount > 0) { notificationCount--; }
            return false;
        }

        // For calls: iOS sends Modified every second for the call timer.
        // Re-fetching would waste BLE bandwidth and — because showed is set to true
        // below — the updated data would be discarded by takeCallingNotification
        // anyway.  Simply leave the existing (already-shown) call notification in
        // place; the call-state icon in the header stays accurate via
        // isCallingNotification().
        if (wasCall) {
            return false;
        }

        // Preserve the showed flag across reset so that already-displayed
        // notifications (e.g. old missed calls) are re-fetched to keep their
        // in-memory content current, but are NOT re-shown to the user when
        // a call ends or DRAW_NOTIFY fires.  Without this, a Modified event
        // from iOS on any previously-shown notification would clear showed=false,
        // causing it to appear again after the next call ends.
        bool wasShowed = notification->showed;
        notification->reset();
        notification->showed = wasShowed;
        shouldRequeue = true;
    }
    return shouldRequeue;
}

void NotificationService::addPendingNotification(uint32_t uuid, uint8_t categoryId, uint8_t eventFlags)
{
    if (mEventQueue == nullptr) { return; }
    ancs_event_t event;
    event.type   = ancs_event_t::EVT_PENDING_UUID;
    event.length = 6;
    event.data[0] = (uint8_t)(uuid);
    event.data[1] = (uint8_t)(uuid >> 8);
    event.data[2] = (uint8_t)(uuid >> 16);
    event.data[3] = (uint8_t)(uuid >> 24);
    event.data[4] = categoryId;   // ANCS CategoryID — preserved for handleDataSourceEvent
    event.data[5] = eventFlags;   // ANCS EventFlags — e.g. EventFlagPreExisting
    xQueueSend(mEventQueue, &event, 0);
}

void NotificationService::clearPendingNotifications()
{
    if (mEventQueue != nullptr)
    {
        xQueueReset(mEventQueue);
    }
    // Also clear the category map and cancelled-UUID set — stale entries from the
    // old connection must not pollute a fresh reconnect.  The cancelled set in
    // particular could suppress real notifications if iOS reuses a UUID.
    ScopedLock lock(mMutex);
    pendingCategoryCount = 0;
    cancelledCount = 0;
}

void NotificationService::storePendingCategory(uint32_t uuid, uint8_t categoryId, uint8_t eventFlags)
{
    ScopedLock lock(mMutex);
    // Update existing entry if present (e.g. Modified re-queues the same UUID).
    for (size_t i = 0; i < pendingCategoryCount; i++) {
        if (pendingCategoryMap[i].uuid == uuid) {
            pendingCategoryMap[i].categoryId = categoryId;
            pendingCategoryMap[i].eventFlags = eventFlags;
            return;
        }
    }
    if (pendingCategoryCount < pendingCategoryMapSize) {
        pendingCategoryMap[pendingCategoryCount++] = { uuid, categoryId, eventFlags };
    } else {
        // Map full: evict the oldest entry (index 0) and append the new one.
        memmove(&pendingCategoryMap[0], &pendingCategoryMap[1],
                (pendingCategoryMapSize - 1) * sizeof(PendingCategoryEntry));
        pendingCategoryMap[pendingCategoryMapSize - 1] = { uuid, categoryId, eventFlags };
        ESP_LOGW(TAG, "pendingCategoryMap full — oldest entry evicted");
    }
}

void NotificationService::consumePendingCategory(uint32_t uuid, uint8_t& outCategoryId, uint8_t& outEventFlags)
{
    ScopedLock lock(mMutex);
    for (size_t i = 0; i < pendingCategoryCount; i++) {
        if (pendingCategoryMap[i].uuid == uuid) {
            outCategoryId  = pendingCategoryMap[i].categoryId;
            outEventFlags  = pendingCategoryMap[i].eventFlags;
            memmove(&pendingCategoryMap[i], &pendingCategoryMap[i + 1],
                    (pendingCategoryCount - i - 1) * sizeof(PendingCategoryEntry));
            pendingCategoryCount--;
            return;
        }
    }
    outCategoryId = 0; // not found — treat as CategoryIDOther
    outEventFlags = 0;
}

void NotificationService::addNotification(notification_def const& notification, bool isCalling)
{
    ScopedLock lock(mMutex);
    if (isCalling)
    {
        callingNotification = notification;
        callingNotification.fetchStartTime = xTaskGetTickCount();
    }
    else
    {
        int targetIndex = -1;
        for (size_t i = 0; i < notificationListSize; i++)
        {
            if (notificationList[i].key == 0) { targetIndex = static_cast<int>(i); break; }
        }
        if (targetIndex != -1) {
            notificationCount++;
        } else {
            for (size_t i = 0; i < notificationListSize; i++)
            {
                if (notificationList[i].showed) { targetIndex = static_cast<int>(i); break; }
            }
            if (targetIndex == -1) { targetIndex = 0; }
        }
        notificationList[targetIndex] = notification;
        notificationList[targetIndex].fetchStartTime = xTaskGetTickCount();
    }
}

size_t NotificationService::getNotificationCount() const
{
    ScopedLock lock(mMutex);
    return notificationCount;
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
    ScopedLock lock(mMutex);
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
                    return true;
                }
                break;
            }
            currentIndex++;
        }
    }
    return false;
}

size_t NotificationService::takeAllPendingNotifications(notification_def* buf, size_t maxCount)
{
    ScopedLock lock(mMutex);
    size_t taken = 0;
    for (size_t i = 0; i < notificationListSize && taken < maxCount; i++)
    {
        if (notificationList[i].key != 0 &&
            !notificationList[i].showed &&
            notificationList[i].isComplete)
        {
            buf[taken++] = notificationList[i];
            notificationList[i].showed = true;
        }
    }
    return taken;
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
    ScopedLock lock(mMutex);
    callingNotification.key = 0;
}

bool NotificationService::isCallingNotification() const
{
    ScopedLock lock(mMutex);
    return callingNotification.key != 0;
}

bool NotificationService::takeCallingNotification(notification_def& out)
{
    ScopedLock lock(mMutex);
    if (callingNotification.key != 0 &&
        callingNotification.isComplete &&
        !callingNotification.showed)
    {
        out = callingNotification;
        callingNotification.showed = true;
        return true;
    }
    return false;
}

bool NotificationService::removeNotification(uint32_t uuid)
{
    ScopedLock lock(mMutex);
    int index = findNotificationIndex(uuid);
    if (index != -1)
    {
        notificationList[index].reset();
        notificationList[index].key = 0;
        notificationList[index].type = APP_UNKNOWN;
        // Guard against underflow: notificationCount is size_t (unsigned).
        // A duplicate remove (e.g. BLE disconnect race) must not wrap it to SIZE_MAX.
        if (notificationCount > 0) { notificationCount--; }
    }
    return index != -1;
}

void NotificationService::addCancelledUUID(uint32_t uuid)
{
    ScopedLock lock(mMutex);
    bool found = false;
    for (size_t i = 0; i < cancelledCount; i++) {
        if (cancelledUUIDs[i] == uuid) { found = true; break; }
    }
    if (!found && cancelledCount < cancelledSetSize) {
        cancelledUUIDs[cancelledCount++] = uuid;
    } else if (!found) {
        memmove(&cancelledUUIDs[0], &cancelledUUIDs[1], (cancelledSetSize - 1) * sizeof(uint32_t));
        cancelledUUIDs[cancelledSetSize - 1] = uuid;
        ESP_LOGW(TAG, "cancelledUUIDs set full — oldest entry evicted");
    }
}

bool NotificationService::consumeCancelledUUID(uint32_t uuid)
{
    ScopedLock lock(mMutex);
    for (size_t i = 0; i < cancelledCount; i++) {
        if (cancelledUUIDs[i] == uuid) {
            memmove(&cancelledUUIDs[i], &cancelledUUIDs[i + 1],
                    (cancelledCount - i - 1) * sizeof(uint32_t));
            cancelledCount--;
            return true;
        }
    }
    return false;
}

void NotificationService::markFetchStart(uint32_t uuid)
{
    ScopedLock lock(mMutex);
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
}

bool NotificationService::resetIfStale(uint32_t uuid, TickType_t timeoutTicks)
{
    ScopedLock lock(mMutex);
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
        ESP_LOGW(TAG, "Fetch for UUID %08lx timed out — resetting for re-fetch",
                 static_cast<unsigned long>(uuid));
        bool wasShowed = notification->showed;
        notification->reset();
        notification->showed = wasShowed; // preserve showed so re-fetch won't re-display
        return true;
    }
    return false;
}


NotificationDescription::NotificationDescription(const char* name, uint16_t stack_size)
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

    // Attribute value is the bytes from offset 8 to end.
    // Null-terminate into a fixed stack buffer; excess bytes are silently truncated.
    char message[65] = {};
    if (length > 8) {
        size_t msgLen = std::min((size_t)(length - 8), sizeof(message) - 1);
        memcpy(message, pData + 8, msgLen);
        message[msgLen] = '\0';
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
            // Retrieve the CategoryID and EventFlags stored when the NotificationSource
            // event was processed.  CategoryID distinguishes an active incoming call
            // (CategoryIDIncomingCall=1) from a missed call or informational entry.
            // EventFlagPreExisting marks notifications that already existed on iOS before
            // this BLE connection — they must be fetched to populate the list accurately,
            // but must NOT be displayed or buzzed (they are not new to the user).
            uint8_t pendingCategoryId = 0;
            uint8_t pendingEventFlags = 0;
            consumePendingCategory(messageId, pendingCategoryId, pendingEventFlags);
            notification.categoryId  = pendingCategoryId;
            notification.preExisting = (pendingEventFlags & ANCS::EventFlagPreExisting) != 0;
            strncpy(notification.bundleId, message, sizeof(notification.bundleId) - 1);
            addNotification(notification, notification.isCall());
            ESP_LOGI(TAG, "Message from %s added (category=%u, isCall=%d, preExisting=%d)",
                     message,
                     static_cast<unsigned>(notification.categoryId),
                     static_cast<int>(notification.isCall()),
                     static_cast<int>(notification.preExisting));
        }
        else
        {
            // App not whitelisted — consume the pending category entry so it doesn't
            // accumulate in the map for a notification that will never be created.
            uint8_t dummy1, dummy2;
            consumePendingCategory(messageId, dummy1, dummy2);
            ESP_LOGI(TAG, "Message from %s suppressed", message);
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

    // pData[1] = EventFlags, pData[2] = CategoryID, pData[3] = CategoryCount
    const uint8_t eventFlags = pData[1];
    const uint8_t categoryId = pData[2];

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
            addPendingNotification(messageId, categoryId, eventFlags);
        }
    }
    else if (pData[0] == ANCS::EventIDNotificationAdded)
    {
        // Guard against duplicate Added events for UUIDs already tracked (can
        // occur with some iOS versions or rapid reconnects).  Re-fetching a
        // notification that already exists wastes BLE bandwidth and could
        // transiently reset an otherwise-complete entry.
        if (!exists(messageId))
        {
            addPendingNotification(messageId, categoryId, eventFlags);
        }
        else
        {
            ESP_LOGD(TAG, "Duplicate Added for %08" PRIx32 " — ignored", messageId);
        }
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
            uint8_t categoryId = event.data[4]; // stored by addPendingNotification
            uint8_t eventFlags = (event.length >= 6) ? event.data[5] : 0;

            if (consumeCancelledUUID(uuid))
            {
                ESP_LOGI(TAG, "Skipping cancelled UUID %08" PRIx32, uuid);
                break;
            }
            if (Ble.isConnected() && uuid != 0)
            {
                // Record the CategoryID and EventFlags so handleDataSourceEvent() can
                // correctly classify incoming calls vs. missed calls, and detect
                // pre-existing notifications that should be silently populated.
                storePendingCategory(uuid, categoryId, eventFlags);

                // If a previous fetch for this UUID timed out, reset it and re-queue
                // a fresh attempt rather than leaving the slot stuck incomplete.
                if (resetIfStale(uuid, FETCH_TIMEOUT))
                {
                    addPendingNotification(uuid, categoryId, eventFlags);
                    break;
                }

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