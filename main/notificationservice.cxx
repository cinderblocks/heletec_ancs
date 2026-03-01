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

#include "notificationservice.h"

#include "ancs.h"
#include "bleservice.h"
#include "hardware.h"
#include <BLERemoteCharacteristic.h>

static const char* TAG = "notify";

NotificationService::NotificationService()
:   notificationCount(0)
,   pendingNotificationQueue(nullptr)
,   mMutex(xSemaphoreCreateMutex())
{
    // Initialize notification list
    for (size_t i = 0; i < notificationListSize; i++)
    {
        notificationList[i].key = 0;
    }
    
    // Create FreeRTOS queue for pending notifications
    pendingNotificationQueue = xQueueCreate(pendingQueueSize, sizeof(uint32_t));
}

NotificationService::~NotificationService()
{
    if (pendingNotificationQueue != nullptr)
    {
        vQueueDelete(pendingNotificationQueue);
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
void NotificationService::DataSourceNotifyCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    String message;

    uint32_t messageId = pData[4];
    messageId = messageId << 8 | pData[3];
    messageId = messageId << 16 | pData[2];
    messageId = messageId << 24 | pData[1];

    for (int i = 8; i < length; i++)
    {
        message += static_cast<char>(pData[i]);
    }
    if (Notifications.exists(messageId))
    {
        notification_def *notification = Notifications.getNotification(messageId);
        if (notification != nullptr) {
            switch (pData[5])
            {
                case ANCS::NotificationAttributeIDTitle:
                {
                    notification->title = message;
                    Serial.println(message.c_str());
                    break;
                }
                case ANCS::NotificationAttributeIDMessage:
                {
                    notification->message = message;
                    Serial.println(message.c_str());
                    break;
                }
                case ANCS::NotificationAttributeIDDate:
                {
                    tm t = {};
                    strptime(message.c_str(), "%Y%m%dT%H%M%S", &t);
                    notification->time = mktime(&t);
                    break;
                }
                default:
                    break;
            }
            if (!notification->title.isEmpty() && !notification->message.isEmpty() && notification->time != 0) {
                notification->isComplete = true;
                Heltec.notifyDraw(Hardware::DRAW_NOTIFY);
            }
        }
    }
    else if (pData[5] == ANCS::NotificationAttributeIDAppIdentifier)
    {
        if (AppList.isAllowedApplication(message))
        {
            const application_def applicationType = AppList.getApplicationId(message);
            notification_def notification;
            notification.type = applicationType;
            notification.key = messageId;
            Notifications.addNotification(notification, notification.isCall());
            ESP_LOGI(TAG, "Message from %s added", message.c_str());
        } else {
            ESP_LOGI(TAG, "Message from %s suppressed", message.c_str());
        }
    }
}

/* static */
void NotificationService::NotificationSourceNotifyCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length,bool isNotify)
{
    uint32_t messageId = pData[7];
    messageId = messageId << 8 | pData[6];
    messageId = messageId << 16 | pData[5];
    messageId = messageId << 24 | pData[4];
    if (pData[0] == ANCS::EventIDNotificationRemoved)
    {
        notification_def *notification = Notifications.getNotification(messageId);
        if (notification != nullptr)
        {
            if (notification->isCall())
            {
                Notifications.removeCallNotification();
                Heltec.notifyDraw(Hardware::DRAW_NOTIFY);
            }
        }
    }
    else if (pData[0] == ANCS::EventIDNotificationAdded)
    {
        Notifications.addPendingNotification(messageId);
    }
}

void NotificationService::addPendingNotification(uint32_t uuid)
{
    if (pendingNotificationQueue != nullptr)
    {
        xQueueSend(pendingNotificationQueue, &uuid, 0);
    }
}

void NotificationService::clearPendingNotifications()
{
    if (pendingNotificationQueue != nullptr)
    {
        xQueueReset(pendingNotificationQueue);
    }
}

uint32_t NotificationService::getNextPendingNotification()
{
    uint32_t uuid = 0;
    if (pendingNotificationQueue != nullptr)
    {
        xQueueReceive(pendingNotificationQueue, &uuid, 0);
    }
    return uuid;
}

uint32_t NotificationService::waitForNextPendingNotification()
{
    uint32_t uuid = 0;
    if (pendingNotificationQueue != nullptr)
    {
        xQueueReceive(pendingNotificationQueue, &uuid, portMAX_DELAY);
    }
    return uuid;
}

void NotificationService::addNotification(notification_def const& notification, bool isCalling)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    if (isCalling)
    {
        callingNotification = notification;
    }
    else
    {
        // Find empty slot or oldest notification
        int targetIndex = -1;
        
        // First try to find empty slot (key == 0)
        for (size_t i = 0; i < notificationListSize; i++)
        {
            if (notificationList[i].key == 0)
            {
                targetIndex = static_cast<int>(i);
                break;
            }
        }
        
        // If no empty slot, use first slot (oldest) — count stays the same
        if (targetIndex == -1)
        {
            targetIndex = 0;
        }
        else
        {
            notificationCount++;
        }

        notificationList[targetIndex] = notification;
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

notification_def& NotificationService::getCallingNotification()
{
    return callingNotification;
}

void NotificationService::removeNotification(uint32_t uuid)
{
    xSemaphoreTake(mMutex, portMAX_DELAY);
    int index = findNotificationIndex(uuid);
    if (index != -1)
    {
        notificationList[index].key = 0;
        notificationList[index].title = "";
        notificationList[index].message = "";
        notificationList[index].time = 0;
        notificationList[index].type = APP_UNKNOWN;
        notificationList[index].showed = false;
        notificationList[index].isComplete = false;
        notificationCount--;
    }
    xSemaphoreGive(mMutex);
}


NotificationDescription::NotificationDescription(String const& name, uint16_t stack_size)
:   Task(name, stack_size)
{ }

void NotificationDescription::run(void *data)
{
    while (true)
    {
        // Block until a UUID is available — no polling delay needed.
        // clearPendingNotifications() drains the queue on disconnect, so this
        // unblocks immediately if the queue is reset while we are waiting.
        uint32_t pendingNotificationId = Notifications.waitForNextPendingNotification();

        // Only attempt ANCS writes when we are actually connected and the
        // control-point characteristic is live.  Ble.isConnected() returns false
        // as soon as ServerCallback::onDisconnect fires, which is always BEFORE
        // EndTask calls delete pClient.  This prevents the use-after-free of
        // _controlPointCharacteristic that was trashing the BTC task state.
        if (Ble.isConnected() && pendingNotificationId != 0)
        {
            Ble.retrieveNotificationData(pendingNotificationId);
        }
    }
}

/* extern */
NotificationService Notifications;
NotificationDescription NotificationReceiver("NotificationReceiver", 50000);