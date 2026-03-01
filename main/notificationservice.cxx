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
                ::xTaskNotifyGive(Heltec.mDrawTask);
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
                ::xTaskNotifyGive(Heltec.mDrawTask);
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

void NotificationService::addNotification(notification_def const& notification, bool isCalling)
{
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
        
        // If no empty slot, use first slot (oldest)
        if (targetIndex == -1)
        {
            targetIndex = 0;
        }
        
        notificationList[targetIndex] = notification;
        notificationCount = 0;
        for (size_t i = 0; i < notificationListSize; i++)
        {
            if (notificationList[i].key != 0)
            {
                notificationCount++;
            }
        }
    }
}

size_t NotificationService::getNotificationCount() const
{
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
    callingNotification.key = 0;
}

bool NotificationService::isCallingNotification() const
{
    return callingNotification.key != 0;
}

notification_def& NotificationService::getCallingNotification()
{
    return callingNotification;
}

void NotificationService::removeNotification(uint32_t uuid)
{
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
}


NotificationDescription::NotificationDescription(String const& name, uint16_t stack_size)
:   Task(name, stack_size)
{ }

void NotificationDescription::run(void *data)
{
    while (true)
    {
        // Only attempt ANCS writes when we are actually connected and the
        // control-point characteristic is live.  Ble.isConnected() returns false
        // as soon as ServerCallback::onDisconnect fires, which is always BEFORE
        // EndTask calls delete pClient.  This prevents the use-after-free of
        // _controlPointCharacteristic that was trashing the BTC task state.
        if (Ble.isConnected())
        {
            uint32_t pendingNotificationId = Notifications.getNextPendingNotification();
            if (pendingNotificationId != 0)
            {
                Ble.retrieveNotificationData(pendingNotificationId);
            }
        }
        delay(500);
    }
}

/* extern */
NotificationService Notifications;
NotificationDescription NotificationReceiver("NotificationReceiver", 50000);