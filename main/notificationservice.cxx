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
#include "heltec.h"
#include <BLERemoteCharacteristic.h>

static const char* TAG = "Notifications";

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
            if (!notification->title.isEmpty() && !notification->message.isEmpty()) {
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
    pendingNotification.push(uuid);
}

uint32_t NotificationService::getNextPendingNotification()
{
    if (!pendingNotification.empty())
    {
        const uint32_t uuid = pendingNotification.top();
        pendingNotification.pop();
        return uuid;
    }
    return 0;
}

void NotificationService::addNotification(notification_def const& notification, bool isCalling)
{
    if (isCalling)
    {
        callingNotification = notification;
    }
    else
    {
        if (notificationList.size() >= notificationListSize)
        {
            const auto it = notificationList.cbegin();
            notificationList.erase(it);
        }
        notificationList.insert(std::pair(notification.key, notification));
    }
}

std::map<uint32_t, notification_def>& NotificationService::getNotificationList()
{
    return notificationList;
}

bool NotificationService::exists(uint32_t uuid) const
{
    const auto it = notificationList.find(uuid);
    return (it != notificationList.end()) | (callingNotification.key != 0);
}

notification_def* NotificationService::getNotification(uint32_t uuid)
{
    if (callingNotification.key == uuid)
    {
        return &callingNotification;
    }
    if (const auto it = notificationList.find(uuid); it != notificationList.end()) {
        return &it->second;
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
    if (const auto it = notificationList.find(uuid); it != notificationList.end())
    {
        notificationList.erase(it);
    }
}

/* extern */
NotificationService Notifications;