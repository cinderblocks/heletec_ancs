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
#include <time.h>

class BLERemoteCharacteristic;

struct notification_def
{
    String title = "";
    String message = "";
    time_t time = 0;
    application_def type = APP_UNKNOWN;
    uint32_t key = 0;
    boolean showed = false;
    boolean isComplete = false;

    bool isCall() const { return type == APP_PHONE || type == APP_FACETIME; }
};

class NotificationService
{
public:
    NotificationService();
    ~NotificationService();
    
    void addPendingNotification(uint32_t uuid);
    uint32_t getNextPendingNotification();
    uint32_t waitForNextPendingNotification();
    void clearPendingNotifications();
    void addNotification(notification_def const& notification, bool isCalling);
    void removeNotification(uint32_t uuid);
    void removeCallNotification();
    bool exists(uint32_t uuid) const;
    [[nodiscard]] bool isCallingNotification() const;
    notification_def& getCallingNotification();
    notification_def* getNotification(uint32_t uuid);
    size_t getNotificationCount() const;
    notification_def* getNotificationByIndex(size_t index);
    bool takeNotificationByIndex(size_t index, notification_def& out);

    static void DataSourceNotifyCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify);
    static void NotificationSourceNotifyCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length,bool isNotify);

private:
    static constexpr size_t notificationListSize = 16;
    static constexpr size_t pendingQueueSize = 64;
    
    notification_def notificationList[notificationListSize];
    size_t notificationCount;
    QueueHandle_t pendingNotificationQueue;
    notification_def callingNotification;
    mutable SemaphoreHandle_t mMutex;
    
    int findNotificationIndex(uint32_t uuid) const;
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
