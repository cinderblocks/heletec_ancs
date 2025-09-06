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

#include <Arduino.h>
#include <map>
#include <stack>
#include <ctime>

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
    void addPendingNotification(uint32_t uuid);
    uint32_t getNextPendingNotification();
    void addNotification(notification_def const& notification, bool isCalling);
    void removeNotification(uint32_t uuid);
    void removeCallNotification();
    bool exists(uint32_t uuid) const;
    [[nodiscard]] bool isCallingNotification() const;
    notification_def& getCallingNotification();
    notification_def* getNotification(uint32_t uuid);
    std::map<uint32_t, notification_def>& getNotificationList();

    static void DataSourceNotifyCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify);
    static void NotificationSourceNotifyCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length,bool isNotify);

private:
    static constexpr int notificationListSize = 8;
    std::map<uint32_t, notification_def> notificationList;
    std::stack<uint32_t> pendingNotification;
    notification_def callingNotification;
};

extern NotificationService Notifications;

#endif /* NOTIFICATION_SERVICE_H_ */
