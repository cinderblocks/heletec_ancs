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

#ifndef ANCS_H_
#define ANCS_H_

namespace ANCS
{
    enum category_id_t
    {
        CategoryIDOther = 0,
        CategoryIDIncomingCall = 1,
        CategoryIDMissedCall = 2,
        CategoryIDVoicemail = 3,
        CategoryIDSocial = 4,
        CategoryIDSchedule = 5,
        CategoryIDEmail = 6,
        CategoryIDNews = 7,
        CategoryIDHealthAndFitness = 8,
        CategoryIDBusinessAndFinance = 9,
        CategoryIDLocation = 10,
        CategoryIDEntertainment = 11
    };

    enum event_id_t
    {
        EventIDNotificationAdded = 0,
        EventIDNotificationModified = 1,
        EventIDNotificationRemoved = 2
    };

    enum event_flags_t
    {
        EventFlagSilent          = (1 << 0),
        EventFlagImportant       = (1 << 1),
        EventFlagPreExisting     = (1 << 2),
        EventFlagPositiveAction  = (1 << 3),
        EventFlagNegativeAction  = (1 << 4)
    };

    enum command_id_t
    {
        CommandIDGetNotificationAttributes  = 0,
        CommandIDGetAppAttributes           = 1,
        CommandIDPerformNotificationAction  = 2
    };

    enum notification_attribute_id_t
    {
        NotificationAttributeIDAppIdentifier        = 0,
        NotificationAttributeIDTitle                = 1,  // followed by 2-byte max-length
        NotificationAttributeIDSubtitle             = 2,  // followed by 2-byte max-length
        NotificationAttributeIDMessage              = 3,  // followed by 2-byte max-length
        NotificationAttributeIDMessageSize          = 4,
        NotificationAttributeIDDate                 = 5,
        NotificationAttributeIDPositiveActionLabel  = 6,
        NotificationAttributeIDNegativeActionLabel  = 7
    };
} // namespace ANCS

#endif /* ANCS_H_ */