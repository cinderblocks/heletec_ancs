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

#ifndef APP_LIST_H_
#define APP_LIST_H_

#include <Arduino.h>
#include <map>

typedef enum
{
    APP_UNKNOWN = 0,
    APP_SMS,
    APP_PHONE,
    APP_FACETIME,
    APP_MESSENGER,
    APP_PINGER,
    APP_TEXTNOW,
    APP_KEYBASE,
    APP_SIGNAL,
    APP_HONK,
    APP_GHRN,
    APP_TOWBOOK,
} application_def;

class ApplicationList
{
public:
    bool isAllowedApplication(String const &appName) const;
    application_def getApplicationId(String const& appName) const;
    String getDisplayName(application_def appId) const;
private:
    const std::map<String, application_def> allowedApplication {
        {"com.apple.MobileSMS", APP_SMS},
        {"com.apple.mobilephone", APP_PHONE},
        {"com.apple.facetime", APP_FACETIME},
        {"com.facebook.Messenger", APP_MESSENGER},
        {"com.pinger.textfreeWithVoice", APP_PINGER},
        {"com.tinginteractive.usms", APP_TEXTNOW},
        {"keybase.ios", APP_KEYBASE},
        {"org.whispersystems.signal", APP_SIGNAL},
        {"com.honkforhelp.driver", APP_HONK},
        {"com.arity.rescuer", APP_GHRN},
        {"com.towbook.mobile", APP_TOWBOOK},
    };

    const std::map<application_def, String> applicationName {
        {APP_SMS, "iMessage"},
        {APP_PHONE, "Call"},
        {APP_FACETIME, "Facetime"},
        {APP_MESSENGER, "Facebook"},
        {APP_PINGER, "Pinger"},
        {APP_TEXTNOW, "TextNow"},
        {APP_KEYBASE, "Keybase"},
        {APP_SIGNAL, "Signal"},
        {APP_HONK, "Honk"},
        {APP_GHRN, "GHRN"},
        {APP_TOWBOOK, "Towbook"},
    };
};

extern ApplicationList AppList;

#endif /* APP_LIST_H_ */
