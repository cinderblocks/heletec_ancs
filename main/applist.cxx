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

#include "applist.h"
#include <cstring>

// Define static constexpr arrays
constexpr AppMapping ApplicationList::allowedApplications[];
constexpr AppDisplayName ApplicationList::applicationNames[];

bool ApplicationList::isAllowedApplication(const char* appName) const
{
    for (size_t i = 0; i < allowedAppCount; i++)
    {
        if (strcmp(appName, allowedApplications[i].bundleId) == 0)
            return true;
    }
    return false;
}

application_def ApplicationList::getApplicationId(const char* appName) const
{
    for (size_t i = 0; i < allowedAppCount; i++)
    {
        if (strcmp(appName, allowedApplications[i].bundleId) == 0)
            return allowedApplications[i].appId;
    }
    return APP_UNKNOWN;
}

const char* ApplicationList::getDisplayName(application_def appId) const
{
    for (size_t i = 0; i < appNameCount; i++)
    {
        if (applicationNames[i].appId == appId)
            return applicationNames[i].displayName;
    }
    return "";
}

/* extern */
ApplicationList AppList;