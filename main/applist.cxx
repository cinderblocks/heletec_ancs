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

#include "applist.h"

std::map<String, application_def>::const_iterator allowedApplicationIterator;
std::map<application_def, String>::const_iterator applicationNameIterator;

bool ApplicationList::isAllowedApplication(String const& appName) const
{
    return allowedApplication.contains(appName);
}

application_def ApplicationList::getApplicationId(String const& appName) const
{
    allowedApplicationIterator = allowedApplication.find(appName);
    if (allowedApplicationIterator != allowedApplication.cend())
    {
        return allowedApplicationIterator->second;
    }
    return application_def();
}

String ApplicationList::getDisplayName(application_def appId) const
{
    applicationNameIterator = applicationName.find(appId);
    if (applicationNameIterator != applicationName.cend())
    {
        return applicationNameIterator->second;
    }
    return {};
}

/* extern */
ApplicationList AppList;
