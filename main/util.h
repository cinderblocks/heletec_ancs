/**
* Copyright (c) 2025 Sjofn LLC
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

#ifndef HELTEC_ANCS_UTIL_H
#define HELTEC_ANCS_UTIL_H

inline uint8_t convert_to_tz(uint8_t utc, int8_t tz_offset)
{
    int8_t local = (utc + tz_offset) % 24;
    return local < 0 ? local + 24 : local;
}

#endif //HELTEC_ANCS_UTIL_H
