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

#ifndef SECURITY_CALLBACK_H_
#define SECURITY_CALLBACK_H_

// With esp-nimble-cpp 2.x, security callbacks (onAuthenticationComplete,
// onPassKeyDisplay, onConfirmPasskey) are methods of NimBLEServerCallbacks.
// They are now implemented directly in BleService::ServerCallback.
//
// This header is kept for backward compatibility — it was previously included
// by bleservice.cxx and other files. It is now essentially empty.

#endif /* SECURITY_CALLBACK_H_ */
