/*
 * Copyright (C) 2026 Jarkko Vääräniemi
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

/*
 * A small, fixed-capacity ring buffer that any app can push short log
 * lines into, queryable over the USB-serial shell via the `applog`
 * command (see usb_serial_shell.cpp's cmd_applog()) without needing a
 * screenshot/screenframe capture at all -- much faster for live
 * debugging over serial, and it survives past whatever currently fits
 * on screen (a Console widget's own on-screen scrollback is bounded by
 * pixels, this by line count).
 *
 * Deliberately global/single-instance rather than per-app: only one app
 * runs at a time on this firmware, so there's no ambiguity about which
 * app's log a query is reading, and it avoids needing any per-app
 * registration/lookup mechanism. Switching to a different app just
 * means old lines age out of the ring as the new app's own lines (if
 * it pushes any) start arriving.
 */

#ifndef __APP_DEBUG_LOG_H__
#define __APP_DEBUG_LOG_H__

#include <string>
#include <vector>

namespace app_debug_log {

constexpr size_t CAPACITY = 128;

void push(const std::string& line);

// Oldest-first snapshot of whatever's currently in the ring (fewer than
// CAPACITY entries until it's wrapped at least once). Empty if nothing
// has been pushed yet.
std::vector<std::string> snapshot();

void clear();

}  // namespace app_debug_log

#endif /*__APP_DEBUG_LOG_H__*/
