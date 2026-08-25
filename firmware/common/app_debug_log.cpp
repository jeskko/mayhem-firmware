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

#include "app_debug_log.hpp"

#include <array>

namespace app_debug_log {
namespace {

std::array<std::string, CAPACITY> ring_{};
size_t next_write_ = 0;
size_t count_ = 0;

}  // namespace

void push(const std::string& line) {
    ring_[next_write_] = line;
    next_write_ = (next_write_ + 1) % CAPACITY;
    if (count_ < CAPACITY) count_++;
}

std::vector<std::string> snapshot() {
    std::vector<std::string> out;
    out.reserve(count_);
    const size_t start = (count_ < CAPACITY) ? 0 : next_write_;
    for (size_t i = 0; i < count_; i++) {
        out.push_back(ring_[(start + i) % CAPACITY]);
    }
    return out;
}

void clear() {
    count_ = 0;
    next_write_ = 0;
}

}  // namespace app_debug_log
