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

#include "dmr_golay2087.hpp"

namespace dmr {

namespace {

// data bit i, i=0 is the MSB (CC3), i=7 is the LSB (DT0).
inline int bit_of(uint8_t data, int i) {
    return (data >> (7 - i)) & 1;
}

// 12 systematic parity bits for an 8-bit Slot Type message, per ETSI TS
// 102 361-1 Annex B.3.1 (see dmr_golay2087.hpp for cross-check notes).
// Returned MSB-first as p0..p11 in bits 11..0.
uint16_t golay2087_parity(uint8_t data) {
    auto b = [&](int i) { return bit_of(data, i); };
    uint16_t p = 0;
    p |= (b(1) ^ b(4) ^ b(5) ^ b(6) ^ b(7)) << 11;
    p |= (b(1) ^ b(2) ^ b(4)) << 10;
    p |= (b(0) ^ b(2) ^ b(3) ^ b(5)) << 9;
    p |= (b(0) ^ b(1) ^ b(3) ^ b(4) ^ b(6)) << 8;
    p |= (b(0) ^ b(1) ^ b(2) ^ b(4) ^ b(5) ^ b(7)) << 7;
    p |= (b(0) ^ b(2) ^ b(3) ^ b(4) ^ b(7)) << 6;
    p |= (b(3) ^ b(6) ^ b(7)) << 5;
    p |= (b(0) ^ b(1) ^ b(5) ^ b(6)) << 4;
    p |= (b(0) ^ b(1) ^ b(2) ^ b(6) ^ b(7)) << 3;
    p |= (b(2) ^ b(3) ^ b(4) ^ b(5) ^ b(6)) << 2;
    p |= (b(0) ^ b(3) ^ b(4) ^ b(5) ^ b(6) ^ b(7)) << 1;
    p |= (b(1) ^ b(2) ^ b(3) ^ b(5) ^ b(7));
    return p & 0x0FFF;
}

}  // namespace

uint32_t golay2087_encode(uint8_t data8) {
    return (static_cast<uint32_t>(data8) << 12) | golay2087_parity(data8);
}

SlotTypeResult golay2087_decode(uint32_t word20) {
    SlotTypeResult result{};
    const uint32_t received = word20 & 0xFFFFFu;  // 20 bits

    int best_dist = 21;
    uint8_t best_val = 0;

    for (int v = 0; v < 256; v++) {
        const uint32_t codeword = golay2087_encode(static_cast<uint8_t>(v));
        const int dist = __builtin_popcount(codeword ^ received);
        if (dist < best_dist) {
            best_dist = dist;
            best_val = static_cast<uint8_t>(v);
            if (dist == 0) break;
        }
    }

    result.color_code = (best_val >> 4) & 0x0F;
    result.data_type = best_val & 0x0F;
    result.corrected_errors = static_cast<uint8_t>(best_dist);
    // d_min=7 guarantees exact recovery for <=3 bit errors anywhere in
    // the 20-bit codeword; beyond that the nearest codeword may not be
    // the transmitted one (empirically ~1/3 right at 4 errors, ~1/16 at
    // 5, per the cross-verification's brute-force trials), so treat
    // anything past 3 as unreliable.
    result.ok = best_dist <= 3;

    return result;
}

}  // namespace dmr
