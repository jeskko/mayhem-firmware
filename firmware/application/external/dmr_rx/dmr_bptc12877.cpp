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

#include "dmr_bptc12877.hpp"

namespace dmr {
namespace {

// Same table as dmr_bptc19696.cpp's HAMMING_15_11_3_PARITY -- confirmed
// byte-identical against dsd-fme's Hamming_16_11_4_m_G generator matrix
// (each data bit's expected 4-bit parity contribution, MSB-first; see
// this file's header comment for why duplicating rather than sharing a
// header is the deliberate choice here).
constexpr uint8_t HAMMING_15_11_3_PARITY[11] = {
    0b1001, 0b1101, 0b1111, 0b1110, 0b0111, 0b1010, 0b0101, 0b1011, 0b1100, 0b0110, 0b0011};

// Identical to dmr_bptc19696.cpp's hamming_fix() -- single-bit-error
// correction for a systematic Hamming code, duplicated rather than
// shared for the same self-containment reason as the table above.
bool hamming_fix(uint8_t* data, uint8_t* parity) {
    uint8_t calc = 0;
    for (int i = 0; i < 11; i++)
        if (data[i]) calc ^= HAMMING_15_11_3_PARITY[i];

    uint8_t received = 0;
    for (int b = 0; b < 4; b++) received |= parity[b] << (3 - b);

    const uint8_t syndrome = calc ^ received;
    if (syndrome == 0) return true;

    for (int i = 0; i < 11; i++) {
        if (HAMMING_15_11_3_PARITY[i] == syndrome) {
            data[i] ^= 1;
            return true;
        }
    }
    for (int b = 0; b < 4; b++) {
        if (syndrome == static_cast<uint8_t>(1u << (3 - b))) {
            parity[b] ^= 1;
            return true;
        }
    }
    return false;
}

}  // namespace

bool bptc12877_decode(const uint8_t bits128[128], uint8_t out72[72], uint8_t checksum5[5]) {
    // m[row][col] = bits128[col*8 + row] -- column-major reshape, see
    // header comment (this IS the deinterleaving; unlike BPTC(196,96)
    // there's no separate closed-form permutation step).
    uint8_t m[8][16];
    for (int col = 0; col < 16; col++)
        for (int row = 0; row < 8; row++)
            m[row][col] = bits128[col * 8 + row];

    bool ok = true;
    for (int r = 0; r < 7; r++) {
        uint8_t data[11];
        for (int c = 0; c < 11; c++) data[c] = m[r][c];
        uint8_t parity[4] = {m[r][11], m[r][12], m[r][13], m[r][14]};
        // m[r][15] (the SECDED extension bit) and row 7 (column parity)
        // are deliberately not used for correction -- see header comment.
        const bool row_ok = hamming_fix(data, parity);
        ok = ok && row_ok;
        for (int c = 0; c < 11; c++) m[r][c] = data[c];
    }

    int idx = 0;
    for (int r = 0; r < 2; r++)
        for (int c = 0; c < 11; c++) out72[idx++] = m[r][c];
    for (int r = 2; r < 7; r++)
        for (int c = 0; c < 10; c++) out72[idx++] = m[r][c];

    for (int r = 2; r < 7; r++) checksum5[r - 2] = m[r][10];

    return ok;
}

}  // namespace dmr
