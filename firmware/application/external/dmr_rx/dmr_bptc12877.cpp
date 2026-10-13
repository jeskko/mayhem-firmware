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

// Syndrome contribution of each of a row's 15 Hamming columns: the data
// table for cols 0-10, then one bit per parity col 11-14.
uint8_t column_syndrome(int col) {
    return col < 11 ? HAMMING_15_11_3_PARITY[col] : static_cast<uint8_t>(1u << (14 - col));
}

uint8_t row_syndrome(const uint8_t* row) {
    uint8_t s = 0;
    for (int c = 0; c < 15; c++)
        if (row[c]) s ^= column_syndrome(c);
    return s;
}

uint8_t row_parity(const uint8_t* row) {
    uint8_t p = 0;
    for (int c = 0; c < 16; c++) p ^= row[c];
    return p;
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

    // Rows 0-6: SECDED. Correct single errors; remember a double-error row.
    int double_row = -1;
    int double_rows = 0;
    for (int r = 0; r < 7; r++) {
        const uint8_t syndrome = row_syndrome(m[r]);
        if (syndrome == 0) continue;  // clean, or only the parity bit (col 15) is wrong
        if (row_parity(m[r])) {
            for (int c = 0; c < 15; c++) {
                if (column_syndrome(c) == syndrome) {
                    m[r][c] ^= 1;
                    break;
                }
            }
        } else {
            double_row = r;
            double_rows++;
        }
    }

    bool ok = double_rows == 0;
    if (double_rows == 1) {
        // Column parity (row 7) over the otherwise-corrected matrix points
        // at the double-error row's two wrong columns.
        uint8_t wrong[15];
        int n_wrong = 0;
        for (int c = 0; c < 15; c++) {
            uint8_t p = 0;
            for (int r = 0; r < 8; r++) p ^= m[r][c];
            if (p) wrong[n_wrong++] = static_cast<uint8_t>(c);
        }
        const uint8_t syndrome = row_syndrome(m[double_row]);
        int fixes = 0, fix_a = 0, fix_b = 0;
        for (int i = 0; i < n_wrong; i++)
            for (int j = i + 1; j < n_wrong; j++)
                if ((column_syndrome(wrong[i]) ^ column_syndrome(wrong[j])) == syndrome) {
                    fixes++;
                    fix_a = wrong[i];
                    fix_b = wrong[j];
                }
        if (fixes == 1) {
            m[double_row][fix_a] ^= 1;
            m[double_row][fix_b] ^= 1;
            ok = true;
        }
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
