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

#include "dmr_bptc19696.hpp"

#include <cstring>

namespace dmr {
namespace {

// ETSI TS 102 361-1 Annex B.3.4, Tables B.14/B.15: each data bit's 4-bit
// parity contribution (MSB-first), for the two Hamming codes BPTC(196,96)
// uses. Independently confirmed byte-identical against MMDVMHost, go-dmr,
// dmrlib, and ok-dmrlib's own generator matrices/XOR-syndrome equations.
constexpr uint8_t HAMMING_13_9_3_PARITY[9] = {
    0b1111, 0b1110, 0b0111, 0b1010, 0b0101, 0b1011, 0b1100, 0b0110, 0b0011};
constexpr uint8_t HAMMING_15_11_3_PARITY[11] = {
    0b1001, 0b1101, 0b1111, 0b1110, 0b0111, 0b1010, 0b0101, 0b1011, 0b1100, 0b0110, 0b0011};

// Single-bit-error correction for a systematic Hamming code: `data`
// holds `n` data bits, `parity` holds the 4 received parity bits,
// `table` gives each data bit's expected 4-bit parity contribution
// (MSB-first). Returns false only if the syndrome doesn't match any
// single data or parity bit (i.e. 2+ bit errors -- not correctable by
// this code); true means the block was already correct or a single-bit
// error was fixed in place.
template <size_t N>
bool hamming_fix(uint8_t* data, uint8_t* parity, const uint8_t (&table)[N]) {
    uint8_t calc = 0;
    for (size_t i = 0; i < N; i++)
        if (data[i]) calc ^= table[i];

    uint8_t received = 0;
    for (int b = 0; b < 4; b++) received |= parity[b] << (3 - b);

    const uint8_t syndrome = calc ^ received;
    if (syndrome == 0) return true;

    for (size_t i = 0; i < N; i++) {
        if (table[i] == syndrome) {
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

bool bptc19696_decode(const uint8_t bits196[196], uint8_t out96[96]) {
    uint8_t raw[196];
    for (int a = 0; a < 196; a++) raw[a] = bits196[(a * 181) % 196];

    // m[row][col], row 0..12, col 0..14 -- raw[0] (reserved R3) is
    // discarded by starting the matrix fill at raw[1].
    uint8_t m[13][15];
    for (int r = 0; r < 13; r++)
        for (int c = 0; c < 15; c++)
            m[r][c] = raw[1 + r * 15 + c];

    bool ok = true;
    for (int pass = 0; pass < 5; pass++) {
        // Row pass: Hamming(15,11,3) across each data row (cols 0-10
        // data, cols 11-14 parity), rows 0-8 only.
        for (int r = 0; r < 9; r++) {
            uint8_t data[11];
            std::memcpy(data, m[r], 11);
            uint8_t parity[4] = {m[r][11], m[r][12], m[r][13], m[r][14]};
            const bool row_ok = hamming_fix(data, parity, HAMMING_15_11_3_PARITY);
            ok = ok && row_ok;
            std::memcpy(m[r], data, 11);
            m[r][11] = parity[0];
            m[r][12] = parity[1];
            m[r][13] = parity[2];
            m[r][14] = parity[3];
        }

        // Column pass: Hamming(13,9,3) down each of the 15 columns
        // (rows 0-8 data, rows 9-12 parity).
        for (int c = 0; c < 15; c++) {
            uint8_t data[9];
            for (int r = 0; r < 9; r++) data[r] = m[r][c];
            uint8_t parity[4] = {m[9][c], m[10][c], m[11][c], m[12][c]};
            const bool col_ok = hamming_fix(data, parity, HAMMING_13_9_3_PARITY);
            ok = ok && col_ok;
            for (int r = 0; r < 9; r++) m[r][c] = data[r];
            m[9][c] = parity[0];
            m[10][c] = parity[1];
            m[11][c] = parity[2];
            m[12][c] = parity[3];
        }
    }

    int idx = 0;
    for (int c = 3; c <= 10; c++) out96[idx++] = m[0][c];
    for (int r = 1; r <= 8; r++)
        for (int c = 0; c <= 10; c++) out96[idx++] = m[r][c];

    return ok;
}

}  // namespace dmr
