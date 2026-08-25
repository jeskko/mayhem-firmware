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
 * DMR (ETSI TS 102 361-1) BPTC(128,77): the FEC protecting the
 * Embedded Signalling LC payload -- the redundant per-superframe
 * Group/Private Voice LC, Talker Alias, and GPS Info messages carried
 * in voice frames B-F's "sync" bit position (see dmr_bptc19696.hpp's
 * header comment: this is that "fragmented voice-embedded-signalling
 * variant" it flags as a separate, not-yet-implemented path -- now
 * implemented here). A DIFFERENT BPTC variant from BPTC(196,96)
 * (CSBK/Data Header/Full LC): smaller matrix, single-pass row-only
 * correction, and a simple mod-31 checksum instead of a CRC.
 *
 * Structure, confirmed against dsd-fme's src/bptc.c
 * (BPTC_128x77_Extract_Data) and src/dmr_dburst.c (its caller):
 *
 *   - Input is 128 bits: the 4 voice frames B/C/D/E each contribute
 *     one 32-bit fragment (the middle 32 of that frame's 48-bit
 *     embedded-signalling field -- the outer 8+8 bits are separate EMB
 *     framing, not part of this payload), concatenated B,C,D,E in
 *     order into bits128[0..127].
 *   - Reshaped column-major into an 8-row x 16-col matrix:
 *     m[row][col] = bits128[col*8 + row].
 *   - Rows 0-6 (7 rows): each a systematic Hamming(16,11,4) codeword
 *     (11 data bits in cols 0-10, 5 parity bits in cols 11-15). Row 7
 *     is NOT a data row -- it's a simple per-column even-parity check
 *     across rows 0-6, used only as an extra (non-correcting) validity
 *     signal, not decoded here (the payload's own mod-31 checksum,
 *     checked by the caller, is the definitive accept/reject gate).
 *   - This implementation deliberately reuses the EXACT SAME
 *     HAMMING_15_11_3_PARITY table as dmr_bptc19696.cpp for the first
 *     4 of each row's 5 parity bits (confirmed byte-identical against
 *     dsd-fme's own Hamming_16_11_4_m_G generator matrix -- the 5th
 *     "extension" parity bit that turns 15,11,3 into 16,11,4 is
 *     exactly the standard SECDED construction, i.e. redundant with
 *     what's already validated elsewhere in this codebase). The
 *     extension bit and row-7 parity are read but not used for
 *     correction: full SECDED decoding would add double-error
 *     DETECTION on top of the single-error correction this already
 *     gets from reusing 15,11,3, but the mod-31 checksum already
 *     provides a definitive (if less surgical) way to reject a bad
 *     block, so skipping it is a deliberate scope-limiting choice, not
 *     an oversight.
 *   - Payload extraction (77 bits: 72 data + 5 checksum): rows 0-1
 *     cols 0-10 (11 bits each, 22 bits) + rows 2-6 cols 0-9 (10 bits
 *     each, 50 bits) = 72 data bits, then rows 2-6 col 10 (1 bit each,
 *     5 bits) = the checksum.
 */

#ifndef __DMR_BPTC12877_H__
#define __DMR_BPTC12877_H__

#include <cstdint>

namespace dmr {

// bits128: one bit per byte (0 or 1), the 4 concatenated 32-bit
// embedded-signalling fragments (frames B,C,D,E in order).
// out72: one bit per byte (0 or 1), the 72-bit Embedded LC payload
// (PF+Reserved+FLCO(8) + FID(8) + Data(56), same header shape as Full
// LC minus the RS(12,9) field it doesn't have).
// checksum5: the 5 raw checksum bits extracted alongside (caller
// checks these against a mod-31 sum of out72 -- see
// dmr_embedded_lc.hpp -- since that check, not this function, is what
// actually gates trust in the result).
// Returns false if any row's Hamming(16,11,4) syndrome didn't match a
// single-bit error pattern (i.e. that row is uncorrectable) -- callers
// should treat out72/checksum5 as unreliable in that case.
bool bptc12877_decode(const uint8_t bits128[128], uint8_t out72[72], uint8_t checksum5[5]);

}  // namespace dmr

#endif /*__DMR_BPTC12877_H__*/
