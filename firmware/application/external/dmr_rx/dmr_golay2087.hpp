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
 * DMR (ETSI TS 102 361-1 Annex B.3.1) Slot Type FEC: an 8-bit message
 * (4-bit Color Code + 4-bit Data Type, MSB-first: CC3 CC2 CC1 CC0 DT3
 * DT2 DT1 DT0) protected by 12 parity bits (systematic, data-then-
 * parity, not interleaved) inside a 20-bit Golay(20,8,7) codeword.
 *
 * The parity equations here were cross-checked against four
 * independent open-source DMR implementations (MMDVMHost's
 * Golay2087.cpp, the classic dmrlib's golay_20_8.c, pd0mz/go-dmr's
 * golay_20_8.go, and OK-DMR/ok-dmrlib's golay_20_8_7.py, the last of
 * which cites the ETSI generator matrix directly) -- two independent
 * derivations (go-dmr's hand-written equations and ok-dmrlib's spec-
 * cited generator matrix) were multiplied out and agree term-for-term,
 * and the resulting 256-entry encode table was diffed against the
 * other two projects' shipped lookup tables with zero mismatches.
 * Decode is nearest-codeword search over all 256 valid codewords
 * (cheap: 256 popcount comparisons, done once per received burst on
 * the application core, not the baseband core) rather than a
 * syndrome/table decoder, since d_min=7 makes "closest of 256" both
 * simpler to get right and just as correct: guaranteed exact recovery
 * for <=3 bit errors anywhere in the 20 bits, falling off predictably
 * beyond that.
 */

#ifndef __DMR_GOLAY2087_H__
#define __DMR_GOLAY2087_H__

#include <cstdint>

namespace dmr {

struct SlotTypeResult {
    bool ok{false};  // true iff corrected_errors <= 3 (guaranteed-correct region for d_min=7)
    uint8_t color_code{0};
    uint8_t data_type{0};
    uint8_t corrected_errors{0};
};

// word20: 20-bit received codeword, MSB-first as transmitted
// (CC3..CC0 DT3..DT0 p0..p11), as reassembled from the two 10-bit Slot
// Type halves either side of the burst's Sync field.
SlotTypeResult golay2087_decode(uint32_t word20);

// data8: color_code<<4 | data_type. Returns the encoded 20-bit codeword.
uint32_t golay2087_encode(uint8_t data8);

}  // namespace dmr

#endif /*__DMR_GOLAY2087_H__*/
