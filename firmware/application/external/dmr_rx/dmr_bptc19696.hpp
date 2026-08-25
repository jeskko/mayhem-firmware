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
 * DMR (ETSI TS 102 361-1 Annex B.1.1/B.3.3/B.3.4) BPTC(196,96): the FEC
 * that protects a Data-type burst's CSBK/Data Header/Multi Block Control
 * payload, and (via the voice-superframe EMB fragment path -- not
 * implemented here) Full/Short Link Control. This file only handles the
 * "whole 196 bits in one burst" case (Data-type bursts); the fragmented
 * voice-embedded-signalling variant is a separate, not-yet-implemented
 * path.
 *
 * Structure (ETSI Annex B.1.1, Fig B.1/Table B.1-B.3), confirmed against
 * seven independent open-source implementations (MMDVMHost, go-dmr,
 * dmrlib, dmrshark, dsd-dmr, ok-dmrlib, dsd-fme) plus the ETSI standard
 * text directly:
 *
 *   - The 196 received bits are deinterleaved by a single closed-form
 *     permutation: deinter[a] = raw[(a * 181) % 196] for a in 0..195.
 *   - deinter[0] is a reserved bit (R3), discarded.
 *   - deinter[1..195] forms a 13-row x 15-column matrix, row-major:
 *     rows 0-8 are "data rows" (11 info bits in cols 0-10 + 4 Hamming
 *     (15,11,3) row-parity bits in cols 11-14); rows 9-12 are pure
 *     Hamming(13,9,3) column-parity rows protecting each of the 15
 *     columns' 9 data-row bits.
 *   - Correction: iterate row-then-column Hamming fixups a few times
 *     (fixing one axis changes the other axis's syndrome).
 *   - Payload extraction (96 bits total): row 0's columns 0-2 are 3
 *     more reserved bits (R0-R2, discarded); row 0 cols 3-10 (8 bits)
 *     plus rows 1-8 cols 0-10 (11 bits each, 88 bits) = 96 payload
 *     bits, MSB-first in that row-major reading order -- this IS the
 *     raw CSBK/Data-Header/MBC PDU (which carries its own CRC as part
 *     of those 96 bits; BPTC provides only the FEC, nothing else).
 */

#ifndef __DMR_BPTC19696_H__
#define __DMR_BPTC19696_H__

#include <cstdint>

namespace dmr {

// bits196: one bit per byte (0 or 1), raw received (still-interleaved)
// order, exactly the burst's Info1(98 bits) followed by Info2(98 bits).
// out96: one bit per byte (0 or 1), MSB-first PDU order, on success.
// Returns false if a row or column had more than 1 bit wrong (i.e. an
// uncorrectable block) -- callers should treat out96 as unreliable in
// that case (the caller's own PDU-level CRC is still the real gate on
// trusting the content either way).
bool bptc19696_decode(const uint8_t bits196[196], uint8_t out96[96]);

}  // namespace dmr

#endif /*__DMR_BPTC19696_H__*/
