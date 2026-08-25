/*
 * GF(256) tables, generator, and the Berlekamp-Massey/Chien-search/Forney
 * decode algorithm below are ported verbatim from dmrshark's rs-12-9.c
 * (https://github.com/nonoo/dmrshark, libs/coding/rs-12-9.c). That file
 * carries no individual author name, only:
 *
 *   This file is part of dmrshark.
 *   dmrshark is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published
 *   by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 * dmrshark only offers GPLv3-or-later, not v2 -- unlike most other files
 * in this project, this file (and any combined work containing it) is
 * therefore bound by GPLv3-or-later terms specifically.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Wrapper/integration code Copyright (C) 2026 Jarkko Vääräniemi.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
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
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Reed-Solomon (12,9) over GF(256) -- ETSI TS 102 361-1 Annex B.3.6, the
 * integrity check for Full Link Control (a 12-byte codeword: 9 data
 * bytes + 3 parity bytes, 3 parity bytes meaning up to 1 byte can be
 * corrected). Ported directly from dmrshark's rs-12-9.c (also used
 * as-is by dsd-fme) -- GF(256) log/exp tables, generator, and the
 * Berlekamp-Massey/Chien-search/Forney error-correction algorithm are
 * all copied verbatim rather than re-derived, since a transcription
 * slip anywhere in a Galois-field algorithm like this is exactly the
 * kind of bug that's silent until it corrects a codeword to the WRONG
 * value with high confidence -- far worse than not correcting at all.
 * Validated by synthetically flipping single bits in a known-good
 * codeword and confirming the exact original value comes back.
 *
 * This was added because relying on "2 consecutive identical decodes"
 * (SlotLcState::update() in ui_dmr_rx.cpp) as the only confidence gate
 * is insufficient on its own: real multipath/fading correlates bit
 * errors WITHIN a transmission rather than making them independent per
 * burst, so the same corruption of an address field can survive
 * several consecutive bursts unchanged, passing that heuristic despite
 * being wrong. RS(12,9) is the spec's own mechanism for exactly this.
 */

#ifndef __DMR_RS129_H__
#define __DMR_RS129_H__

#include <cstdint>

namespace dmr {

// codeword: 12 bytes, MSB-first-bit-packed same as the rest of this
// project's PDU handling (byte 0 = the FIRST 8 bits of the 96-bit PDU,
// ... byte 11 = the LAST 8, i.e. the last of the 3 RS parity bytes).
// mask: XORed into the last 3 (parity) bytes before checking -- ETSI
// uses a different mask per Data Type sharing this same RS code (e.g.
// 0x969696 for Voice LC Header, 0x999999 for Terminator with LC) so
// that a PDU misidentified as the wrong type reliably fails the check
// rather than accidentally validating.
//
// Returns true if the codeword is now valid (no errors found, or a
// single-byte error was found and corrected in place); false if
// uncorrectable (more corruption than 1 byte's worth), in which case
// codeword is left unmodified.
bool rs129_correct(uint8_t codeword[12], uint32_t mask);

}  // namespace dmr

#endif /*__DMR_RS129_H__*/
