/*
 * Encode/decode tables ported from OpenGD77's QR1676.c
 * (https://github.com/rogerclarkmelbourne/OpenGD77):
 *
 *   Copyright (C) 2015, 2016 by Jonathan Naylor G4KLX
 *   Ported to OpenGD77 by Roger Clark VK3KYY / G4KYF
 *
 * Ported to PortaPack/Mayhem (decoder-only; also fixes a final-shift
 * bug in the original decoder -- see dmr_qr1676.cpp), Copyright (C)
 * 2026 Jarkko Vääräniemi.
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

#ifndef __DMR_QR1676_H__
#define __DMR_QR1676_H__

#include <cstdint>

namespace dmr {

// Decodes the 16-bit "EMB" (EMbedded signalling Block) field that DMR
// voice frames B-F carry in place of a real sync word. Per ETSI TS 102
// 361-1, a voice burst's 48-bit sync-word-position field is split as:
// 8 bits EMB (first half) + 32 bits embedded-signalling data payload +
// 8 bits EMB (second half). The two 8-bit EMB halves concatenate into a
// single 16-bit codeword protected by a QR(16,7,6) code, carrying 7 bits
// of info: Colour Code (4 bits), PI/Privacy Indicator (1 bit), and LCSS
// (Link Control Start/Stop, 2 bits) -- LCSS is what actually tells a
// receiver which of the 4 embedded-LC fragments a given burst carries;
// see dmr_symbol_processor.hpp for how that sequencing works.
//
// Ported from OpenGD77's firmware/source/hotspot/QR1676.c (itself
// credited to Jonathan Naylor G4KLX, of MMDVM/dsd-fme lineage). Only the
// decoder is ported -- we never need to encode.
//
// emb_byte0/emb_byte1 are the two EMB halves packed MSB-first, exactly
// as extracted bit-by-bit from the burst (bits 108-115 and 148-155 in
// DmrSymbolProcessor's absolute burst-relative numbering).
//
// Returns the corrected 7-bit info word: bit 6..3 = Colour Code,
// bit 2 = PI, bits 1..0 = LCSS. (This is genuinely error-corrected --
// not merely validated -- unlike OpenGD77's own uiHotspot.c consumer,
// which was found on inspection to call this decoder but then read the
// PRE-correction raw bits anyway; we use the corrected value.)
uint8_t qr1676_decode(uint8_t emb_byte0, uint8_t emb_byte1);

}  // namespace dmr

#endif /*__DMR_QR1676_H__*/
