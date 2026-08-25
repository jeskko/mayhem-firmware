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
 * CACH (Common Announcement Channel, ETSI TS 102 361-1 SS6.3/Table 9.5,
 * Tier II only) and the Short Link Control PDU (SS9.1.7) it carries,
 * spread 17 bits at a time across 4 consecutive bursts' CACH fields.
 *
 * Every field layout and bit position here was read directly out of
 * dsd-fme's source (src/dmr_bs.c, src/dmr_flco.c, src/dmr_utils.c) --
 * the same live, working reference decoder used throughout this
 * project -- rather than reconstructed from notes, specifically to
 * avoid transcription drift on a piece this fiddly (three nested
 * interleaves/FEC layers before a single usable bit comes out).
 *
 * One correction made relative to that reference: dsd-fme's own crc8_ok
 * call (dmr_flco.c's dmr_cach()) zeroes bits[36..67] and THEN compares
 * the computed CRC-8 against bits[36..43] of that now-zeroed range --
 * i.e. it always compares against 0x00, not the real transmitted CRC.
 * That bug happens not to matter much for dsd-fme's own display (SLCO
 * content still gets *shown*, just without a real CRC gate protecting
 * it), but this file computes and checks the CRC-8 properly per ETSI
 * Annex B.3.7 (poly x^8+x^2+x+1, init 0, no reflection) over the actual
 * SLCO(4)+Data(24) bits against the actual transmitted CRC(8) bits.
 */

#ifndef __DMR_CACH_H__
#define __DMR_CACH_H__

#include <array>
#include <cstdint>

namespace dmr {

// Single-CACH-field decode: the 7-bit Hamming(7,4,3)-protected TACT
// header (Access Type, TC, LCSS) plus the 17-bit unprotected Short LC
// fragment it carries (ETSI is explicit that the CACH provides no FEC
// for these 17 bits at all -- whatever protection exists is applied to
// the assembled 4-fragment PDU instead, see ShortLcAssembler below).
struct CachHeader {
    bool tact_ok{false};                 // Hamming(7,4,3) found <=1 bit error (or none)
    bool at{false};                      // Access Type
    bool tc{false};                      // TDMA channel (which of TS1/TS2 the NEXT burst is)
    uint8_t lcss{0};                     // Link Control Start/Stop: 0=single,1=first,2=last,3=continuation
    std::array<uint8_t, 17> fragment{};  // one bit per byte, MSB-first field order
};

// cach_bits: the 24-bit CACH field immediately preceding a burst (see
// DmrSymbolProcessor::Burst::cach_bits), one bit per byte, MSB-first
// packed exactly like Burst::bits.
//
// Defined `inline` right here (not out-of-line in dmr_cach.cpp) so it
// compiles directly into every translation unit that calls it --
// notably ui_dmr_rx.cpp -- instead of requiring a cross-compilation-unit
// call through this app's 32KB external-app relocatable region. A
// cross-TU call through that region's linkage was found to be
// unreliable, with everything about the callee's own body otherwise
// verified correct by static analysis (DWARF layout, linker symbol
// placement, veneer disassembly); making it inline sidesteps that
// call/veneer path entirely.
namespace detail {

// ETSI TS 102 361-1 SS6.3: raw air-interface bit position -> field-order
// position within a single 24-bit CACH field. Confirmed directly against
// dsd-fme's src/dmr_bs.c `cachInterleave[24]` table (their dibit-pair
// extraction populates cachdata[cachInterleave[i]] from raw bit i, which
// is exactly this permutation applied the same direction here).
constexpr int CACH_INTERLEAVE[24] = {
    0, 7, 8, 9, 1, 10,
    11, 12, 2, 13, 14,
    15, 3, 16, 4, 17, 18,
    19, 5, 20, 21, 22, 6, 23};

// Hamming(7,4,3) over the TACT header (AT, TC, LCSS0, LCSS1 + 3 parity),
// ETSI Annex B.3.5 Table B.17. bits[0..3]=data, bits[4..6]=parity,
// corrected in place; returns false only on an uncorrectable (2+ bit)
// error. Parity contributions/one-hot syndromes confirmed against
// dsd-fme's src/fec.c Hamming_7_4_m_H/Hamming_7_4_m_corr tables.
inline bool hamming_7_4_decode(uint8_t bits[7]) {
    constexpr uint8_t data_syndrome[4] = {0b101, 0b111, 0b110, 0b011};

    uint8_t calc = 0;
    for (int i = 0; i < 4; i++)
        if (bits[i]) calc ^= data_syndrome[i];

    uint8_t received = static_cast<uint8_t>((bits[4] << 2) | (bits[5] << 1) | bits[6]);
    const uint8_t syndrome = calc ^ received;
    if (syndrome == 0) return true;

    for (int i = 0; i < 4; i++) {
        if (data_syndrome[i] == syndrome) {
            bits[i] ^= 1;
            return true;
        }
    }
    for (int i = 0; i < 3; i++) {
        if (syndrome == static_cast<uint8_t>(1u << (2 - i))) {
            bits[4 + i] ^= 1;
            return true;
        }
    }
    return false;
}

}  // namespace detail

inline CachHeader cach_decode_header(const uint8_t cach_bits[24]) {
    CachHeader h{};

    uint8_t field[24];
    for (int i = 0; i < 24; i++) {
        const uint8_t raw_bit = (cach_bits[i >> 3] >> (7 - (i & 7))) & 1;
        field[detail::CACH_INTERLEAVE[i]] = raw_bit;
    }

    uint8_t tact[7];
    for (int i = 0; i < 7; i++) tact[i] = field[i];
    h.tact_ok = detail::hamming_7_4_decode(tact);
    h.at = tact[0] != 0;
    h.tc = tact[1] != 0;
    h.lcss = static_cast<uint8_t>((tact[2] << 1) | tact[3]);

    for (int i = 0; i < 17; i++) h.fragment[i] = field[7 + i];

    return h;
}

struct ShortLcResult {
    bool recognized{false};  // true iff slco is a value parsed below
    uint8_t slco{0};

    // SLCO=0x1 Activity Update (ETSI TS 102 361-2 SS7.1.3.2): coarse
    // per-slot activity state, available even with no active call.
    struct ActivityUpdate {
        uint8_t ts1_activity{0};
        uint8_t ts2_activity{0};
        uint8_t ts1_hash{0};
        uint8_t ts2_hash{0};
    };

    // SLCO=0xF: Motorola Capacity Plus (FID 0x10 CSBK family) site
    // beacon carried via Short LC instead of CSBK -- confirmed against
    // dsd-fme's own decoding of this opcode. This is the repeater/site
    // ID source for that system -- its CSBK traffic can be noisy/
    // unreliable, but Short LC repeats constantly regardless of call
    // activity.
    struct CapacityPlusSite {
        uint8_t site_id{0};
        uint8_t rest_channel{0};
        uint8_t reserved{0};
    };

    bool has_activity{false};
    ActivityUpdate activity{};
    bool has_capacity_plus_site{false};
    CapacityPlusSite capacity_plus_site{};
};

// The 4-fragment assembler: feed it every CACH header this app decodes
// (regardless of LCSS value -- it tracks the sequence itself), get back
// a validated ShortLcResult once a complete 4-fragment sequence's BPTC-
// style FEC (3x Hamming(17,12,3) rows) and final CRC-8 both pass.
class ShortLcAssembler {
   public:
    // Returns true if `result` was populated (a complete, CRC-verified
    // Short LC PDU just finished assembling).
    bool add_fragment(const CachHeader& header, ShortLcResult& result);

   private:
    void reset();

    uint8_t fragments_[4][17]{};
    int next_index_{-1};  // -1 = not currently assembling

    // Working buffers for the assembly step, held as members rather
    // than locals in add_fragment() -- this runs on every single burst
    // from the application thread's on_data_dmr(), which already has
    // its own sizeable locals (DmrChannelDecoder::decode_burst()'s
    // 196+96-byte BPTC buffers); keeping these off that call's stack
    // frame avoids stacking two large scratch buffers on top of each
    // other on what's likely a small Cortex-M0 thread stack.
    uint8_t raw_[68]{};
    uint8_t deint_[68]{};
    uint8_t pdu_[36]{};
};

}  // namespace dmr

#endif /*__DMR_CACH_H__*/
