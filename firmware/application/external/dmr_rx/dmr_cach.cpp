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

#include "dmr_cach.hpp"

namespace dmr {
namespace {

// Hamming(17,12,3), one 17-bit block: d[0..11]=data, d[12..16]=parity.
// Corrected in place; false only on an uncorrectable error. Ported
// directly from dsd-fme's src/dmr_utils.c Hamming17123() (itself citing
// the ETSI Annex B.3.3 parity equations) rather than re-derived, since a
// transcription slip here would silently corrupt every Short LC decode.
bool hamming_17_12_decode(uint8_t* d) {
    const bool c0 = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[6] ^ d[7] ^ d[9];
    const bool c1 = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[7] ^ d[8] ^ d[10];
    const bool c2 = d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[8] ^ d[9] ^ d[11];
    const bool c3 = d[0] ^ d[1] ^ d[4] ^ d[5] ^ d[7] ^ d[10];
    const bool c4 = d[0] ^ d[1] ^ d[2] ^ d[5] ^ d[6] ^ d[8] ^ d[11];

    unsigned n = 0;
    n |= (c0 != d[12]) ? 0x01u : 0u;
    n |= (c1 != d[13]) ? 0x02u : 0u;
    n |= (c2 != d[14]) ? 0x04u : 0u;
    n |= (c3 != d[15]) ? 0x08u : 0u;
    n |= (c4 != d[16]) ? 0x10u : 0u;

    switch (n) {
        case 0x01u:
            d[12] = !d[12];
            return true;
        case 0x02u:
            d[13] = !d[13];
            return true;
        case 0x04u:
            d[14] = !d[14];
            return true;
        case 0x08u:
            d[15] = !d[15];
            return true;
        case 0x10u:
            d[16] = !d[16];
            return true;
        case 0x1Bu:
            d[0] = !d[0];
            return true;
        case 0x1Fu:
            d[1] = !d[1];
            return true;
        case 0x17u:
            d[2] = !d[2];
            return true;
        case 0x07u:
            d[3] = !d[3];
            return true;
        case 0x0Eu:
            d[4] = !d[4];
            return true;
        case 0x1Cu:
            d[5] = !d[5];
            return true;
        case 0x11u:
            d[6] = !d[6];
            return true;
        case 0x0Bu:
            d[7] = !d[7];
            return true;
        case 0x16u:
            d[8] = !d[8];
            return true;
        case 0x05u:
            d[9] = !d[9];
            return true;
        case 0x0Au:
            d[10] = !d[10];
            return true;
        case 0x14u:
            d[11] = !d[11];
            return true;
        case 0x00u:
            return true;
        default:
            return false;
    }
}

// ETSI Annex B.3.7: CRC-8, G8(x)=x^8+x^2+x+1 (poly 0x07), init 0, no
// reflection, no final complement (unlike the CSBK CRC-16 in
// dmr_csbk.cpp, which does complement+mask -- this one doesn't).
uint8_t crc8(const uint8_t* bits, int n) {
    uint8_t crc = 0;
    for (int i = 0; i < n; i++) {
        const uint8_t bit = bits[i] & 1;
        const uint8_t msb = static_cast<uint8_t>((crc >> 7) & 1);
        crc = static_cast<uint8_t>((crc << 1) & 0xFF);
        if (msb ^ bit) crc ^= 0x07;
    }
    return crc;
}

uint32_t bits_to_u32(const uint8_t* bits, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 1) | (bits[i] & 1);
    return v;
}

}  // namespace

void ShortLcAssembler::reset() {
    next_index_ = -1;
}

bool ShortLcAssembler::add_fragment(const CachHeader& header, ShortLcResult& result) {
    if (!header.tact_ok) {
        reset();
        return false;
    }

    // Mirrors dsd-fme's dmr_cach() sequencing exactly: LCSS=1 (first)
    // resets to index 0, LCSS=3 (continuation) advances, LCSS=2 (last)
    // forces index 3 and triggers assembly regardless of how it got
    // there -- the Hamming(17,12,3)x3 + CRC-8 checks below are the real
    // gate, not strict sequencing (a fragment lost off-air is just as
    // likely to produce a checksum failure as an out-of-order one).
    int index;
    if (header.lcss == 1) {
        next_index_ = 0;
        index = 0;
    } else if (header.lcss == 3) {
        if (next_index_ < 0) return false;  // continuation with no first fragment seen yet
        index = ++next_index_;
    } else if (header.lcss == 2) {
        if (next_index_ < 0) return false;
        index = 3;
    } else {
        // LCSS=0 ("single fragment"): ETSI's own table lists this but
        // notes no single-fragment Short LC PDU is actually defined --
        // dsd-fme's own comment agrees ("no support"). Not handled.
        reset();
        return false;
    }

    if (index > 3) {
        reset();
        return false;
    }

    for (int i = 0; i < 17; i++) fragments_[index][i] = header.fragment[i];

    if (header.lcss != 2) return false;  // not the last fragment yet

    // Assemble: 4x17 = 68 raw bits, ETSI's secondary interleave
    // (confirmed against dsd-fme: src=(i*4)%67 for i<67, identity for
    // the last bit), then Hamming(17,12,3) over each of the first 3
    // 17-bit blocks (the 4th is a column-parity row this project
    // doesn't check either, matching dsd-fme -- CRC-8 below is the
    // real final gate regardless).
    for (int frag = 0; frag < 4; frag++)
        for (int i = 0; i < 17; i++)
            raw_[frag * 17 + i] = fragments_[frag][i];

    for (int i = 0; i < 67; i++) deint_[i] = raw_[(i * 4) % 67];
    deint_[67] = raw_[67];

    reset();  // done with this sequence either way

    const bool h1 = hamming_17_12_decode(deint_ + 0);
    const bool h2 = hamming_17_12_decode(deint_ + 17);
    const bool h3 = hamming_17_12_decode(deint_ + 34);
    if (!h1 || !h2 || !h3) return false;

    // Compact: each Hamming(17,12,3) block's first 12 bits are its real
    // data; drop the 5 parity bits from each to get 3*12=36 bits.
    for (int i = 0; i < 12; i++) pdu_[i] = deint_[i];
    for (int i = 0; i < 12; i++) pdu_[12 + i] = deint_[17 + i];
    for (int i = 0; i < 12; i++) pdu_[24 + i] = deint_[34 + i];

    const uint8_t computed_crc = crc8(pdu_, 28);
    const uint8_t received_crc = static_cast<uint8_t>(bits_to_u32(pdu_ + 28, 8));
    if (computed_crc != received_crc) return false;

    result = ShortLcResult{};
    result.slco = static_cast<uint8_t>(bits_to_u32(pdu_, 4));

    if (result.slco == 0x0) {
        result.recognized = true;
    } else if (result.slco == 0x1) {
        result.recognized = true;
        result.has_activity = true;
        result.activity.ts1_activity = static_cast<uint8_t>(bits_to_u32(pdu_ + 4, 4));
        result.activity.ts2_activity = static_cast<uint8_t>(bits_to_u32(pdu_ + 8, 4));
        result.activity.ts1_hash = static_cast<uint8_t>(bits_to_u32(pdu_ + 12, 8));
        result.activity.ts2_hash = static_cast<uint8_t>(bits_to_u32(pdu_ + 20, 8));
    } else if (result.slco == 0xF) {
        // Motorola Capacity Plus/Cap Max site beacon. Field offsets
        // confirmed against dsd-fme's dmr_flco.c dmr_slco():
        // rest_channel=bits[16..19], reserved=bits[20..21],
        // site_id=bits[22..24] (their own comment: "seems more
        // consistent" -- i.e. empirically tuned, not from a public
        // spec table; flagged here for the same reason).
        result.recognized = true;
        result.has_capacity_plus_site = true;
        result.capacity_plus_site.rest_channel = static_cast<uint8_t>(bits_to_u32(pdu_ + 16, 4));
        result.capacity_plus_site.reserved = static_cast<uint8_t>(bits_to_u32(pdu_ + 20, 2));
        result.capacity_plus_site.site_id = static_cast<uint8_t>(bits_to_u32(pdu_ + 22, 3));
    }

    return true;
}

}  // namespace dmr
