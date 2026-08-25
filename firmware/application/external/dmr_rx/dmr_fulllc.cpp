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

#include "dmr_fulllc.hpp"
#include "dmr_rs129.hpp"

namespace dmr {
namespace {

uint32_t bits_to_u32(const uint8_t* bits, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 1) | (bits[i] & 1);
    return v;
}

// ETSI Annex B.3.6 RS(12,9) masks, shared with dmr_csbk.cpp's CRC-CCITT
// mask convention (a different check, same idea: distinguish PDU types
// sharing one code so a misidentified type reliably fails rather than
// validating by chance) -- confirmed against dsd-fme's src/dmr_dburst.c
// (`databurst==0x01` VLC -> 0x969696, `databurst==0x02` TLC -> 0x999999).
constexpr uint32_t RS_MASK_VOICE_LC_HEADER = 0x969696;
constexpr uint32_t RS_MASK_TERMINATOR_WITH_LC = 0x999999;

}  // namespace

FullLcResult fulllc_decode(const uint8_t pdu96[96], bool is_terminator) {
    FullLcResult r{};

    uint8_t codeword[12];
    for (int i = 0; i < 12; i++) codeword[i] = static_cast<uint8_t>(bits_to_u32(&pdu96[i * 8], 8));

    const uint32_t mask = is_terminator ? RS_MASK_TERMINATOR_WITH_LC : RS_MASK_VOICE_LC_HEADER;
    if (!rs129_correct(codeword, mask)) return r;  // uncorrectable -- reject outright

    // Re-derive bit-per-byte fields from the (possibly RS-corrected)
    // codeword bytes rather than the original pdu96 array, so a
    // corrected byte actually takes effect.
    r.flco = static_cast<uint8_t>(codeword[0] & 0x3F);
    r.fid = codeword[1];

    if (r.flco != static_cast<uint8_t>(Flco::GROUP_VOICE) &&
        r.flco != static_cast<uint8_t>(Flco::PRIVATE_VOICE)) {
        return r;
    }

    r.service_options = codeword[2];
    r.dest_address = (static_cast<uint32_t>(codeword[3]) << 16) | (static_cast<uint32_t>(codeword[4]) << 8) | codeword[5];
    r.source_address = (static_cast<uint32_t>(codeword[6]) << 16) | (static_cast<uint32_t>(codeword[7]) << 8) | codeword[8];
    r.recognized = true;
    return r;
}

ServiceOptions decode_service_options(uint8_t so, uint8_t fid) {
    ServiceOptions r{};
    r.emergency = (so & 0x80) != 0;
    r.privacy = (so & 0x40) != 0;
    r.broadcast = (so & 0x08) != 0;
    r.ovcm = (so & 0x04) != 0;
    r.priority = so & 0x03;
    if (fid == 0x10) {
        r.moto_txi = (so & 0x20) != 0;
        r.moto_rpt = (so & 0x10) != 0;
    }
    return r;
}

}  // namespace dmr
