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

#include "dmr_csbk.hpp"

namespace dmr {
namespace {

uint64_t bits_to_u64(const uint8_t* bits, int n) {
    uint64_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 1) | (bits[i] & 1);
    return v;
}

// Standard bit-serial CRC-CCITT (poly 0x1021, init 0x0000, MSB-first,
// no reflection) -- the classic "shift and conditionally XOR the top
// bit" polynomial-division form, computing (x^16 * M(x)) mod G(x)
// directly one message bit at a time (ETSI Annex B.3.8).
uint16_t crc_ccitt(const uint8_t* bits, int n) {
    uint16_t crc = 0x0000;
    for (int i = 0; i < n; i++) {
        const uint8_t bit = bits[i] & 1;
        const uint8_t msb = static_cast<uint8_t>((crc >> 15) & 1);
        crc = static_cast<uint16_t>((crc << 1) & 0xFFFF);
        if (msb ^ bit) crc ^= 0x1021;
    }
    return crc;
}

constexpr uint16_t CSBK_CRC_MASK = 0xA5A5;

}  // namespace

CsbkHeader csbk_decode_header(const uint8_t pdu96[96]) {
    CsbkHeader h{};
    h.last_block = pdu96[0] != 0;
    h.protect_flag = pdu96[1] != 0;
    h.csbko = static_cast<uint8_t>(bits_to_u64(&pdu96[2], 6));
    h.fid = static_cast<uint8_t>(bits_to_u64(&pdu96[8], 8));
    h.data = bits_to_u64(&pdu96[16], 64);

    const uint16_t calc = static_cast<uint16_t>((crc_ccitt(pdu96, 80) ^ 0xFFFF) ^ CSBK_CRC_MASK);
    const uint16_t received = static_cast<uint16_t>(bits_to_u64(&pdu96[80], 16));
    h.crc_ok = (calc == received);

    return h;
}

CapacityPlusAdjacentSites csbk_decode_capacity_plus_adjacent_sites(const CsbkHeader& header) {
    CapacityPlusAdjacentSites result{};
    if (header.fid != 0x10 || header.csbko != 0x3B || !header.crc_ok) return result;

    result.recognized = true;
    // header.data's bit 63 is Data-relative bit 0 (== pdu96[16]) --
    // dsd-fme's cs_pdu_bits[32+8i]/[36+8i] are therefore Data-relative
    // bits (16+8i)/(20+8i), i.e. header.data bits (63-(16+8i))=(47-8i)
    // downto (44-8i) for the site number, and (43-8i) downto (40-8i)
    // for the rest channel.
    for (int i = 0; i < 6; i++) {
        const uint8_t site_id = static_cast<uint8_t>((header.data >> (44 - 8 * i)) & 0xF);
        const uint8_t rest_channel = static_cast<uint8_t>((header.data >> (40 - 8 * i)) & 0xF);
        auto& site = result.sites[static_cast<unsigned>(i)];
        site.valid = site_id != 0;
        site.site_id = site_id;
        site.rest_channel = rest_channel;
    }
    return result;
}

CapacityPlusChannelStatus csbk_decode_capacity_plus_channel_status(const CsbkHeader& header) {
    CapacityPlusChannelStatus result{};
    if (header.fid != 0x10 || header.csbko != 0x3E || !header.crc_ok) return result;

    result.recognized = true;
    // Field-start-bit k (0-based, MSB-first within Data) + width w bits
    // -> shift = 64-k-w into header.data, mask (1<<w)-1 -- same
    // derivation as Adjacent Sites above. k values confirmed against
    // dsd-fme's cs_pdu_bits[16]/[18]/[20]/[24] (Data-relative bit =
    // cs_pdu_bits index - 16).
    result.fl = static_cast<uint8_t>((header.data >> 62) & 0x3);
    result.ts = static_cast<uint8_t>((header.data >> 61) & 0x1);
    result.rest_channel = static_cast<uint8_t>((header.data >> 56) & 0xF);

    if (result.fl == 2 || result.fl == 3) {
        result.has_channel_map = true;
        const uint8_t bank_one = static_cast<uint8_t>((header.data >> 48) & 0xFF);
        for (int i = 0; i < 8; i++)
            result.channel_busy[static_cast<unsigned>(i)] = ((bank_one >> (7 - i)) & 1) != 0;
    }
    return result;
}

ChannelGrant csbk_decode_channel_grant(const CsbkHeader& header) {
    ChannelGrant result{};
    if (header.csbko < 48 || header.csbko > 56 || !header.crc_ok) return result;

    result.recognized = true;
    result.csbko = header.csbko;
    // k/width -> shift=64-k-w, same derivation as elsewhere in this
    // file. k values confirmed against dsd-fme's cs_pdu_bits[16]
    // (lpchannum, 12 bits) / [28]/[29]/[30]/[31] (1 bit each) / [32]
    // (target, 24 bits) / [56] (source, 24 bits) -- Data-relative bit =
    // cs_pdu_bits index - 16.
    result.logical_channel = static_cast<uint16_t>((header.data >> 52) & 0xFFF);
    result.timeslot = static_cast<uint8_t>((header.data >> 51) & 0x1);
    // bit at k=14 (dsd-fme's "st2") is documented there as the
    // emergency flag; k=13/15 ("st1"/"st3") are context-dependent
    // (late-entry/hi-rate/offset/call-direction depending on which of
    // the 9 grant variants this is) and not decoded here.
    result.emergency = ((header.data >> 49) & 0x1) != 0;
    result.target_address = static_cast<uint32_t>((header.data >> 24) & 0xFFFFFF);
    result.source_address = static_cast<uint32_t>(header.data & 0xFFFFFF);
    return result;
}

namespace {
// Connect Plus's Data field happens to be byte-aligned in every field
// dsd-fme reads (cs_pdu[2] is Data byte 0, i.e. header.data's top byte),
// so pulling individual bytes out of header.data here mirrors dsd-fme's
// own cs_pdu[N] indexing exactly: byte i = bits (63-8*i) downto (56-8*i).
uint8_t data_byte(uint64_t data, int i) {
    return static_cast<uint8_t>((data >> (56 - 8 * i)) & 0xFF);
}
}  // namespace

ConnectPlusAdjacentSites csbk_decode_connect_plus_adjacent_sites(const CsbkHeader& header) {
    ConnectPlusAdjacentSites result{};
    if (header.fid != 0x06 || header.csbko != 0x01 || !header.crc_ok) return result;

    result.recognized = true;
    for (int i = 0; i < 5; i++)
        result.sites[static_cast<unsigned>(i)] = data_byte(header.data, i) & 0x3F;
    return result;
}

ConnectPlusVoiceGrant csbk_decode_connect_plus_voice_grant(const CsbkHeader& header) {
    ConnectPlusVoiceGrant result{};
    if (header.fid != 0x06 || header.csbko != 0x03 || !header.crc_ok) return result;

    result.recognized = true;
    result.source_address = (static_cast<uint32_t>(data_byte(header.data, 0)) << 16) |
                            (static_cast<uint32_t>(data_byte(header.data, 1)) << 8) |
                            data_byte(header.data, 2);
    result.target_address = (static_cast<uint32_t>(data_byte(header.data, 3)) << 16) |
                            (static_cast<uint32_t>(data_byte(header.data, 4)) << 8) |
                            data_byte(header.data, 5);
    const uint8_t byte6 = data_byte(header.data, 6);
    result.logical_channel = static_cast<uint8_t>((byte6 & 0xF0) >> 4);
    result.timeslot = static_cast<uint8_t>((byte6 & 0x08) >> 3);
    result.call_option = data_byte(header.data, 7);
    return result;
}

ConnectPlusDataGrant csbk_decode_connect_plus_data_grant(const CsbkHeader& header) {
    ConnectPlusDataGrant result{};
    if (header.fid != 0x06 || header.csbko != 0x06 || !header.crc_ok) return result;

    result.recognized = true;
    result.target_address = (static_cast<uint32_t>(data_byte(header.data, 0)) << 16) |
                            (static_cast<uint32_t>(data_byte(header.data, 1)) << 8) |
                            data_byte(header.data, 2);
    const uint8_t byte3 = data_byte(header.data, 3);
    result.logical_channel = static_cast<uint8_t>((byte3 & 0xF0) >> 4);
    result.timeslot = static_cast<uint8_t>((byte3 & 0x08) >> 3);
    return result;
}

ConnectPlusSlotTermination csbk_decode_connect_plus_slot_termination(const CsbkHeader& header) {
    ConnectPlusSlotTermination result{};
    if (header.fid != 0x06 || header.csbko != 0x0C || !header.crc_ok) return result;

    result.recognized = true;
    result.target_address = (static_cast<uint32_t>(data_byte(header.data, 0)) << 16) |
                            (static_cast<uint32_t>(data_byte(header.data, 1)) << 8) |
                            data_byte(header.data, 2);
    return result;
}

StandardCsbk csbk_decode_standard(const CsbkHeader& header) {
    StandardCsbk result{};
    if (!header.crc_ok) return result;

    // k/width -> shift=64-k-w, same derivation as elsewhere in this file.
    // Data-relative bit = dsd-fme's cs_pdu_bits index - 16.
    switch (header.csbko) {
        case 7:  // Channel Timing CSBK (CT_CSBK) -- no fields
            result.recognized = true;
            result.type = StandardCsbkType::CHANNEL_TIMING;
            break;
        case 38:  // NACK Response (NACK_Rsp) -- target k=16/w=24, source k=40/w=24
            result.recognized = true;
            result.type = StandardCsbkType::NACK_RESPONSE;
            result.target_address = static_cast<uint32_t>((header.data >> 24) & 0xFFFFFF);
            result.source_address = static_cast<uint32_t>(header.data & 0xFFFFFF);
            break;
        case 40:  // Announcements (C_BCAST) -- a_type k=0/w=5. Only the
                  // sub-type classification is decoded, see the header comment.
            result.recognized = true;
            result.type = StandardCsbkType::ANNOUNCEMENT;
            result.announcement_type = static_cast<uint8_t>((header.data >> 59) & 0x1F);
            break;
        case 56:  // BS Outbound Activation (BS_Dwn_Act) -- same field
                  // layout as NACK_Rsp. dsd-fme additionally gates this on
                  // the burst's sync type being MS Data (it's normally an
                  // inbound wake-up call); not checked here since CsbkHeader
                  // doesn't carry sync type -- see caller if that matters.
            result.recognized = true;
            result.type = StandardCsbkType::BS_OUTBOUND_ACTIVATION;
            result.target_address = static_cast<uint32_t>((header.data >> 24) & 0xFFFFFF);
            result.source_address = static_cast<uint32_t>(header.data & 0xFFFFFF);
            break;
        case 57:  // Move (C_MOVE) -- no fields decoded in the reference
            result.recognized = true;
            result.type = StandardCsbkType::MOVE;
            break;
        case 61:  // Preamble CSBK -- content k=0/w=1, gi k=1/w=1,
                  // blocks k=8/w=8, target k=16/w=24, source k=40/w=24
            result.recognized = true;
            result.type = StandardCsbkType::PREAMBLE;
            result.preamble_is_data = ((header.data >> 63) & 0x1) != 0;
            // dsd-fme: `if (gi == 0) "Individual"; else "Group"` -- gi==1 means group.
            result.preamble_is_group = ((header.data >> 62) & 0x1) != 0;
            result.preamble_blocks = static_cast<uint8_t>((header.data >> 48) & 0xFF);
            result.target_address = static_cast<uint32_t>((header.data >> 24) & 0xFFFFFF);
            result.source_address = static_cast<uint32_t>(header.data & 0xFFFFFF);
            break;
        default:
            break;
    }
    return result;
}

}  // namespace dmr
