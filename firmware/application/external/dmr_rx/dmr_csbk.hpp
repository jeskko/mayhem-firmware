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
 * Generic CSBK (Control Signalling Block, ETSI TS 102 361-1 Table 9.9)
 * header: LB(1b)+PF(1b)+CSBKO(6b), FID(8b), 64 bits of opcode-specific
 * Data (not parsed here), CRC-CCITT(16b, masked).
 *
 * Deliberately stops at the generic header + CRC check for now. CSBKO/
 * FID field layouts diverge heavily beyond this point: FID=0x00 has its
 * own standard (ETSI TS 102 361-4) opcode family, while Capacity Plus
 * (FID=0x10) and Connect Plus (FID=0x06) are DIFFERENT vendor-opcode
 * families with different CSBKO tables, and a given control channel may
 * run any of them. Reporting the generic header lets the real FID/CSBKO
 * values be observed before committing to a specific vendor's opcode
 * table for further decoding.
 *
 * CRC (ETSI Annex B.3.8 + B.3.12): CRC-CCITT (poly 0x1021, init 0, MSB-
 * first) over the 80-bit header+Data, then bitwise-complemented, then
 * XORed with the Data-Type-specific mask (0xA5A5 for CSBK) -- confirmed
 * against the ETSI standard text directly plus six independent open-
 * source implementations (MMDVMHost, go-dmr, dmrshark, SDRTrunk,
 * dsd-fme, DMRDecode), all in exact agreement.
 */

#ifndef __DMR_CSBK_H__
#define __DMR_CSBK_H__

#include <array>
#include <cstdint>

namespace dmr {

struct CsbkHeader {
    bool crc_ok{false};
    bool last_block{false};    // LB
    bool protect_flag{false};  // PF
    uint8_t csbko{0};          // 6-bit opcode
    uint8_t fid{0};            // vendor/feature ID: 0x00=standard, 0x10=Motorola Capacity+/Cap Max, 0x06=Motorola Connect+
    uint64_t data{0};          // the 64 opcode-specific bits, MSB-first, not decoded here
};

// pdu96: the output of bptc19696_decode() for a burst whose Data Type is CSBK.
CsbkHeader csbk_decode_header(const uint8_t pdu96[96]);

// Motorola Capacity Plus/Cap Max (FID=0x10) CSBKO=0x3B "Adjacent Sites"
// -- confirmed against dsd-fme's src/dmr_csbk.c (the `csbk_o == 0x3B`
// block): six fixed 4-bit-site-number + 4-bit-rest-channel pairs packed
// back to back in Data, starting 16 bits in (i.e. Data-relative bit 16,
// matching dsd-fme's cs_pdu_bits[32] since cs_pdu_bits[16] is Data's own
// first bit). A site number of 0 means "no site listed in this slot" --
// dsd-fme's own display only prints entries where the site number is
// nonzero, done the same way here via AdjacentSite::valid.
//
// This is a neighbor-site list -- a repeater periodically announces
// which OTHER sites are nearby and their rest channels, independent of
// (and complementary to) this site's own ID/rest channel from Short LC
// (see dmr_cach.hpp's CapacityPlusSite) or from this same CSBK family's
// far more involved CSBKO=0x3E "Channel Status" (deliberately not
// decoded here -- dsd-fme's own comments flag several of its fields as
// empirically reverse-engineered/uncertain, unlike this opcode's simple,
// fixed-size layout).
struct CapacityPlusAdjacentSites {
    bool recognized{false};  // fid==0x10, csbko==0x3B, crc_ok
    struct AdjacentSite {
        bool valid{false};  // site number was nonzero
        uint8_t site_id{0};
        uint8_t rest_channel{0};
    };
    std::array<AdjacentSite, 6> sites{};
};

CapacityPlusAdjacentSites csbk_decode_capacity_plus_adjacent_sites(const CsbkHeader& header);

// Motorola Capacity Plus/Cap Max (FID=0x10) CSBKO=0x3E "Channel Status"
// -- confirmed against dsd-fme's src/dmr_csbk.c (the `csbk_o == 0x3E`
// block), field-for-field cross-checked against dsd-fme's own decode of
// a captured burst.
//
// Deliberately scoped to SINGLE-BLOCK reports only (fl==2 "initial" or
// fl==3 "single", i.e. an all-idle/small system that fits its whole
// channel-activity map in one CSBK with no reassembly needed) -- a
// multi-block report (fl==0 "appended"/fl==1 "final", i.e. >8 logical
// channels) would need dsd-fme's own per-timeslot reassembly state
// machine (state->cap_plus_csbk_bits[ts][...] accumulated across up to
// 7 appended blocks) plus a private/data-call-target extraction whose
// own comments flag several byte offsets as "seems more consistent"
// (empirically tuned, not spec-derived) rather than confirmed -- not
// attempted here for the same reason Adjacent Sites was chosen over
// this opcode in the first place: real uncertainty in the reference
// implementation, not just complexity. A multi-block report is reported
// as recognized but with has_channel_map=false rather than guessed at.
struct CapacityPlusChannelStatus {
    bool recognized{false};  // fid==0x10, csbko==0x3E, crc_ok
    uint8_t fl{0};           // 0=appended, 1=final, 2=initial, 3=single
    uint8_t ts{0};           // which timeslot this report concerns
    uint8_t rest_channel{0};

    // Only true for fl==2/3 (single-block) -- see the class comment for
    // why fl==0/1 (multi-block, >8 channels) isn't decoded.
    bool has_channel_map{false};
    // Index 0 = logical channel/LSN 1, ... index 7 = LSN 8. true = busy
    // (group/private/data call in progress on that channel), false =
    // idle. The channel that equals rest_channel is the control/rest
    // channel rather than a "call" per se, same distinction dsd-fme's
    // own display makes (labelled "Rest" instead of "Idle"/"Active").
    std::array<bool, 8> channel_busy{};
};

CapacityPlusChannelStatus csbk_decode_capacity_plus_channel_status(const CsbkHeader& header);

// "Channel Grant CSBK/MBC PDU" family, CSBKO 48-56 -- confirmed against
// dsd-fme's src/dmr_csbk.c (the `csbk_o >= 48 && csbk_o <= 56` block,
// itself citing spec section "7.1.1.1.1 Channel Grant CSBK/MBC PDU").
// Deliberately NOT restricted to fid==0x10 here: dsd-fme's own reference
// evaluates this block completely unconditionally on FID -- the one CSBK
// family it treats as shared across vendor control-channel dialects
// rather than tied to a specific one. This is the actual mechanism used
// to announce which physical channel + talkgroup/radio ID an active
// call is being granted to (voice or data, group or private), as
// opposed to the FID=0x10-specific CSBKO=0x3A/0x3B/0x3E messages above,
// which are site/control-channel STATUS, not per-call grants.
//
// Caveat: unlike most other decoders in this file, this one has NOT yet
// been cross-checked against a real CRC-verified example -- implemented
// directly from source, but flagged as comparatively untested until a
// live one is seen.
struct ChannelGrant {
    bool recognized{false};
    uint8_t csbko{0};             // which of the 9 grant variants (48-56)
    uint16_t logical_channel{0};  // 12-bit logical/physical channel number
    uint8_t timeslot{0};          // 0 or 1 -- which TDMA slot this grant is on
    bool emergency{false};
    uint32_t target_address{0};  // talkgroup ID (group call) or dest radio ID
    uint32_t source_address{0};
};

ChannelGrant csbk_decode_channel_grant(const CsbkHeader& header);

// Motorola Connect Plus (FID=0x06) -- a DIFFERENT CSBK vendor family from
// Capacity Plus/Cap Max (FID=0x10) above, with its own opcode numbers
// and a byte-aligned (not just bit-aligned) field layout -- confirmed
// against dsd-fme's own `csbk_fid == 0x06` block. Field extraction here
// mirrors dsd-fme's own byte-indexed reads (cs_pdu[2] etc.) rather than
// re-deriving bit offsets, to avoid a transcription slip on the
// byte/bit-boundary conversion.
//
// Same caveat as ChannelGrant above: not yet cross-validated against a
// real CRC-verified Connect Plus example, unlike most of the Capacity
// Plus/Cap Max decoders in this file.

// CSBKO=0x01: up to 5 neighbor site numbers, 6 bits each -- note this
// is a DIFFERENT (simpler, no rest-channel-per-site) format than
// Capacity Plus's own Adjacent Sites (CSBKO=0x3B under FID=0x10).
struct ConnectPlusAdjacentSites {
    bool recognized{false};
    std::array<uint8_t, 5> sites{};  // 0 = "not listed" in that slot
};
ConnectPlusAdjacentSites csbk_decode_connect_plus_adjacent_sites(const CsbkHeader& header);

// CSBKO=0x03: grants a voice call (group or private) to a channel/slot.
struct ConnectPlusVoiceGrant {
    bool recognized{false};
    uint32_t target_address{0};  // talkgroup (call_option==2) or dest radio ID (==3)
    uint32_t source_address{0};
    uint8_t logical_channel{0};
    uint8_t timeslot{0};     // 0 or 1
    uint8_t call_option{0};  // 2=group voice, 3=private voice, other=unknown
};
ConnectPlusVoiceGrant csbk_decode_connect_plus_voice_grant(const CsbkHeader& header);

// CSBKO=0x06 (a Connect Plus CSBKO value -- unrelated to FID 0x06 also
// being 6, just a coincidence of numbering): grants a data call.
struct ConnectPlusDataGrant {
    bool recognized{false};
    uint32_t target_address{0};
    uint8_t logical_channel{0};
    uint8_t timeslot{0};
};
ConnectPlusDataGrant csbk_decode_connect_plus_data_grant(const CsbkHeader& header);

// CSBKO=0x0C: signals a call on the given target ending (a repeater
// returning that slot to the control channel).
struct ConnectPlusSlotTermination {
    bool recognized{false};
    uint32_t target_address{0};
};
ConnectPlusSlotTermination csbk_decode_connect_plus_slot_termination(const CsbkHeader& header);

// Standard ETSI CSBK family (ETSI TS 102 361-4) -- the vendor-agnostic
// opcodes referenced but deliberately deferred in this file's header
// comment above. Confirmed against dsd-fme's src/dmr_csbk.c, none of
// which gate these specific opcodes on FID (unlike e.g. Aloha/CSBKO=25,
// deliberately NOT covered here: its "system parameters" fields are
// mostly trunking/registration-backoff bookkeeping of limited value for
// a passive monitor, and its bit layout is one of the more involved and
// least confidently-sourced in the reference). Kept as one tagged
// result rather than one struct per opcode, since half of these carry
// no fields at all -- CHANNEL_TIMING/MOVE are just presence markers.
enum class StandardCsbkType : uint8_t {
    NONE = 0,
    CHANNEL_TIMING,          // CSBKO=7 (CT_CSBK), no fields
    NACK_RESPONSE,           // CSBKO=38/0x26 (NACK_Rsp)
    ANNOUNCEMENT,            // CSBKO=40/0x28 (C_BCAST)
    BS_OUTBOUND_ACTIVATION,  // CSBKO=56/0x38 (BS_Dwn_Act) -- normally MS->BS wake-up
    MOVE,                    // CSBKO=57/0x39 (C_MOVE), no fields
    PREAMBLE,                // CSBKO=61/0x3D
};

struct StandardCsbk {
    bool recognized{false};
    StandardCsbkType type{StandardCsbkType::NONE};

    // NACK_RESPONSE, BS_OUTBOUND_ACTIVATION, PREAMBLE only.
    uint32_t target_address{0};
    uint32_t source_address{0};

    // ANNOUNCEMENT only -- the sub-type classification (0-7 named in the
    // spec, e.g. 6="Adjacent Site Information"; per-type payload fields
    // beyond this classification are NOT decoded, unlike this project's
    // other CSBK opcodes -- deferred as lower value than getting more
    // opcodes recognized at all, see the class comment).
    uint8_t announcement_type{0};

    // PREAMBLE only.
    bool preamble_is_group{false};  // GI bit: true=group, false=individual
    bool preamble_is_data{false};   // "content" bit: true=Data follows, false=CSBK follows
    uint8_t preamble_blocks{0};     // blocks to follow
};

StandardCsbk csbk_decode_standard(const CsbkHeader& header);

}  // namespace dmr

#endif /*__DMR_CSBK_H__*/
