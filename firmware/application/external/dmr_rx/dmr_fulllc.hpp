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
 * Full Link Control (ETSI TS 102 361-1 Table 9.7): the 96-bit PDU
 * carried (via BPTC(196,96), see dmr_bptc19696.hpp) in a burst whose
 * Slot Type Data Type is VOICE_LC_HEADER or TERMINATOR_WITH_LC.
 * Byte layout: PF(1b)+Reserved(1b)+FLCO(6b), FID(8b), then 56 bits of
 * FLCO-specific Data, then a 24-bit RS(12,9) parity field.
 *
 * For FLCO=Group Voice Channel User / Private Voice Channel User (the
 * only two parsed here -- everything else, e.g. Talker Alias blocks or
 * GPS, is out of scope for now), the Data field is Service Options(8b)
 * + Destination Address(24b) + Source Address(24b) -- i.e. exactly
 * talkgroup/dest ID and talker/source radio ID. Field layout confirmed
 * against dsd-fme's own independently-decoded TGT/SRC output for a
 * Private call, matching byte-for-byte against the raw BPTC-decoded
 * PDU bytes.
 *
 * The RS(12,9) parity check (see dmr_rs129.hpp) IS now applied, over
 * the whole 12-byte PDU (so it can fix/reject a corrupted FLCO/FID too,
 * not just the address fields) -- a repeat-match confidence check alone
 * (SlotLcState's "2 consecutive identical decodes" in ui_dmr_rx.cpp) is
 * not sufficient on its own: real multipath/fading correlates bit
 * errors WITHIN a transmission rather than making them independent per
 * burst, so the same single/double-bit corruption of an address field
 * can survive several consecutive bursts unchanged, passing that
 * heuristic despite being wrong. RS(12,9) can correct up to 1 corrupted
 * byte outright (3 parity bytes) and rejects (recognized=false)
 * anything worse -- the repeat-match check in ui_dmr_rx.cpp is kept as
 * a second, independent layer on top, not replaced by this.
 */

#ifndef __DMR_FULLLC_H__
#define __DMR_FULLLC_H__

#include <cstdint>

namespace dmr {

enum class Flco : uint8_t {
    GROUP_VOICE = 0,
    PRIVATE_VOICE = 3,
};

struct FullLcResult {
    bool recognized{false};  // true iff RS(12,9) validated/corrected AND flco is one of the Flco values above
    uint8_t flco{0};
    uint8_t fid{0};
    uint8_t service_options{0};
    uint32_t dest_address{0};    // talkgroup ID (group call) or dest radio ID (private call)
    uint32_t source_address{0};  // talker/source radio ID
};

// pdu96: the output of bptc19696_decode() for a burst whose Data Type
// is VOICE_LC_HEADER or TERMINATOR_WITH_LC. is_terminator selects which
// of the two ETSI-defined RS(12,9) masks to unmask with (see
// dmr_rs129.hpp) -- pass true for TERMINATOR_WITH_LC, false for
// VOICE_LC_HEADER; using the wrong one will reliably fail the check
// rather than silently validating, by design.
FullLcResult fulllc_decode(const uint8_t pdu96[96], bool is_terminator);

// Service Options byte (ETSI TS 102 361-1 Table 7.2), bit-for-bit
// confirmed against dsd-fme's src/dmr_flco.c (the `so & 0x80` etc.
// chain): Emergency(0x80), Privacy(0x40), Broadcast Service(0x08),
// OVCM/Open Voice Call Mode(0x04), Priority(0x03, 0=none/1-3=level).
// Bits 0x20/0x10 (TXI / "repeat backwards channel") are Motorola FID
// 0x10-specific per the same reference -- reported as false on any
// other FID rather than potentially misreporting a reserved/vendor-
// specific bit as those flags.
struct ServiceOptions {
    bool emergency{false};
    bool privacy{false};
    bool broadcast{false};
    bool ovcm{false};
    uint8_t priority{0};   // 0 = none, 1-3 = priority level
    bool moto_txi{false};  // Motorola FID 0x10 only
    bool moto_rpt{false};  // Motorola FID 0x10 only
};

ServiceOptions decode_service_options(uint8_t service_options_byte, uint8_t fid);

}  // namespace dmr

#endif /*__DMR_FULLLC_H__*/
