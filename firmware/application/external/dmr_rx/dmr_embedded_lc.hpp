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
 * Embedded Signalling LC (ETSI TS 102 361-1 Table 9.6 family + TS 102
 * 361-2 7.2.19): the 72-bit payload recovered by BPTC(128,77) (see
 * dmr_bptc12877.hpp) from voice frames B-F's embedded-signalling field.
 * Same PF+Reserved+FLCO(8)+FID(8) header shape as Full LC (see
 * dmr_fulllc.hpp) but with a 56-bit Data field (no RS(12,9) -- this
 * channel's own integrity check is the simple mod-31 checksum below)
 * and a different set of FLCO values:
 *   0x00/0x03 Group/Unit-to-Unit Voice -- same fields as Full LC,
 *     redundantly retransmitted every superframe. NOT decoded here yet
 *     (recognized but no field extraction) -- lower priority than
 *     Talker Alias since Full LC already gives this info at call start;
 *     the main value would be "late entry" (joining mid-call), not
 *     implemented.
 *   0x04 Talker Alias Header, 0x05-0x07 Talker Alias Blocks 1-3 --
 *     assembled into a display string below. Cross-checked against
 *     dsd-fme's own independent decode of the same content.
 *   0x08 GPS Info -- NOT decoded here yet (recognized but no field
 *     extraction), a candidate follow-up once this dispatch shell
 *     exists.
 *
 * Checksum (ETSI TS 102 361-1 Annex, dsd-fme's ComputeCrc5Bit): despite
 * the name some references give it, this is NOT a polynomial CRC --
 * it's sum-of-9-bytes(the 72 payload bits packed MSB-first) mod 31,
 * compared against the 5 checksum bits BPTC(128,77) extracts alongside
 * the payload.
 */

#ifndef __DMR_EMBEDDED_LC_H__
#define __DMR_EMBEDDED_LC_H__

#include <array>
#include <cstdint>
#include <string>

namespace dmr {

// data72: the 72-bit Embedded LC payload from bptc12877_decode().
// checksum5: the 5 checksum bits from the same call.
bool embedded_lc_checksum_ok(const uint8_t data72[72], const uint8_t checksum5[5]);

struct TalkerAliasResult {
    bool updated{false};   // true if this fragment changed the assembled text
    bool complete{false};  // true once the declared character count is assembled
    std::string text;
};

// Assembles a Talker Alias across a header (FLCO 0x04) and up to 3
// continuation blocks (FLCO 0x05-0x07, block_num 0-2) for ONE slot.
// Fragments can arrive in any order after the header (a block before
// its header is buffered raw and reinterpreted once the header's
// format/length is known) -- mirrors dsd-fme's own tolerance for a
// missing/late header on some real radios (see dsd_alias.c's own
// comments about a sample missing its header on one block).
class TalkerAliasAssembler {
   public:
    // data56: Data field bits (56 bits) of one embedded LC fragment,
    // i.e. bits 16-71 of the 72-bit payload (after PF+Reserved+FLCO+FID).
    bool add_header(const uint8_t data56[56], TalkerAliasResult& out);
    bool add_block(uint8_t block_num, const uint8_t data56[56], TalkerAliasResult& out);
    void reset();

   private:
    void rebuild_text(TalkerAliasResult& out);

    bool have_header_{false};
    uint8_t format_{0};        // 0=7-bit, 1/2=8-bit, 3=16-bit (see dsd-fme's own mapping)
    uint8_t char_size_{0};     // 7, 8, or 16
    uint8_t declared_len_{0};  // 0 = unknown/use whatever's assembled so far

    // Raw character-data bits, concatenated header-tail + blocks 0-2 in
    // order, up to 49 (7-bit header tail) + 3*56 = 217 bits.
    std::array<uint8_t, 217> bits_{};
    size_t bits_len_{0};
    bool block_received_[3]{false, false, false};
    // Raw per-block bits, kept alongside `bits_` regardless of arrival
    // order -- needed both to buffer a block that arrives before the
    // header (replayed once the header's char-size fixes their real
    // position) and to re-derive that position if a later block arrives
    // after the header already advanced bits_len_ past a gap.
    std::array<std::array<uint8_t, 56>, 3> pending_block_bits_{};
};

// Assembles the LCSS-driven 4-fragment BPTC(128,77) sequence from raw
// 48-bit Embedded Signalling windows (see
// DmrSymbolProcessor::EmbeddedSignallingWindow / DmrEmbeddedSignallingMessage),
// decodes the resulting 72-bit Embedded LC payload once complete, and
// dispatches Talker Alias fragments to an internal TalkerAliasAssembler.
// One instance tracks ONE slot's in-progress sequence.
//
// This is the shared implementation used by both this project's
// offline capture-validation tooling and the real UI (ui_dmr_rx.cpp) --
// extracted here specifically so the two never drift apart. An earlier
// fixed-schedule version had bugs that made clear how easy this
// bit-level logic is to get subtly wrong (see
// DmrSymbolProcessor::arm_embedded_signalling()'s comment for details).
//
// Mirrors OpenGD77's DMREmbeddedData_addData() state machine: LCSS=01
// starts a sequence, LCSS=11 continues it (twice), LCSS=10 completes
// it (ETSI table 9.20) -- forward chronological order, NOT the reverse
// order an earlier version of this code incorrectly assumed.
class EmbeddedSignallingAssembler {
   public:
    struct Result {
        uint8_t lcss{0};  // always populated -- the QR(16,7,6)-decoded LCSS this window carried
        // 1/2/3 if this window's LCSS fit the current state and advanced
        // it into FIRST/SECOND/THIRD; 0 if ignored (didn't fit) or if
        // this window was the 4th/completing fragment (see
        // attempted_decode instead for that case). Diagnostic-oriented,
        // but harmless/cheap to always populate.
        int reached_state{0};
        bool attempted_decode{false};  // true iff the 4th fragment arrived and BPTC+checksum decode was attempted
        bool bptc_failed{false};       // valid only if attempted_decode
        bool checksum_ok{false};       // valid only if attempted_decode && !bptc_failed
        uint8_t flco{0};               // valid only if checksum_ok
        bool talker_alias_updated{false};
        TalkerAliasResult talker_alias;
    };

    // bits48: one raw window, MSB-first packed exactly as laid out on
    // air -- EMB(8) + embedded signalling data(32) + EMB(8) -- see
    // EmbeddedSignallingWindow's own comment for why the FEC/LCSS
    // decode deliberately does not happen any earlier than this.
    Result add_window(const uint8_t bits48[6]);

    // Clears the in-progress LCSS/BPTC assembly state only -- call on a
    // confirmed new call (Voice LC Header, Golay data_type==1) to avoid
    // completing a sequence using a stale fragment left over from
    // before. Optional robustness touch, not required for correctness:
    // a stale/misaligned assembly almost always fails the checksum
    // anyway and gets silently discarded like any other noise.
    // Deliberately does NOT reset the Talker Alias text itself (a
    // separate, higher-level concern) -- call talker_alias() and reset
    // that directly if a full new-call reset is wanted.
    void reset();

   private:
    enum class State { NONE,
                       FIRST,
                       SECOND,
                       THIRD } state_{State::NONE};
    std::array<uint8_t, 16> raw128_{};
    TalkerAliasAssembler talker_alias_assembler_;
};

}  // namespace dmr

#endif /*__DMR_EMBEDDED_LC_H__*/
