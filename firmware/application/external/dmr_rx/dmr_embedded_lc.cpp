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

#include "dmr_embedded_lc.hpp"

#include "dmr_bptc12877.hpp"
#include "dmr_qr1676.hpp"

#include <cstring>

namespace dmr {
namespace {

uint32_t bits_to_u32(const uint8_t* bits, int n) {
    uint32_t v = 0;
    for (int i = 0; i < n; i++) v = (v << 1) | (bits[i] & 1);
    return v;
}

// dsd-fme's own character-validity filter per char_size, replacing
// anything outside the printable range with a space rather than
// dropping it (keeps the assembled string's length/spacing meaningful
// even with a partially-corrupted alias).
char decode_char(uint32_t code, uint8_t char_size) {
    if (char_size == 7) {
        return (code >= 0x20 && code <= 0x7E) ? static_cast<char>(code) : ' ';
    }
    if (char_size == 8) {
        return (code >= 0x20 && code != 0x7F) ? static_cast<char>(code) : ' ';
    }
    // 16-bit: take the low byte if it lands in printable ASCII --
    // a deliberate simplification (not full UTF-16 handling) for a
    // format variant real traffic here hasn't been seen to use.
    const uint8_t lo = static_cast<uint8_t>(code & 0xFF);
    return (lo >= 0x20 && lo != 0x7F) ? static_cast<char>(lo) : ' ';
}

}  // namespace

bool embedded_lc_checksum_ok(const uint8_t data72[72], const uint8_t checksum5[5]) {
    uint32_t sum = 0;
    for (int i = 0; i < 9; i++) sum += bits_to_u32(&data72[i * 8], 8);
    const uint8_t computed = static_cast<uint8_t>(sum % 31);
    const uint8_t extracted = static_cast<uint8_t>(bits_to_u32(checksum5, 5));
    return computed == extracted;
}

bool TalkerAliasAssembler::add_header(const uint8_t data56[56], TalkerAliasResult& out) {
    format_ = static_cast<uint8_t>(bits_to_u32(&data56[0], 2));
    declared_len_ = static_cast<uint8_t>(bits_to_u32(&data56[2], 5));
    char_size_ = (format_ == 0) ? 7 : (format_ == 1 || format_ == 2) ? 8
                                                                     : 16;
    const size_t header_tail_start = (char_size_ == 7) ? 7 : 8;
    const size_t header_tail_len = 56 - header_tail_start;

    have_header_ = true;
    std::memcpy(bits_.data(), &data56[header_tail_start], header_tail_len);
    bits_len_ = header_tail_len;

    // Replay any blocks that arrived before this header.
    for (int b = 0; b < 3; b++) {
        if (!block_received_[b]) continue;
        const size_t pos = header_tail_len + static_cast<size_t>(b) * 56;
        if (pos + 56 <= bits_.size()) {
            std::memcpy(bits_.data() + pos, pending_block_bits_[b].data(), 56);
            if (pos + 56 > bits_len_) bits_len_ = pos + 56;
        }
    }

    rebuild_text(out);
    return true;
}

bool TalkerAliasAssembler::add_block(uint8_t block_num, const uint8_t data56[56], TalkerAliasResult& out) {
    if (block_num > 2) return false;

    if (!have_header_) {
        // Buffer raw -- repositioned once the header (with its format-
        // dependent tail length) arrives. See add_header()'s replay.
        std::memcpy(pending_block_bits_[block_num].data(), data56, 56);
        block_received_[block_num] = true;
        out.updated = false;
        return false;
    }

    const size_t header_tail_len = (char_size_ == 7) ? 49 : 48;
    const size_t pos = header_tail_len + static_cast<size_t>(block_num) * 56;
    block_received_[block_num] = true;
    std::memcpy(pending_block_bits_[block_num].data(), data56, 56);
    if (pos + 56 <= bits_.size()) {
        std::memcpy(bits_.data() + pos, data56, 56);
        if (pos + 56 > bits_len_) bits_len_ = pos + 56;
    }

    rebuild_text(out);
    return true;
}

void TalkerAliasAssembler::reset() {
    *this = TalkerAliasAssembler{};
}

void TalkerAliasAssembler::rebuild_text(TalkerAliasResult& out) {
    if (!have_header_ || char_size_ == 0) {
        out.updated = false;
        return;
    }

    const size_t total_chars = declared_len_ ? declared_len_ : (bits_len_ / char_size_);
    std::string text;
    text.reserve(total_chars);
    for (size_t i = 0; i < total_chars; i++) {
        const size_t bit_pos = i * char_size_;
        if (bit_pos + char_size_ > bits_len_) break;  // not enough data yet
        text.push_back(decode_char(bits_to_u32(&bits_[bit_pos], char_size_), char_size_));
    }

    out.updated = (text != out.text) || text.size() > 0;
    out.complete = text.size() == total_chars;
    out.text = text;
}

EmbeddedSignallingAssembler::Result EmbeddedSignallingAssembler::add_window(const uint8_t bits48[6]) {
    Result result{};

    // window bits48 is packed MSB-first: byte0 = bits[0:8) = EMB half 1
    // directly; byte5 = bits[40:48) = EMB half 2. See
    // EmbeddedSignallingWindow's comment.
    const uint8_t emb_byte0 = bits48[0];
    const uint8_t emb_byte1 = bits48[5];
    const uint8_t emb_info = qr1676_decode(emb_byte0, emb_byte1);  // CC(4)|PI(1)|LCSS(2)
    const uint8_t lcss = emb_info & 0x03U;
    result.lcss = lcss;

    // Data payload = bits [8:40) of the 48-bit field = bytes 1-4.
    std::array<uint8_t, 4> data32{};
    std::memcpy(data32.data(), bits48 + 1, 4);

    // LCSS-driven state machine, mirroring OpenGD77's
    // DMREmbeddedData_addData(): an LCSS value that doesn't fit the
    // current state is ignored, leaving state as-is.
    if (lcss == 0x01U) {
        std::copy(data32.begin(), data32.end(), raw128_.begin());
        state_ = State::FIRST;
        result.reached_state = 1;
        return result;
    } else if (lcss == 0x03U && state_ == State::FIRST) {
        std::copy(data32.begin(), data32.end(), raw128_.begin() + 4);
        state_ = State::SECOND;
        result.reached_state = 2;
        return result;
    } else if (lcss == 0x03U && state_ == State::SECOND) {
        std::copy(data32.begin(), data32.end(), raw128_.begin() + 8);
        state_ = State::THIRD;
        result.reached_state = 3;
        return result;
    } else if (lcss == 0x02U && state_ == State::THIRD) {
        std::copy(data32.begin(), data32.end(), raw128_.begin() + 12);
        state_ = State::NONE;
        // fall through: full 128-bit block assembled, decode below.
    } else {
        return result;  // doesn't fit current state -- ignore, as-is.
    }

    result.attempted_decode = true;

    uint8_t bits128[128];
    for (int i = 0; i < 128; i++)
        bits128[i] = (raw128_[static_cast<size_t>(i / 8)] >> (7 - (i % 8))) & 1;

    uint8_t data72[72], checksum5[5];
    if (!bptc12877_decode(bits128, data72, checksum5)) {
        result.bptc_failed = true;
        return result;
    }
    if (!embedded_lc_checksum_ok(data72, checksum5)) return result;

    result.checksum_ok = true;

    // data72 is one-bit-per-byte (matching bptc12877_decode's out72
    // convention) -- extract FLCO (bits 2-7, after PF+Reserved) by hand.
    uint8_t flco = 0;
    for (int i = 0; i < 6; i++) flco = static_cast<uint8_t>((flco << 1) | (data72[2 + i] & 1));
    result.flco = flco;

    uint8_t data56[56];
    for (int i = 0; i < 56; i++) data56[i] = data72[16 + i];

    if (flco == 0x04U) {
        result.talker_alias_updated = talker_alias_assembler_.add_header(data56, result.talker_alias);
    } else if (flco >= 0x05U && flco <= 0x07U) {
        result.talker_alias_updated = talker_alias_assembler_.add_block(
            static_cast<uint8_t>(flco - 0x05U), data56, result.talker_alias);
    }
    return result;
}

void EmbeddedSignallingAssembler::reset() {
    state_ = State::NONE;
}

}  // namespace dmr
