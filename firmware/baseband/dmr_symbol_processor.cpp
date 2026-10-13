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

#include "dmr_symbol_processor.hpp"

#include <cmath>

void DmrSymbolProcessor::reset_sync() {
    // The burst grid anchor and the embedded schedules are deliberately
    // kept: bit_count_ keeps running through a carrier gap, and inbound/
    // simplex traffic gaps the carrier between every burst.
    pending_burst_.valid = false;
    sign_register_ = 0;
}

uint8_t DmrSymbolProcessor::slot_phase(uint64_t burst_start, uint8_t errors, bool& new_grid, bool& on_grid) {
    // Phase is counted in 288-bit steps from the previous sync burst, not
    // from an absolute origin, so slow symbol-clock drift can't flip it.
    // A burst far off that grid (or after a long silence) is a new
    // transmission with its own timing and starts a new grid -- as in
    // datarx dmr.py _phase().
    new_grid = false;
    on_grid = true;
    if (has_anchor_ && burst_start >= anchor_start_) {
        const uint64_t d = burst_start - anchor_start_;
        const uint64_t k = (d + PHYSICAL_BURST_PERIOD_BITS / 2) / PHYSICAL_BURST_PERIOD_BITS;
        const uint64_t grid_start = k * PHYSICAL_BURST_PERIOD_BITS;
        const uint64_t off = (d > grid_start) ? d - grid_start : grid_start - d;
        const uint8_t phase = static_cast<uint8_t>((anchor_phase_ + k) & 1);
        if (d <= GRID_MAX_GAP_BITS) {
            if (off <= GRID_TOLERANCE_BITS) {
                anchor_phase_ = phase;
                anchor_start_ = burst_start;
                return phase;
            }
            if (errors > 1) {
                // Marginal sync off a live grid: almost certainly a false
                // match inside a burst. Leave the grid alone.
                on_grid = false;
                return phase;
            }
        }
    }
    new_grid = has_anchor_;
    has_anchor_ = true;
    anchor_start_ = burst_start;
    anchor_phase_ = 0;
    return anchor_phase_;
}

void DmrSymbolProcessor::update_embedded_schedule(SyncType sync_type, uint8_t phase, uint64_t burst_start) {
    auto& armed = armed_embedded_signalling_[phase];
    if (sync_type == SyncType::BS_VOICE || sync_type == SyncType::MS_VOICE) {
        // Voice burst A: B-F follow at 576-bit steps in this slot
        // (ETSI TS 102 361-1 clause 7.1.3: the LC starts on the first
        // non-SYNC, non-RC burst of the superframe).
        armed.active = true;
        armed.voice_a_start = burst_start;
        armed.bs_sourced = sync_type == SyncType::BS_VOICE;
        armed.next_index = 1;
    } else {
        // Data/RC sync: this slot is no longer carrying a voice superframe.
        armed.active = false;
    }
}

void DmrSymbolProcessor::emit_embedded_windows() {
    for (uint8_t phase = 0; phase < 2; phase++) {
        auto& armed = armed_embedded_signalling_[phase];
        if (!armed.active) continue;

        const uint64_t voice_burst_start = armed.voice_a_start + armed.next_index * SLOT_BURST_PERIOD_BITS;
        const uint64_t window_start = voice_burst_start + SYNC_OFFSET;
        if (bit_count_ < window_start + SYNC_BITS) continue;

        EmbeddedSignallingWindow window{};
        window.slot_phase = phase;
        for (size_t i = 0; i < SYNC_BITS; i++) {
            const uint8_t b = history_bit(window_start + i);
            window.bits48[i >> 3] |= static_cast<uint8_t>(b << (7 - (i & 7)));
        }
        if (armed.bs_sourced) {
            window.has_cach = true;
            const uint64_t cach_start = voice_burst_start - CACH_BITS;
            for (size_t i = 0; i < CACH_BITS; i++) {
                const uint8_t b = history_bit(cach_start + i);
                window.cach_bits[i >> 3] |= static_cast<uint8_t>(b << (7 - (i & 7)));
            }
        }
        if (on_embedded_signalling_) on_embedded_signalling_(window);

        // Skip position 6: that is the next superframe's voice burst A,
        // which carries a sync word and re-arms this schedule if detected.
        armed.next_index++;
        if (armed.next_index == 6) armed.next_index++;
        if (armed.next_index > EMBEDDED_LAST_INDEX) armed.active = false;
    }
}

void DmrSymbolProcessor::process_symbol(const float raw_symbol) {
    // AFC: remove the CURRENT dc_estimate_ before doing anything else.
    // The estimate itself is updated further down, and ONLY on a
    // confirmed-quality sync (see there for why) -- not here,
    // unconditionally, on every symbol regardless of context.
    const float symbol = raw_symbol - dc_estimate_;

    amplitude_history_[symbol_count_ % amplitude_history_.size()] = symbol;
    symbol_count_++;
    avg_abs_symbol_ += (std::abs(symbol) - avg_abs_symbol_) * (1.0f / 64.0f);

    const uint32_t sign_bit = (symbol < 0.0f) ? 1u : 0u;
    sign_register_ = ((sign_register_ << 1) | sign_bit) & SIGN_MASK24;

    const float threshold = SLICE_THRESHOLD_RATIO * outer_amp_est_;
    uint8_t level;  // 0 = -3, 1 = -1, 2 = +1, 3 = +3
    if (symbol >= 0.0f) {
        level = (symbol > threshold) ? 3 : 2;
    } else {
        level = (symbol < -threshold) ? 0 : 1;
    }

    static constexpr uint8_t dibit_table[4] = {0b11, 0b10, 0b00, 0b01};
    const uint8_t dibit = dibit_table[level];

    for (int b = 1; b >= 0; b--) {
        const uint8_t bit_val = (dibit >> b) & 0x01;

        const size_t byte_idx = (history_write_idx_ / 8) % 128;
        if ((history_write_idx_ % 8) == 0) bit_history_buffer_[byte_idx] = 0;

        bit_history_buffer_[byte_idx] |= (bit_val << (7 - (history_write_idx_ % 8)));
        history_write_idx_ = (history_write_idx_ + 1) % 1024;
        bit_count_++;
    }

    if (!pending_burst_.valid && symbol_count_ >= SYNC_SYMBOLS) {
        uint32_t best_err = 25;
        size_t best_w = 0;
        for (size_t w = 0; w < SYNC_SIGN24.size(); w++) {
            const uint32_t err = __builtin_popcount(sign_register_ ^ SYNC_SIGN24[w]);
            if (err < best_err) {
                best_err = err;
                best_w = w;
            }
        }
        last_best_sync_err_ = static_cast<uint8_t>(best_err);

        if (best_err <= SYNC_ERROR_THRESHOLD) {
            // Magnitude-consistency gate -- see MAG_RATIO_REJECT_
            // THRESHOLD's comment. Computed for every candidate that
            // passes the sign-pattern check, BEFORE the amplitude/AFC
            // recalibration and burst-creation below, so an outright
            // implausible candidate (huge min/max spread -- "FM click
            // noise", not a real 4FSK signal) never reaches either.
            float mag_min = 1e30f, mag_max = 0.0f;
            for (uint32_t k = 0; k < SYNC_SYMBOLS; k++) {
                const uint64_t idx = (symbol_count_ - SYNC_SYMBOLS + k) % amplitude_history_.size();
                const float mag = std::abs(amplitude_history_[idx]);
                if (mag < mag_min) mag_min = mag;
                if (mag > mag_max) mag_max = mag;
            }
            const float mag_ratio = (mag_min > 1.0f) ? (mag_max / mag_min) : 1.0e6f;

            if (mag_ratio <= MAG_RATIO_REJECT_THRESHOLD) {
                // Only let a NEAR-PERFECT match (<=1 bit, stricter than
                // the <=2 accepted for reporting a burst at all) feed
                // the amplitude recalibration and AFC below. A marginal
                // match is more likely to be a noise-driven false
                // positive than a real sync -- and FM discriminators
                // famously produce LARGER, more chaotic excursions on
                // pure noise than on a real signal ("FM click noise"),
                // so letting a marginal match through here risks
                // calibrating the slicing threshold (and frequency
                // estimate) to noise, corrupting payload extraction even
                // on bursts whose sync WAS genuine.
                if (best_err <= 1) {
                    float sum_abs = 0.0f;
                    float sum_signed = 0.0f;
                    for (uint32_t k = 0; k < SYNC_SYMBOLS; k++) {
                        const uint64_t idx = (symbol_count_ - SYNC_SYMBOLS + k) % amplitude_history_.size();
                        sum_abs += std::abs(amplitude_history_[idx]);
                        sum_signed += amplitude_history_[idx];
                    }
                    const float measured_outer = sum_abs / static_cast<float>(SYNC_SYMBOLS);
                    outer_amp_est_ = 0.5f * outer_amp_est_ + 0.5f * measured_outer;

                    // AFC, anchored to this SAME confirmed-good sync
                    // window -- ground truth, exactly like
                    // outer_amp_est_ above, instead of a blind
                    // continuous per-symbol average. A continuous
                    // unconditional tracker drifts toward pure-noise
                    // statistics during the long silent/noisy stretches
                    // between a repeater's periodic beacon (minority
                    // real signal, majority noise, by design in that
                    // traffic pattern), corrupting slicing once real
                    // signal returns -- gating on sync quality means
                    // noise (which sits at ~12/24, chance level) can't
                    // reach this at all, only content that's already
                    // very close to a real sync word.
                    //
                    // Sync words use ONLY outer-level dibits (see
                    // SYNC_SIGN24's derivation), so each of the 24
                    // symbols' EXPECTED value -- if dc_estimate_ were
                    // already exactly right -- is precisely
                    // +-outer_amp_est_ per SYNC_SIGN24's own sign
                    // pattern. Any gap between that and what was
                    // actually measured (already dc_estimate_-corrected)
                    // is residual bias still to remove. DC_ANCHOR_BLEND
                    // is a per-EVENT blend (not per-symbol like the old
                    // design), since updates now fire only on
                    // confirmed-good syncs -- far rarer than every
                    // symbol, but each one far more trustworthy.
                    const uint32_t neg_count = static_cast<uint32_t>(__builtin_popcount(SYNC_SIGN24[best_w]));
                    const uint32_t pos_count = SYNC_SYMBOLS - neg_count;
                    const float expected_sum = outer_amp_est_ * (static_cast<float>(pos_count) - static_cast<float>(neg_count));
                    const float residual_bias = (sum_signed - expected_sum) / static_cast<float>(SYNC_SYMBOLS);
                    dc_estimate_ += residual_bias * DC_ANCHOR_BLEND;
                }

                const uint64_t sync_start_bit = bit_count_ - SYNC_BITS;
                if (sync_start_bit >= SYNC_OFFSET) {
                    pending_burst_.valid = true;
                    pending_burst_.burst_start = sync_start_bit - SYNC_OFFSET;
                    pending_burst_.ready_at = pending_burst_.burst_start + BURST_BITS;
                    pending_burst_.sync_type = static_cast<SyncType>(best_w);
                    pending_burst_.errors = static_cast<uint8_t>(best_err);
                    pending_burst_.mag_ratio = mag_ratio;
                }
            }  // mag_ratio <= MAG_RATIO_REJECT_THRESHOLD
        }
    }

    if (pending_burst_.valid && bit_count_ >= pending_burst_.ready_at)
        push_burst();

    // Embedded-signalling windows sit where voice bursts B-F would have
    // a sync word, so they never conflict with the sync search above.
    emit_embedded_windows();
}

uint8_t DmrSymbolProcessor::history_bit(uint64_t absolute_bit) const {
    const size_t p = absolute_bit % 1024;

    return (bit_history_buffer_[p >> 3] >>
            (7 - (p & 7))) &
           1;
}

void DmrSymbolProcessor::push_burst() {
    Burst burst{};

    for (size_t i = 0; i < BURST_BITS; i++) {
        const uint8_t b = history_bit(pending_burst_.burst_start + i);
        burst.bits[i >> 3] |= b << (7 - (i & 7));
    }
    burst.sync_type = pending_burst_.sync_type;
    burst.errors = pending_burst_.errors;
    burst.sync_mag_ratio = pending_burst_.mag_ratio;
    burst.burst_start = pending_burst_.burst_start;
    bool on_grid;
    burst.slot_phase = slot_phase(pending_burst_.burst_start, burst.errors, burst.new_grid, on_grid);

    const bool bs_sourced = burst.sync_type == SyncType::BS_VOICE ||
                            burst.sync_type == SyncType::BS_DATA;
    if (bs_sourced && pending_burst_.burst_start >= CACH_BITS) {
        burst.has_cach = true;
        const uint64_t cach_start = pending_burst_.burst_start - CACH_BITS;
        for (size_t i = 0; i < CACH_BITS; i++) {
            const uint8_t b = history_bit(cach_start + i);
            burst.cach_bits[i >> 3] |= b << (7 - (i & 7));
        }
    }

    if (on_grid) update_embedded_schedule(burst.sync_type, burst.slot_phase, burst.burst_start);

    if (on_burst_) on_burst_(burst);

    pending_burst_.valid = false;
}
