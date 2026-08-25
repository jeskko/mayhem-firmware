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

#include "proc_dmr.hpp"
#include "portapack_shared_memory.hpp"
#include "event_m4.hpp"

#include <algorithm>
#include <cmath>

DmrProcessor::DmrProcessor() {
    decim_0.configure(taps_12k5_decim_0.taps);
    decim_1.configure(taps_12k5_decim_1.taps);
    demod_.configure(channel_fs, nominal_deviation_hz);

    baseband_thread.start();
    configured = true;
}

void DmrProcessor::execute(const buffer_c8_t& buffer) {
    if (!configured) return;

    const auto decim_0_out = decim_0.execute(buffer, dst_buffer_0);
    const auto channel_out = decim_1.execute(decim_0_out, dst_buffer_1);

    feed_channel_stats(channel_out);

    // Carrier-present gate: tracks a decaying envelope of |I|+|Q| at the
    // channel-filter output. On a falling edge -- the inter-slot gap of
    // a Tier I DMO/simplex burst, where the transmitting radio is
    // completely silent -- discard any in-flight sync/burst state so it
    // doesn't get stitched together with bits from after the gap. Tier
    // II (repeater) traffic keeps its carrier up continuously across
    // both slots, so this gate simply never fires there.
    for (size_t i = 0; i < channel_out.count; i++) {
        const auto& s = channel_out.p[i];
        const float mag = std::abs(static_cast<float>(s.real())) + std::abs(static_cast<float>(s.imag()));
        carrier_envelope_ += (mag - carrier_envelope_) * carrier_alpha;

        const bool now_present = carrier_envelope_ > carrier_squelch_threshold;
        if (carrier_present_ && !now_present) {
            symbol_processor_.reset_sync();
        }
        carrier_present_ = now_present;
    }

    const auto disc_out = demod_.execute(channel_out, disc_buffer_);

    for (size_t i = 0; i < disc_out.count; i++) {
        clock_recovery_(matched_filter_(disc_out.p[i]));
    }
}

void DmrProcessor::maybe_report_debug() {
    // ~1/s at 4800 sym/s. Independent of whether any burst is ever
    // found -- lets the UI show live signal/calibration stats even
    // when nothing ever syncs, which a burst-triggered message alone
    // can't do.
    if (++debug_report_counter_ >= 4800) {
        debug_report_counter_ = 0;
        shared_memory.application_queue.push(
            DmrDebugMessage(
                symbol_processor_.avg_abs_symbol(),
                symbol_processor_.outer_amp_est(),
                carrier_present_,
                symbol_processor_.last_best_sync_err(),
                static_cast<uint32_t>(symbol_processor_.symbol_count()),
                symbol_processor_.dc_estimate()));
    }
}

void DmrProcessor::push_burst_to_ui(const DmrSymbolProcessor::Burst& burst) {
    shared_memory.application_queue.push(
        DmrBurstMessage(
            burst.bits.data(),
            /* inverted */ false,  // no longer a meaningful concept -- see dmr_symbol_processor.hpp
            static_cast<uint8_t>(burst.sync_type),
            burst.errors,
            burst.has_cach,
            burst.cach_bits.data(),
            burst.sync_mag_ratio));

    // Re-anchor embedded-signalling window tracking on EVERY confirmed
    // voice-sync burst, not just a just-recognized Voice LC Header --
    // ETSI clause 7.1.3 has a call's LC content repeating every
    // superframe for the call's whole duration, and each superframe's
    // own frame A gives a fresh, precise anchor for that superframe's
    // B-F windows. This decision only needs sync_type (available here,
    // on M4, straight off sync correlation) -- unlike deciding whether
    // to RESET the M0-side LCSS/BPTC assembly state on a genuine new
    // call, which needs Golay-decoded data_type and so happens on M0
    // instead (see DmrRxView::on_data_dmr()) -- no M0->M4 round-trip
    // needed for arming itself. See arm_embedded_signalling()'s own
    // comment for the full derivation.
    if (burst.sync_type == DmrSymbolProcessor::SyncType::BS_VOICE ||
        burst.sync_type == DmrSymbolProcessor::SyncType::MS_VOICE) {
        symbol_processor_.arm_embedded_signalling(burst.burst_start);
    }
}

void DmrProcessor::push_embedded_signalling_to_ui(const DmrSymbolProcessor::EmbeddedSignallingWindow& window) {
    shared_memory.application_queue.push(
        DmrEmbeddedSignallingMessage(window.bits48.data()));
}

void DmrProcessor::on_message(const Message* const msg) {
    (void)msg;
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<DmrProcessor>()};
    event_dispatcher.run();
    return 0;
}
