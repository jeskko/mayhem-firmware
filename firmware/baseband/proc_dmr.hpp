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
 * DMR (ETSI TS 102 361-1) burst-sync / network-parameter baseband
 * front end. This is intentionally scoped like proc_tetra.cpp: the M4
 * core here only decimates, demodulates, recovers symbol timing, and
 * hands raw synchronized burst bits up to the application core. All
 * FEC (Golay/Hamming/BPTC), CRC checks, and PDU parsing happen on M0
 * (see external/dmr_rx/). No voice (AMBE) decode is attempted anywhere.
 *
 * The actual burst-sync/4-level-slicing logic lives in
 * dmr_symbol_processor.hpp/.cpp, split out specifically so it has no
 * hardware dependency and can be linked into a native (x86) test
 * harness (tools/dmr_native_test/) that feeds it symbols derived from
 * captured IQ files -- letting that logic be checked against real
 * captures in seconds instead of a rebuild-flash-listen cycle on real
 * hardware. This file is just the hardware plumbing: decimation, FM
 * discrimination, the Gardner clock-recovery wiring, the carrier-
 * present gate, and pushing a completed burst up to the application
 * core as a DmrBurstMessage.
 *
 * DMR is 4-level FSK (4FSK/C4FM-class) at 4800 symbols/s (9600 bit/s),
 * NOT phase-differential like TETRA's pi/4-DQPSK -- so unlike
 * proc_tetra.cpp, the demod core here is a frequency discriminator
 * (dsp::demodulate::FM) followed by a 4-level amplitude slicer, the
 * same structural idea as proc_flex.cpp's FLEX 4-level slicer
 * (flex_sym()/read_data()), not a dot-product phase detector.
 *
 * Burst geometry (Tier II repeater and Tier I DMO simplex both use the
 * same 264-bit/27.5ms burst format; ETSI TS 102 361-1):
 *   Info1(98) + SlotType1(10) + Sync(48) + SlotType2(10) + Info2(98)
 * i.e. the 48-bit sync field sits 108 bits into the burst, with 108
 * bits following it. Five fixed 48-bit sync words distinguish
 * BS/MS-sourced Voice/Data bursts and the reverse channel (RC), cross-
 * checked against four independent open-source DMR decoders during
 * development (MMDVMHost, dmrlib, go-dmr, ok-dmrlib) and additionally
 * confirmed against dsd-fme by direct decode of a captured IQ file:
 *   BS Voice: 0x755FD7DF75F7   BS Data: 0xDFF57D75DF5D
 *   MS Voice: 0x7F7D5DD57DFD   MS Data: 0xD5D7F77FD757
 *   RC:       0x77D55F7DFD77
 * (DmrSymbolProcessor::SYNC_SIGN24 holds the sign-only reduction of
 * these actually used for detection -- see that header for why.)
 *
 * IMPORTANT DMO/simplex note: unlike a Tier II repeater (which keeps
 * its carrier up continuously, alternating slots), a Tier I DMO/
 * simplex handset only ever transmits one slot and goes silent (no
 * carrier at all) during the other slot's 30ms window, with an RF
 * ramp transient at each burst edge. A carrier-present gate
 * (see carrier_envelope_/carrier_present_ below) resets in-flight
 * sync/burst state on the falling edge so stale bits from before a
 * gap don't get correlated together with bits from after it. This is
 * a deliberate deviation from proc_tetra.cpp's free-running model,
 * which assumes a continuous stream.
 */

#ifndef __PROC_DMR_H__
#define __PROC_DMR_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "message.hpp"
#include "dsp_decimate.hpp"
#include "dsp_demodulate.hpp"
#include "dsp_fir_taps.hpp"
#include "rssi_thread.hpp"
#include "clock_recovery.hpp"
#include "dmr_matched_filter.hpp"
#include "dmr_symbol_processor.hpp"

#include <cstdint>
#include <array>

class DmrProcessor : public BasebandProcessor {
   public:
    DmrProcessor();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const msg);

   private:
    static constexpr size_t baseband_fs = 3072000;
    static constexpr size_t channel_fs = 48000;
    static constexpr float symbol_rate = 4800.0f;

    // dsp::demodulate::FM::configure(fs, deviation_hz) sets its output
    // scale to +-1.0 at +-deviation_hz. Passing 1.0f here instead of a
    // "real" deviation figure makes the output plain radians-per-sample
    // converted to Hz (kf = fs/(2*pi)) -- i.e. RAW Hz, matching exactly
    // the scale used to validate this module's demod logic offline
    // against real captures. Using DMR's nominal +-1944Hz deviation
    // here instead would silently rescale the discriminator into a
    // +-1-ish-unit range while DmrSymbolProcessor's outer_amp_est_
    // prior (2000.0f) still assumed a Hz-scale value -- harmless to
    // sign-only sync detection (sign is scale-invariant) but badly
    // wrong for the amplitude-threshold payload slicing until many
    // sync hits slowly dragged the running estimate down by half each
    // time. This was a real, confirmed bug in an earlier version.
    static constexpr float nominal_deviation_hz = 1.0f;

    std::array<complex16_t, 512> dst_0{};
    const buffer_c16_t dst_buffer_0{dst_0.data(), dst_0.size()};

    std::array<complex16_t, 512> dst_1{};
    const buffer_c16_t dst_buffer_1{dst_1.data(), dst_1.size()};

    std::array<float, 512> disc_{};
    const buffer_f32_t disc_buffer_{disc_.data(), disc_.size()};

    dsp::decimate::FIRC8xR16x24FS4Decim8 decim_0{};
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};
    dsp::demodulate::FM demod_{};

    // Post-discriminator matched/shaping filter -- see dmr_matched_filter.hpp
    // for why this stage exists and the A/B evidence it helps.
    DmrMatchedFilter matched_filter_{};

    // --- Carrier-present gate (see DMO/simplex note above) ---
    float carrier_envelope_{0.0f};
    bool carrier_present_{false};
    static constexpr float carrier_alpha = 1.0f / 32.0f;
    // Originally an untested guess (40.0f) -- confirmed wrong against
    // two real captures of actual repeater traffic (continuous
    // carrier, so this gate should essentially never fire): measured
    // channel |I|+|Q| had a MEAN of only ~29-38 there, i.e. sitting
    // right at/below the old threshold, which meant the gate could
    // flicker present/absent continuously during genuinely continuous
    // reception -- repeatedly wiping sync state via on_carrier_lost()
    // before a sync or burst could ever be captured. Still an
    // estimate (no true-silence capture to calibrate the low end
    // against yet), but now grounded in real measured signal levels
    // rather than a blind guess.
    static constexpr float carrier_squelch_threshold = 8.0f;

    bool configured{false};
    uint32_t debug_report_counter_{0};

    void push_burst_to_ui(const DmrSymbolProcessor::Burst& burst);
    void push_embedded_signalling_to_ui(const DmrSymbolProcessor::EmbeddedSignallingWindow& window);
    void maybe_report_debug();

    // The actual burst-sync/slicing logic -- see dmr_symbol_processor.hpp.
    DmrSymbolProcessor symbol_processor_{
        [this](const DmrSymbolProcessor::Burst& burst) {
            this->push_burst_to_ui(burst);
        },
        [this](const DmrSymbolProcessor::EmbeddedSignallingWindow& window) {
            this->push_embedded_signalling_to_ui(window);
        }};

    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive, false};
    RSSIThread rssi_thread{};

    /* Captures `this` in its symbol_handler lambda, but only calls into
     * it once execute() runs -- and baseband_thread above is
     * constructed with auto_start=false and only .start()-ed at the
     * end of DmrProcessor(), by which point every member (including
     * this one) is fully constructed regardless of declaration order. */
    clock_recovery::ClockRecovery<clock_recovery::FixedErrorFilter> clock_recovery_{
        static_cast<float>(channel_fs),
        symbol_rate,
        {0.0555f},
        [this](const float raw_symbol) {
            this->symbol_processor_.process_symbol(raw_symbol);
            this->maybe_report_debug();
        }};
};

#endif /*__PROC_DMR_H__*/
