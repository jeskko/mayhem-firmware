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
 * The hardware-independent core of DMR burst-sync/slicing: everything
 * proc_dmr.cpp needs after it has a recovered-symbol float stream, with
 * no dependency on BasebandProcessor/shared_memory/ChibiOS. Split out
 * specifically so it can ALSO be linked into a native (x86, no ARM
 * toolchain) test harness that feeds it symbols derived from a captured
 * IQ file -- letting the exact production logic be iterated on and
 * checked against real captures in seconds, without a rebuild-flash-
 * SD-card-update-and-listen-on-real-hardware cycle for every change.
 * See tools/dmr_native_test/ for that harness.
 *
 * See proc_dmr.hpp for the protocol/algorithm background (sign-only
 * sync detection, sync-anchored amplitude recalibration, etc.) -- this
 * file is just that logic with the hardware plumbing removed.
 */

#ifndef __DMR_SYMBOL_PROCESSOR_H__
#define __DMR_SYMBOL_PROCESSOR_H__

#include <cstdint>
#include <array>
#include <functional>

class DmrSymbolProcessor {
   public:
    static constexpr uint32_t BURST_BITS = 264;
    static constexpr uint32_t BURST_SYMBOLS = BURST_BITS / 2;
    static constexpr uint32_t SYNC_BITS = 48;
    static constexpr uint32_t SYNC_SYMBOLS = SYNC_BITS / 2;
    static constexpr uint32_t SYNC_OFFSET = 108;
    static constexpr uint32_t SIGN_MASK24 = (1UL << 24) - 1;
    static constexpr uint32_t SYNC_ERROR_THRESHOLD = 2;  // out of 24 sign bits (~8%)
    static constexpr float SLICE_THRESHOLD_RATIO = 0.625f;

    // Magnitude-consistency gate: max(|symbol|)/min(|symbol|) across a
    // sync candidate's 24-symbol window. Real sync words use ONLY
    // outer-level dibits (see SYNC_SIGN24), so a genuine match's 24
    // symbols should all sit near the SAME magnitude; FM discriminators
    // produce much larger, more erratic excursions ("click noise") when
    // there's no real carrier locked. Validated offline against both a
    // totally-desynced capture (ratio
    // 19-177 on every false decode, vs 1.7-5.3 on genuine ones -- a
    // clean, dramatic separation) and realistic mixed-quality repeater
    // captures cross-checked against INDEPENDENT confirmation (CRC-
    // verified CSBK/Full LC, not just the Golay check this gate sits
    // upstream of): real channel noise/fading can occasionally push a
    // genuine burst's ratio quite high too, so this threshold is
    // deliberately conservative (only ~0.6-5% of independently-confirmed
    // real bursts exceeded 200 across the test captures) rather than
    // tuned to catch every noise-driven false positive -- see
    // Burst::sync_mag_ratio for making the actual observed values
    // visible in live use, to check whether tightening this further
    // stays safe.
    static constexpr float MAG_RATIO_REJECT_THRESHOLD = 200.0f;

    enum class SyncType : uint8_t {
        BS_VOICE = 0,
        BS_DATA = 1,
        MS_VOICE = 2,
        MS_DATA = 3,
        RC = 4,
        COUNT = 5
    };

    // Sign bit of each of a 48-bit sync word's 24 dibits, one entry per
    // SyncType value above -- see proc_dmr.hpp for full derivation notes.
    static constexpr std::array<uint32_t, 5> SYNC_SIGN24{{
        0x439B4DUL,  // BS sourced Voice
        0xBC64B2UL,  // BS sourced Data
        0x76286EUL,  // MS sourced Voice
        0x89D791UL,  // MS sourced Data
        0x5836E5UL,  // RC (reverse channel)
    }};

    static constexpr uint32_t CACH_BITS = 24;

    struct Burst {
        std::array<uint8_t, 33> bits;  // 264 bits, MSB-first packed
        SyncType sync_type;
        uint8_t errors;  // sign-pattern Hamming distance at match time, out of 24

        // max(|symbol|)/min(|symbol|) across this burst's own 24-symbol
        // sync window -- see MAG_RATIO_REJECT_THRESHOLD's comment. Only
        // ever <= that threshold (anything higher was already rejected
        // before a Burst could even be created), but the actual value is
        // exposed here so live use can show it (console/applog) and
        // help decide whether the threshold can be tightened further
        // without starting to reject genuine bursts.
        float sync_mag_ratio;

        // The CACH (Common Announcement Channel, Tier II only) field
        // immediately preceding this burst in the same continuous
        // bitstream -- ETSI TS 102 361-1 slot layout is CACH(24) +
        // Burst(264) = 288 bits/30ms, contiguous. Only BS-sourced bursts
        // (BS voice/data sync) have one: MS-sourced, RC and direct-mode
        // bursts have no CACH, so the 24 bits before them are not one.
        bool has_cach{false};
        std::array<uint8_t, 3> cach_bits{};  // 24 bits, MSB-first packed

        // Absolute bit position (this instance's own bit_count_ address
        // space) of this burst's own first bit.
        uint64_t burst_start{0};

        // Slot phase (0/1) of this burst on the 288-bit burst grid,
        // counted from the previous sync burst -- see slot_phase(). Not a
        // timeslot number: the UI maps phases to TS1/TS2 from CACH votes.
        uint8_t slot_phase{0};
        // True when this burst is off the previous grid (a new
        // transmission with its own timing), so phase<->slot votes
        // collected so far no longer apply.
        bool new_grid{false};
    };

    // One physical burst period's worth of RAW 48-bit sync-field content
    // from a voice frame that doesn't carry a real sync word (its
    // sync-word bit position is repurposed for embedded signalling),
    // delivered for voice bursts B-F of each armed slot -- see
    // update_embedded_schedule().
    // Deliberately NOT yet interpreted: bits48 is EMB(8) + embedded
    // signalling data(32) + EMB(8) exactly as laid out on air (ETSI
    // clause 6.1, figure 6.4), and the caller must QR(16,7,6)-decode the
    // two EMB halves to get LCSS before it even knows whether/where this
    // window belongs in an Embedded LC assembly. That FEC/parsing work
    // deliberately does NOT live in this class, to keep its M4-side
    // raw-bit-extraction role intact -- matching proc_dmr.cpp/
    // proc_tetra.cpp's "dumb baseband (M4), all FEC on M0" split that the
    // rest of this decoder already follows (see dmr_qr1676.hpp/
    // dmr_symbol_processor's caller for the actual decode+state machine).
    struct EmbeddedSignallingWindow {
        std::array<uint8_t, 6> bits48{};  // 48 bits, MSB-first packed
        uint8_t slot_phase{0};            // see Burst::slot_phase
        // The CACH ahead of this voice burst, when the superframe is
        // BS-sourced -- these bursts have no sync, so this is the only
        // way their CACH (and its Short LC fragment) gets read.
        bool has_cach{false};
        std::array<uint8_t, 3> cach_bits{};  // 24 bits, MSB-first packed
    };

    using BurstHandler = std::function<void(const Burst&)>;
    using EmbeddedSignallingHandler = std::function<void(const EmbeddedSignallingWindow&)>;

    explicit DmrSymbolProcessor(BurstHandler on_burst, EmbeddedSignallingHandler on_embedded_signalling = nullptr)
        : on_burst_(std::move(on_burst)), on_embedded_signalling_(std::move(on_embedded_signalling)) {
    }

    // Feed one recovered symbol (the output of clock_recovery_'s
    // symbol_handler in proc_dmr.cpp -- i.e. already timing-recovered,
    // NOT a raw discriminator sample).
    //
    // Embedded signalling (voice bursts B-F) carries no sync word, so its
    // windows are taken by timing from the slot's voice burst A: B starts
    // one same-slot burst (2 physical bursts, 576 bits) after A, and so on.
    // One schedule per slot phase -- the other slot's bursts sit in between
    // and must never reach the same LCSS assembler.
    //
    // process_symbol() arms a phase on every voice sync and disarms it on a
    // data/RC sync in the same phase. An armed schedule covers B-F of that
    // superframe plus B-F of the next one, so a single missed voice sync
    // costs nothing; it then expires unless re-armed.
    //
    // Survives reset_sync() (the carrier gate): inbound/simplex traffic
    // only carries one slot, so the carrier drops between its bursts.
    void process_symbol(float raw_symbol);

    // Equivalent of proc_dmr.cpp's on_carrier_lost(): discard in-flight
    // sync/burst state after a signal gap.
    void reset_sync();

    // For diagnostics/testing.
    float outer_amp_est() const { return outer_amp_est_; }
    uint64_t symbol_count() const { return symbol_count_; }
    // Running average of |raw_symbol| across ALL symbols (not just sync
    // hits) -- lets live telemetry compare the actual discriminator
    // scale against what offline analysis/the native test harness saw
    // (outer levels should read roughly in the 1000s of "Hz" with the
    // nominal_deviation_hz=1.0f discriminator scaling -- see
    // proc_dmr.hpp). If this reads near-zero or wildly different in
    // practice, the bug is upstream of this class (decimation/FM
    // discriminator), not in sync detection/slicing.
    float avg_abs_symbol() const { return avg_abs_symbol_; }
    // Closest (lowest Hamming distance) sync-word match seen the last
    // time a sync check actually ran, whether or not it was within
    // SYNC_ERROR_THRESHOLD. 25 (out of a possible max of 24) is the
    // "never checked yet" sentinel.
    uint8_t last_best_sync_err() const { return last_best_sync_err_; }
    // Tracked residual frequency error (Hz, same raw-Hz scale as
    // avg_abs_symbol()/outer_amp_est()) currently being subtracted from
    // every symbol before sign detection/slicing -- see process_symbol().
    // On real hardware with the TCXO disabled this can plausibly sit
    // anywhere from a few hundred Hz to several kHz; if it never
    // converges away from 0 while sync never fires, that itself is a
    // useful diagnostic (means the actual error is even larger than
    // this tracker's pull-in range, or something else is wrong).
    float dc_estimate() const { return dc_estimate_; }

   private:
    uint8_t history_bit(uint64_t absolute_bit) const;
    void push_burst();
    uint8_t slot_phase(uint64_t burst_start, uint8_t errors, bool& new_grid, bool& on_grid);
    void update_embedded_schedule(SyncType sync_type, uint8_t phase, uint64_t burst_start);
    void emit_embedded_windows();

    // Automatic frequency correction. A real DMR signal's dibit levels
    // average to ~0 Hz over many symbols (mixed sync words and payload
    // data are not skewed to one polarity), so any persistent non-zero
    // average is residual LO/oscillator frequency error, not signal --
    // and it's fatal to the sign-only sync correlator below even though
    // it barely shows up in avg_abs_symbol_ (which uses abs()).
    // Confirmed empirically: injecting as little as 1-3kHz of
    // uncorrected offset into an otherwise-known-good capture collapses
    // burst decode from consistently working to failing outright in the
    // native test harness -- and disabling the external TCXO (see
    // proc_dmr.hpp) makes a multi-kHz residual entirely plausible on
    // real hardware.
    //
    // dc_estimate_ is updated in process_symbol() ONLY on a confirmed-
    // good sync (best_err<=1, the same strict gate outer_amp_est_ uses),
    // anchored to that sync window's own known-good content -- NOT via
    // a blind per-symbol average over every symbol regardless of
    // context, which an earlier version of this tracker did. That
    // design is broken for exactly the traffic pattern this protocol
    // produces on a repeater: minority real signal (bursts), majority
    // noise/silence (the CACH-filled gaps between them, Tier I's actual
    // silence, and the long gaps between a periodic beacon's few-second
    // active windows). A continuous tracker drifts toward the NOISE's
    // own statistics during those majority-noise stretches, degrading
    // CC/DT decode confidence over time as it drifts on mostly-noise
    // input. Gating on sync quality fixes this at the root: pure noise
    // sits at ~12/24 (chance level) on the SAME best_err metric burst
    // acceptance already uses, nowhere near the <=1 threshold, so it
    // now literally cannot reach this update at all -- only content
    // already very close to a real sync word can.
    //
    // DC_ANCHOR_BLEND is a per-EVENT blend, not per-symbol -- updates
    // now fire only on confirmed-good syncs (far rarer than every
    // symbol during any real traffic pattern, but each one far more
    // trustworthy than a single arbitrary symbol) -- see
    // process_symbol() for the derivation of each update's residual-
    // bias measurement.
    static constexpr float DC_ANCHOR_BLEND = 0.05f;
    float dc_estimate_{0.0f};

    float outer_amp_est_{2000.0f};
    float avg_abs_symbol_{0.0f};
    uint8_t last_best_sync_err_{25};
    std::array<float, 256> amplitude_history_{};
    uint64_t symbol_count_{0};
    uint32_t sign_register_{0};

    std::array<uint8_t, 128> bit_history_buffer_{};
    size_t history_write_idx_{0};
    uint64_t bit_count_{0};

    struct PendingBurst {
        bool valid{false};
        uint64_t burst_start{0};
        uint64_t ready_at{0};
        SyncType sync_type{SyncType::BS_VOICE};
        uint8_t errors{0};
        float mag_ratio{0.0f};
    } pending_burst_{};

    // One physical burst period in bits: CACH(24) + Burst(264).
    static constexpr uint64_t PHYSICAL_BURST_PERIOD_BITS = BURST_BITS + CACH_BITS;
    // Same-slot burst period: the other slot's burst sits in between.
    static constexpr uint64_t SLOT_BURST_PERIOD_BITS = 2 * PHYSICAL_BURST_PERIOD_BITS;
    // Real bursts land within a symbol of the previous one's grid (0 or
    // +-2 bits on the test recordings); false syncs inside a voice burst
    // land tens of bits off. A burst further off than this starts a new
    // grid -- unless its sync was marginal, in which case it is ignored
    // for grid and embedded-schedule purposes.
    static constexpr uint64_t GRID_TOLERANCE_BITS = 8;
    // ...and so does one after more than ~1 s without a sync burst.
    static constexpr uint64_t GRID_MAX_GAP_BITS = 9600;
    // Superframe positions 1..5 (B-F) and 7..11 (B-F of the next one).
    static constexpr uint8_t EMBEDDED_LAST_INDEX = 11;

    bool has_anchor_{false};
    uint64_t anchor_start_{0};
    uint8_t anchor_phase_{0};

    struct ArmedEmbeddedSignalling {
        bool active{false};
        uint64_t voice_a_start{0};  // burst_start of this superframe's voice burst A
        bool bs_sourced{false};     // armed by a BS voice sync (bursts have a CACH)
        uint8_t next_index{1};      // superframe position of the next window (A=0)
    };
    std::array<ArmedEmbeddedSignalling, 2> armed_embedded_signalling_{};

    BurstHandler on_burst_;
    EmbeddedSignallingHandler on_embedded_signalling_;
};

#endif /*__DMR_SYMBOL_PROCESSOR_H__*/
