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
 * Post-discriminator matched/pulse-shaping filter for DMR 4FSK.
 *
 * Found missing via a direct side-by-side read of dsd-fme's own demod
 * pipeline (dsd_symbol.c/dsd_filters.c) against this project's: our
 * pipeline (proc_dmr.cpp) only channel-filters the complex I/Q BEFORE
 * FM discrimination (decim_0/decim_1); nothing shapes the discriminator's
 * own output afterward besides Gardner clock recovery's minimal 2-tap
 * linear interpolation. dsd-fme runs every raw discriminator sample
 * through a dedicated FIR first. This class is that missing stage,
 * applied to the discriminator output BEFORE clock_recovery_ in
 * proc_dmr.cpp (and the equivalent spot in the native test harness).
 *
 * Offline A/B testing confirmed a real, large improvement on marginal/
 * noisy channels and no measurable regression on clean ones: a
 * previously near-undecodable channel went from effectively no Full LC
 * hits to reliably recovering many real DEST/SRC pairs, while a clean
 * reference channel showed only noise-level (not a real) change.
 *
 * Coefficients: the exact 61-tap root-raised-cosine filter (alpha=0.7,
 * Fc=48kHz) dsd-fme uses for DMR (src/dsd_filters.c, credited there in
 * comments to F4EXB). dsd-fme's own COPYRIGHT file licenses this file's
 * code under ISC, whose terms require the notice below to travel with
 * the reused values:
 *
 *   Copyright (C) 2010 DSD Author
 *
 *   Permission to use, copy, modify, and/or distribute this software
 *   for any purpose with or without fee is hereby granted, provided
 *   that the above copyright notice and this permission notice appear
 *   in all copies.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES
 *   WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR
 *   ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
 *   DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 *   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 *   ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 *   OF THIS SOFTWARE.
 *
 * (Not the same situation as the BPTC/Golay/RS tables elsewhere in this
 * project: those are independently implemented and cross-validated
 * against reference decoders, not reused source values -- see
 * dmr_bptc19696.cpp and dmr_golay2087.cpp. dmr_rs129.cpp IS a direct
 * port like this file, but from dmrshark under GPLv3-or-later, not ISC
 * -- see its own header.)
 *
 * No hardware dependency (same reasoning as dmr_symbol_processor.hpp):
 * linkable into tools/dmr_native_test as-is.
 */

#ifndef __DMR_MATCHED_FILTER_H__
#define __DMR_MATCHED_FILTER_H__

#include <array>
#include <cstddef>

class DmrMatchedFilter {
   public:
    // Applies the FIR to one new discriminator sample and returns the
    // filtered output. Symmetric (linear-phase) taps, so the exact
    // history traversal direction doesn't affect the result -- only
    // that the same fixed pairing of history sample <-> coefficient is
    // used every call, which the circular buffer below guarantees.
    float operator()(float sample) {
        history_[write_idx_] = sample;
        write_idx_ = (write_idx_ + 1) % TAPS;

        float sum = 0.0f;
        size_t idx = write_idx_;
        for (size_t i = 0; i < TAPS; i++) {
            idx = (idx == 0) ? TAPS - 1 : idx - 1;
            sum += history_[idx] * COEFFS[i];
        }
        return sum * (1.0f / GAIN);
    }

   private:
    static constexpr size_t TAPS = 61;
    static constexpr float GAIN = 6.82973073748f;

    // DMR filter F4EXB - root raised cosine alpha=0.7 Ts=6650 S/s Fc=48kHz
    // (dsd-fme's own comment, verbatim -- reproduced here for traceability
    // even though the "Ts=6650 S/s" figure doesn't obviously correspond to
    // DMR's 4800 baud; the filter is empirically validated regardless, see
    // the A/B test results above).
    static constexpr std::array<float, TAPS> COEFFS{
        0.0301506278f,
        0.0269200615f,
        0.0159662432f,
        -0.0013114705f,
        -0.0216605133f,
        -0.0404938748f,
        -0.0528141756f,
        -0.0543747957f,
        -0.0428325003f,
        -0.0186176083f,
        0.0147202645f,
        0.0508418571f,
        0.0816392577f,
        0.0988113688f,
        0.0957187780f,
        0.0691512084f,
        0.0206194642f,
        -0.0431564563f,
        -0.1107569268f,
        -0.1675773224f,
        -0.1981519842f,
        -0.1889130786f,
        -0.1308939560f,
        -0.0218608492f,
        0.1325685970f,
        0.3190962499f,
        0.5182530574f,
        0.7070497652f,
        0.8623526878f,
        0.9644213921f,
        1.0000000000f,
        0.9644213921f,
        0.8623526878f,
        0.7070497652f,
        0.5182530574f,
        0.3190962499f,
        0.1325685970f,
        -0.0218608492f,
        -0.1308939560f,
        -0.1889130786f,
        -0.1981519842f,
        -0.1675773224f,
        -0.1107569268f,
        -0.0431564563f,
        0.0206194642f,
        0.0691512084f,
        0.0957187780f,
        0.0988113688f,
        0.0816392577f,
        0.0508418571f,
        0.0147202645f,
        -0.0186176083f,
        -0.0428325003f,
        -0.0543747957f,
        -0.0528141756f,
        -0.0404938748f,
        -0.0216605133f,
        -0.0013114705f,
        0.0159662432f,
        0.0269200615f,
        0.0301506278f,
    };

    std::array<float, TAPS> history_{};
    size_t write_idx_{0};
};

#endif /*__DMR_MATCHED_FILTER_H__*/
