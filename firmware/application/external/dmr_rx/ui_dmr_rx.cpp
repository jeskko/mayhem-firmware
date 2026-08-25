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

#include "ui_dmr_rx.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "usb_serial_device_to_host.h"
#include "app_debug_log.hpp"

namespace {
// Diagnostic-only: dumps the M4 front end's amp/out/car/best/dc state to
// the serial console once a second (see on_data_debug()). It earned its
// keep tracking down the AFC frequency-offset bug (see dmr_symbol_
// processor.hpp) -- kept, but off by default, in case a future front-
// end change needs the same kind of live visibility again. Flip to
// `true` and reflash to re-enable; no UI-visible effect either way.
constexpr bool kSerialTelemetryEnabled = false;

// Writes straight to the USB-serial console (the same one the "info"/
// shell commands use), bypassing the on-screen UI entirely -- the
// telemetry line doesn't fit legibly on the device's own display.
// SUSBD1's output buffer is small (128 bytes), so keep lines short.
//
// fillOBuffer() blocks the calling thread (TIME_INFINITE) once the queue
// is full -- fine for the shell, where something is always draining it,
// but fatal here: this runs on the UI thread, and with no listener on
// the serial port the queue fills after a handful of lines and the
// whole app freezes. Check free space first (chQSpaceI(), guarded by
// chSysLock/chSysUnlock since it's an "I-class" call) for the WHOLE
// line, not just "not entirely full" -- a partial fit would still let
// fillOBuffer write some bytes then block waiting for the rest to
// drain. Just drop the line if it won't fit outright: telemetry is
// diagnostic-only, losing a line when nobody's listening is fine;
// blocking the UI is not.
void serial_println(const std::string& line) {
    std::string out = line + "\r\n";

    chSysLock();
    const size_t space = chQSpaceI(&SUSBD1.oqueue);
    chSysUnlock();
    if (space < out.length()) return;

    fillOBuffer(&SUSBD1.oqueue, reinterpret_cast<const uint8_t*>(out.c_str()), out.length());
}
}  // namespace

using namespace portapack;
using namespace ui;

namespace ui::external_app::dmr_rx {

uint32_t DmrChannelDecoder::read_bits(const uint8_t* bytes, size_t bit, size_t count) const {
    uint32_t value = 0;
    for (size_t i = 0; i < count; i++) {
        value <<= 1;
        value |= (bytes[(bit + i) >> 3] >> (7 - ((bit + i) & 7))) & 1;
    }
    return value;
}

DmrChannelDecoder::Result DmrChannelDecoder::decode_burst(const DmrBurstMessage& message) {
    Result result{};
    result.inverted = message.inverted;
    result.sync_type = message.sync_type;
    result.sync_errors = message.sync_errors;

    // Slot Type is a 20-bit Golay(20,8,7) codeword split either side of
    // the burst's Sync field: 10 bits just before Sync, 10 bits just
    // after.
    const uint32_t half1 = read_bits(message.payload.data(), SLOT_TYPE1_OFFSET, SLOT_TYPE_HALF_BITS);
    const uint32_t half2 = read_bits(message.payload.data(), SLOT_TYPE2_OFFSET, SLOT_TYPE_HALF_BITS);
    const uint32_t slot_type_word = (half1 << SLOT_TYPE_HALF_BITS) | half2;

    const auto golay = dmr::golay2087_decode(slot_type_word);
    result.golay_ok = golay.ok;
    result.color_code = golay.color_code;
    result.data_type = golay.data_type;

    // Phase 1 scope: report the burst as "decoded" once we have a sync
    // match. Color Code/Data Type are shown either way (nearest-
    // codeword result), but golay_ok tells the caller whether the
    // Golay(20,8,7) check found <=3 bit errors -- its guaranteed-
    // correct region -- so the UI can flag results beyond that as
    // unreliable rather than silently trusting them.
    result.is_ok = true;

    // Full LC (Voice LC Header/Terminator with LC) and CSBK both ride
    // on BPTC(196,96), across the burst's Info1(98)+Info2(98) fields --
    // everything between the two Slot Type halves either side of Sync
    // is skipped, since it isn't part of this codeword.
    if (result.data_type == DATA_TYPE_VOICE_LC_HEADER ||
        result.data_type == DATA_TYPE_TERMINATOR_WITH_LC ||
        result.data_type == DATA_TYPE_CSBK) {
        uint8_t bits196[196];
        for (size_t i = 0; i < 98; i++) bits196[i] = static_cast<uint8_t>(read_bits(message.payload.data(), i, 1));
        for (size_t i = 0; i < 98; i++) bits196[98 + i] = static_cast<uint8_t>(read_bits(message.payload.data(), 166 + i, 1));

        uint8_t pdu96[96];
        if (dmr::bptc19696_decode(bits196, pdu96)) {
            if (result.data_type == DATA_TYPE_CSBK) {
                result.csbk = dmr::csbk_decode_header(pdu96);
                result.has_csbk = true;
            } else {
                result.full_lc = dmr::fulllc_decode(pdu96, result.data_type == DATA_TYPE_TERMINATOR_WITH_LC);
                result.has_full_lc = result.full_lc.recognized;
            }
        }
    }

    return result;
}

bool SlotLcState::update(const dmr::FullLcResult& lc) {
    const bool matched_pending = has_pending && pending_dest == lc.dest_address &&
                                 pending_source == lc.source_address;
    pending_dest = lc.dest_address;
    pending_source = lc.source_address;
    has_pending = true;

    if (!matched_pending) return false;

    const bool changed = !confirmed || talkgroup != lc.dest_address ||
                         talker != lc.source_address;
    talkgroup = lc.dest_address;
    talker = lc.source_address;
    is_group = (lc.flco == static_cast<uint8_t>(dmr::Flco::GROUP_VOICE));
    service_options = lc.service_options;
    fid = lc.fid;
    confirmed = true;
    return changed;
}

std::string SlotLcState::format(const char* label) const {
    if (!confirmed) return std::string(label) + ": --";
    std::string s = std::string(label) + (is_group ? " TG:" : " ID:") + to_string_dec_uint(talkgroup) +
                    " SRC:" + to_string_dec_uint(talker);
    const auto so = dmr::decode_service_options(service_options, fid);
    if (so.emergency) s += " EMERG";
    if (so.privacy) s += " ENC";
    if (so.priority) s += " P" + to_string_dec_uint(so.priority);
    return s;
}

DmrRxView::DmrRxView(NavigationView& nav)
    : nav_{nav} {
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&rssi, &channel, &field_rf_amp, &field_lna, &field_vga,
                  &field_frequency, &text_cc, &text_dt, &text_sync, &text_site,
                  &text_ts1, &text_ts2, &console});

    field_frequency.set_step(12500);

    receiver_model.set_modulation(ReceiverModel::Mode::NarrowbandFMAudio);
    receiver_model.set_sampling_rate(3072000);
    receiver_model.set_baseband_bandwidth(1750000);
    receiver_model.set_squelch_level(0);
    receiver_model.enable();
}

void DmrRxView::focus() {
    field_frequency.focus();
}

void DmrRxView::log(const std::string& line) {
    console.writeln(line);
    app_debug_log::push(line);
}

void DmrRxView::on_freqchg(int64_t freq) {
    field_frequency.set_value(freq);

    // Everything below is per-signal state -- a stale talkgroup/site/
    // adjacent-sites value left over from whatever was previously tuned
    // would otherwise keep showing (and keep being logged) as if it
    // were still current, which is actively misleading when retuning
    // between completely unrelated systems.
    next_slot_is_ts2_ = false;
    slot_lc_[0] = SlotLcState{};
    slot_lc_[1] = SlotLcState{};
    has_shown_site_ = false;
    last_site_id_ = 0;
    last_rest_channel_ = 0;
    has_shown_adjacent_ = false;
    last_adjacent_ = dmr::CapacityPlusAdjacentSites{};
    has_shown_channel_status_[0] = false;
    has_shown_channel_status_[1] = false;
    last_channel_status_[0] = dmr::CapacityPlusChannelStatus{};
    last_channel_status_[1] = dmr::CapacityPlusChannelStatus{};
    has_shown_connect_plus_adjacent_ = false;
    last_connect_plus_adjacent_ = dmr::ConnectPlusAdjacentSites{};

    text_cc.set("CC:  --");
    text_dt.set("DT:  --");
    text_sync.set("SYNC: ----");
    text_site.set("SITE: --");
    text_ts1.set(slot_lc_[0].format("TS1"));
    text_ts2.set(slot_lc_[1].format("TS2"));

    log("--- retuned ---");
}

void DmrRxView::on_data_dmr(const DmrBurstMessage& message) {
    auto result = decoder.decode_burst(message);

    // CACH's TC bit (Tier II only) is authoritative ground truth for
    // which slot THIS burst is on -- ETSI TS 102 361-1 SS6.3, confirmed
    // against dsd-fme's src/dmr_bs.c (`state->currentslot =
    // tact_bits[1]`, the exact same bit this project's own
    // CachHeader::tc is: false=TS1, true=TS2). Decoded once here (ahead
    // of decode_burst()'s own has_cach handling below, which reuses this
    // same `cach` for Short LC) specifically to resync
    // next_slot_is_ts2_ -- see its declaration for why the free-running
    // fallback alone can't self-correct a bad starting guess.
    dmr::CachHeader cach{};
    if (message.has_cach) cach = dmr::cach_decode_header(message.cach_payload.data());

    int slot;
    if (cach.tact_ok) {
        slot = cach.tc ? 1 : 0;
        next_slot_is_ts2_ = !cach.tc;  // the *next* burst is the other slot
    } else {
        slot = next_slot_is_ts2_ ? 1 : 0;
        next_slot_is_ts2_ = !next_slot_is_ts2_;
    }

    // Single reusable line buffer for this whole function, mutated with
    // += rather than built via chained + (which allocates a fresh
    // temporary std::string per operator+, several of them alive at once
    // across a multi-piece concatenation) -- a real, worthwhile stack
    // reduction (confirmed via disassembly: ~300 bytes down to ~180).
    std::string line;

    // Slot Type with more than the guaranteed-correct 3 bit errors (see
    // dmr_golay2087.hpp) is a nearest-codeword GUESS, not a verified
    // decode -- these show up constantly on noise (a false sync match
    // within SYNC_ERROR_THRESHOLD, or genuine signal with a corrupted
    // Slot Type field) and used to be displayed/logged with a "?"
    // marker. Suppressing them outright instead: leave the SYNC/CC/DT
    // display holding its last confident value rather than replacing it
    // with a low-confidence guess, and log nothing to the console for
    // these at all -- only a verified decode updates either.
    if (result.is_ok && result.golay_ok) {
        line = "SYNC: ";
        line += decoder.get_sync_name(result.sync_type);
        text_sync.set(line);

        line = "CC:  ";
        line += to_string_dec_uint(result.color_code);
        text_cc.set(line);

        line = "DT:  ";
        line += to_string_dec_uint(result.data_type);
        text_dt.set(line);

        line = decoder.get_sync_name(result.sync_type);
        line += " CC:";
        line += to_string_dec_uint(result.color_code);
        line += " DT:";
        line += to_string_dec_uint(result.data_type);
        line += " E:";
        line += to_string_dec_uint(result.sync_errors);
        // Magnitude-consistency ratio for THIS burst's own sync window
        // (see DmrSymbolProcessor::MAG_RATIO_REJECT_THRESHOLD) -- only
        // ever <= that threshold (anything higher was already rejected
        // before a burst could exist at all), shown here specifically to
        // help judge from live use whether the threshold can be
        // tightened further without starting to reject genuine bursts.
        line += " R:";
        line += to_string_decimal(message.sync_mag_ratio, 1);
        log(line);

        // Voice LC Header marks the start of a brand new call -- reset
        // the embedded-signalling LCSS/BPTC assembly state so it
        // doesn't try to complete a sequence using a stale fragment
        // left over from before (a previous call, or noise). See
        // EmbeddedSignallingAssembler::reset()'s own comment for why
        // this is a robustness touch, not strictly required for
        // correctness. The M4 side's own window-tracking re-arm (see
        // DmrSymbolProcessor::arm_embedded_signalling()) happens
        // unconditionally on every confirmed voice-sync burst
        // independently of this.
        if (result.data_type == DmrChannelDecoder::DATA_TYPE_VOICE_LC_HEADER) {
            embedded_signalling_assembler_.reset();
        }
    }

    // Full LC/CSBK aren't gated on golay_ok above -- they use their own
    // independent, equally strong confidence checks (BPTC + a 2-in-a-
    // row repeat match for Full LC; BPTC + CRC-16 for CSBK), so a burst
    // with a noisy Slot Type field can still yield a trustworthy Full
    // LC/CSBK decode, and one shouldn't be suppressed by the other's
    // unrelated failure.
    if (result.has_full_lc && slot_lc_[static_cast<size_t>(slot)].update(result.full_lc)) {
        text_ts1.set(slot_lc_[0].format("TS1"));
        text_ts2.set(slot_lc_[1].format("TS2"));
        // Logged (not just displayed) specifically so a slot<->talkgroup
        // correlation can be checked over time via `applog` -- the
        // on-screen TS1/TS2 fields alone only show the LATEST confirmed
        // value per slot, which could miss a transient call on the
        // "wrong" slot that got overwritten before anyone looked.
        log(slot_lc_[static_cast<size_t>(slot)].format(slot == 0 ? "TS1" : "TS2"));
    }

    // Most CSBK opcode fields for this control channel's vendor/opcode
    // family still aren't decoded (see dmr_csbk.hpp) -- log raw
    // FID/CSBKO so reliable CSBK reception can eventually reveal which
    // others matter. One opcode IS decoded below: Motorola Capacity
    // Plus/Cap Max's CSBKO=0x3B Adjacent Sites, a neighbor-site list
    // independent of this site's own ID (which comes from Short LC
    // instead).
    if (result.has_csbk && result.csbk.crc_ok) {
        line = "CSBK FID:";
        line += to_string_dec_uint(result.csbk.fid);
        line += " O:";
        line += to_string_dec_uint(result.csbk.csbko);
        log(line);

        if (result.csbk.fid == 0x10 && result.csbk.csbko == 0x3B) {
            const auto adjacent = dmr::csbk_decode_capacity_plus_adjacent_sites(result.csbk);
            bool changed = has_shown_adjacent_ != adjacent.recognized;
            for (size_t i = 0; !changed && i < adjacent.sites.size(); i++) {
                changed = last_adjacent_.sites[i].valid != adjacent.sites[i].valid ||
                          last_adjacent_.sites[i].site_id != adjacent.sites[i].site_id ||
                          last_adjacent_.sites[i].rest_channel != adjacent.sites[i].rest_channel;
            }
            if (changed) {
                has_shown_adjacent_ = true;
                last_adjacent_ = adjacent;
                line = "ADJACENT:";
                for (const auto& site : adjacent.sites) {
                    if (!site.valid) continue;
                    line += " ";
                    line += to_string_dec_uint(site.site_id);
                    line += "/";
                    line += to_string_dec_uint(site.rest_channel);
                }
                log(line);
            }
        }

        if (result.csbk.fid == 0x10 && result.csbk.csbko == 0x3E) {
            const auto cs = dmr::csbk_decode_capacity_plus_channel_status(result.csbk);
            if (cs.ts <= 1) {
                auto& last = last_channel_status_[cs.ts];
                bool changed = has_shown_channel_status_[cs.ts] != cs.recognized ||
                               last.has_channel_map != cs.has_channel_map ||
                               last.rest_channel != cs.rest_channel;
                for (int i = 0; !changed && i < 8; i++)
                    changed = last.channel_busy[static_cast<size_t>(i)] != cs.channel_busy[static_cast<size_t>(i)];
                if (changed) {
                    has_shown_channel_status_[cs.ts] = true;
                    last = cs;
                    line = "CHANSTAT TS";
                    line += to_string_dec_uint(cs.ts);
                    line += " rest:";
                    line += to_string_dec_uint(cs.rest_channel);
                    if (cs.has_channel_map) {
                        line += " map:";
                        for (int i = 0; i < 8; i++) {
                            if (i + 1 == cs.rest_channel)
                                line += "R";
                            else
                                line += cs.channel_busy[static_cast<size_t>(i)] ? "B" : ".";
                        }
                    } else {
                        line += " (multi-block, not decoded)";
                    }
                    log(line);
                }
            }
        }

        // "Channel Grant" family (CSBKO 48-56, see dmr_csbk.hpp) --
        // deliberately not restricted to fid==0x10: the reference this
        // was ported from evaluates it unconditionally on FID, since
        // it's the one CSBK family treated as shared across vendor
        // control-channel dialects rather than tied to one. This is the
        // actual per-call channel/talkgroup announcement mechanism, as
        // opposed to the FID=0x10-specific site/status messages above.
        if (result.csbk.csbko >= 48 && result.csbk.csbko <= 56) {
            const auto grant = dmr::csbk_decode_channel_grant(result.csbk);
            if (grant.recognized) {
                line = "GRANT O:";
                line += to_string_dec_uint(grant.csbko);
                line += " ch:";
                line += to_string_dec_uint(grant.logical_channel);
                line += " ts:";
                line += to_string_dec_uint(grant.timeslot);
                if (grant.emergency) line += " EMERGENCY";
                line += " tgt:";
                line += to_string_dec_uint(grant.target_address);
                line += " src:";
                line += to_string_dec_uint(grant.source_address);
                log(line);
            }
        }

        // Motorola Connect Plus (FID=0x06) -- a different CSBK vendor
        // family from Capacity Plus/Cap Max above, see dmr_csbk.hpp.
        if (result.csbk.fid == 0x06) {
            if (result.csbk.csbko == 0x01) {
                const auto adj = dmr::csbk_decode_connect_plus_adjacent_sites(result.csbk);
                bool changed = has_shown_connect_plus_adjacent_ != adj.recognized;
                for (int i = 0; !changed && i < 5; i++)
                    changed = last_connect_plus_adjacent_.sites[static_cast<size_t>(i)] != adj.sites[static_cast<size_t>(i)];
                if (changed) {
                    has_shown_connect_plus_adjacent_ = true;
                    last_connect_plus_adjacent_ = adj;
                    line = "CONP ADJACENT:";
                    for (uint8_t site : adj.sites) {
                        if (!site) continue;
                        line += " ";
                        line += to_string_dec_uint(site);
                    }
                    log(line);
                }
            }
            if (result.csbk.csbko == 0x03) {
                const auto grant = dmr::csbk_decode_connect_plus_voice_grant(result.csbk);
                if (grant.recognized) {
                    line = "CONP VOICE opt:";
                    line += to_string_dec_uint(grant.call_option);
                    line += " ch:";
                    line += to_string_dec_uint(grant.logical_channel);
                    line += " ts:";
                    line += to_string_dec_uint(grant.timeslot);
                    line += " tgt:";
                    line += to_string_dec_uint(grant.target_address);
                    line += " src:";
                    line += to_string_dec_uint(grant.source_address);
                    log(line);
                }
            }
            if (result.csbk.csbko == 0x06) {
                const auto grant = dmr::csbk_decode_connect_plus_data_grant(result.csbk);
                if (grant.recognized) {
                    line = "CONP DATA ch:";
                    line += to_string_dec_uint(grant.logical_channel);
                    line += " ts:";
                    line += to_string_dec_uint(grant.timeslot);
                    line += " tgt:";
                    line += to_string_dec_uint(grant.target_address);
                    log(line);
                }
            }
            if (result.csbk.csbko == 0x0C) {
                const auto term = dmr::csbk_decode_connect_plus_slot_termination(result.csbk);
                if (term.recognized) {
                    line = "CONP TERM tgt:";
                    line += to_string_dec_uint(term.target_address);
                    log(line);
                }
            }
        }

        // Standard ETSI CSBK family -- see dmr_csbk.hpp. Logged every
        // time since these are transient (call-related or one-off),
        // same as GRANT/CONP above.
        const auto std_csbk = dmr::csbk_decode_standard(result.csbk);
        if (std_csbk.recognized) {
            switch (std_csbk.type) {
                case dmr::StandardCsbkType::CHANNEL_TIMING:
                    log("CT_CSBK");
                    break;
                case dmr::StandardCsbkType::NACK_RESPONSE:
                    line = "NACK tgt:";
                    line += to_string_dec_uint(std_csbk.target_address);
                    line += " src:";
                    line += to_string_dec_uint(std_csbk.source_address);
                    log(line);
                    break;
                case dmr::StandardCsbkType::ANNOUNCEMENT:
                    line = "ANNOUNCE type:";
                    line += to_string_dec_uint(std_csbk.announcement_type);
                    log(line);
                    break;
                case dmr::StandardCsbkType::BS_OUTBOUND_ACTIVATION:
                    line = "BS_DWN_ACT tgt:";
                    line += to_string_dec_uint(std_csbk.target_address);
                    line += " src:";
                    line += to_string_dec_uint(std_csbk.source_address);
                    log(line);
                    break;
                case dmr::StandardCsbkType::MOVE:
                    log("C_MOVE");
                    break;
                case dmr::StandardCsbkType::PREAMBLE:
                    line = std_csbk.preamble_is_group ? "PREAMBLE grp" : "PREAMBLE ind";
                    line += std_csbk.preamble_is_data ? " data" : " csbk";
                    line += " blocks:";
                    line += to_string_dec_uint(std_csbk.preamble_blocks);
                    line += " tgt:";
                    line += to_string_dec_uint(std_csbk.target_address);
                    line += " src:";
                    line += to_string_dec_uint(std_csbk.source_address);
                    log(line);
                    break;
                default:
                    break;
            }
        }
    }

    // CACH/Short LC is a completely separate PDU/FEC path from the
    // burst content above (Tier II only -- see dmr_cach.hpp), assembled
    // across 4 consecutive bursts regardless of the has_full_lc/has_csbk
    // outcome for any of them. Only updates the display when the value
    // actually changes (a site beacon repeats constantly regardless of
    // call activity) -- to avoid spamming an unchanging value. `cach`
    // itself was already decoded above (for TS1/TS2 resync), reused
    // here rather than decoded twice.
    if (message.has_cach) {
        dmr::ShortLcResult slc;
        if (short_lc_assembler_.add_fragment(cach, slc) && slc.recognized &&
            slc.has_capacity_plus_site) {
            const bool changed = !has_shown_site_ ||
                                 last_site_id_ != slc.capacity_plus_site.site_id ||
                                 last_rest_channel_ != slc.capacity_plus_site.rest_channel;
            if (changed) {
                has_shown_site_ = true;
                last_site_id_ = slc.capacity_plus_site.site_id;
                last_rest_channel_ = slc.capacity_plus_site.rest_channel;
                line = "SITE: ";
                line += to_string_dec_uint(last_site_id_);
                line += " R:";
                line += to_string_dec_uint(last_rest_channel_);
                text_site.set(line);
            }
        }
    }
}

void DmrRxView::on_data_embedded_signalling(const DmrEmbeddedSignallingMessage& message) {
    const auto result = embedded_signalling_assembler_.add_window(message.payload.data());
    if (!result.checksum_ok) return;

    // Logs every successful checksum-verified embedded LC fragment, not
    // just Talker Alias -- a decode with FLCO 0x00/0x03 (redundant
    // Group/Unit-to-Unit Voice LC, not decoded further yet -- see
    // dmr_embedded_lc.hpp) or 0x08 (GPS Info, also not decoded further
    // yet) would otherwise be completely invisible even when the
    // underlying LCSS/BPTC/checksum mechanism is working correctly.
    std::string line = "EMBEDDED LC: FLCO=0x";
    line += to_string_hex(result.flco, 2);
    log(line);

    // Full LC (0x00 Group/0x03 Unit-to-unit) and GPS Info (0x08) are
    // recognized but not decoded further yet -- see dmr_embedded_lc.hpp
    // for why (Full LC already gives this at call start; GPS Info is a
    // lower-priority follow-up). Only Talker Alias gets its own text.
    if (!result.talker_alias_updated || !result.talker_alias.updated) return;

    line = "TALKER ALIAS: ";
    line += result.talker_alias.text;
    log(line);
}

void DmrRxView::on_data_debug(const DmrDebugMessage& message) {
    if (!kSerialTelemetryEnabled) return;

    // dc: tracked residual frequency error (Hz) currently being removed
    // pre-slicing -- see DmrSymbolProcessor::DC_TRACK_ALPHA. Signed, so
    // render it with to_string_dec_int rather than _uint.
    serial_println(
        "dmr amp:" + to_string_dec_uint(static_cast<uint32_t>(message.avg_abs_symbol)) +
        " out:" + to_string_dec_uint(static_cast<uint32_t>(message.outer_amp_est)) +
        " car:" + (message.carrier_present ? "Y" : "N") +
        " best:" + to_string_dec_uint(message.last_best_sync_err) +
        " n:" + to_string_dec_uint(message.symbols_seen) +
        " dc:" + to_string_dec_int(static_cast<int32_t>(message.dc_estimate)));
}

DmrRxView::~DmrRxView() {
    receiver_model.disable();
    baseband::shutdown();
}

}  // namespace ui::external_app::dmr_rx
