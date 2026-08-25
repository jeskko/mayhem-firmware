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

#ifndef __UI_DMR_RX_H__
#define __UI_DMR_RX_H__

#include "ui.hpp"
#include "ui_language.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "message.hpp"
#include "dmr_golay2087.hpp"
#include "dmr_bptc19696.hpp"
#include "dmr_fulllc.hpp"
#include "dmr_csbk.hpp"
#include "dmr_cach.hpp"
#include "dmr_embedded_lc.hpp"
#include <array>
#include <string>

using namespace ui;

namespace ui::external_app::dmr_rx {

// Burst-sync detection, Slot Type (Color Code/Data Type), and Full LC
// (talkgroup/talker ID) + a CSBK header (FID/CSBKO only for now -- see
// dmr_csbk.hpp). No CACH/Short LC yet (the M4 front end doesn't capture
// it), no Motorola-specific CSBK opcode fields yet (repeater/site ID,
// pending knowing which vendor family this control channel uses), and
// no voice decode at all -- see the project plan for the phased
// roadmap this is a slice of.
class DmrChannelDecoder {
   public:
    // ETSI TS 102 361-1 Table 9.4 Data Type field values (standard
    // across the protocol; confirmed both from the spec table and
    // independently from dsd-fme's own terminal labels -- "VLC"/"TLC"/
    // "IDLE"/"PI").
    static constexpr uint8_t DATA_TYPE_VOICE_LC_HEADER = 1;
    static constexpr uint8_t DATA_TYPE_TERMINATOR_WITH_LC = 2;
    static constexpr uint8_t DATA_TYPE_CSBK = 3;

    struct Result {
        bool is_ok{false};
        bool inverted{false};
        uint8_t sync_type{0xff};  // 0=BS Voice,1=BS Data,2=MS Voice,3=MS Data,4=RC
        uint8_t sync_errors{0};

        bool golay_ok{false};
        uint8_t color_code{0xff};
        uint8_t data_type{0xff};

        // Populated only when data_type is VOICE_LC_HEADER or
        // TERMINATOR_WITH_LC and BPTC(196,96) decoded cleanly.
        bool has_full_lc{false};
        dmr::FullLcResult full_lc{};

        // Populated only when data_type is CSBK and BPTC(196,96)
        // decoded cleanly. Header only for now -- see dmr_csbk.hpp for
        // why opcode-specific fields aren't parsed yet.
        bool has_csbk{false};
        dmr::CsbkHeader csbk{};
    };

    static std::string get_sync_name(uint8_t sync_type) {
        switch (sync_type) {
            case 0:
                return "BS-VOICE";
            case 1:
                return "BS-DATA";
            case 2:
                return "MS-VOICE";
            case 3:
                return "MS-DATA";
            case 4:
                return "RC";
            default:
                return "UNK";
        }
    }

    Result decode_burst(const DmrBurstMessage& message);

   private:
    static constexpr size_t SLOT_TYPE1_OFFSET = 98;
    static constexpr size_t SLOT_TYPE2_OFFSET = 156;
    static constexpr size_t SLOT_TYPE_HALF_BITS = 10;

    uint32_t read_bits(const uint8_t* bytes, size_t bit, size_t count) const;
};

// Full LC is retransmitted on every LC-bearing burst within a voice
// superframe (see dmr_fulllc.hpp) -- tracks one slot's talkgroup/talker
// display, requiring the same (dest, source) pair to be seen twice in a
// row before treating it as confirmed, since single-decode BPTC-
// corrected content can still be a false-positive burst-sync/Golay
// match rather than a real one.
struct SlotLcState {
    uint32_t talkgroup{0};
    uint32_t talker{0};
    bool is_group{false};
    bool confirmed{false};
    uint8_t service_options{0};
    uint8_t fid{0};

    uint32_t pending_dest{0};
    uint32_t pending_source{0};
    bool has_pending{false};

    // Returns true if `confirmed` (and therefore the display) changed.
    bool update(const dmr::FullLcResult& lc);
    std::string format(const char* label) const;
};

class DmrRxView : public View {
   public:
    DmrRxView(NavigationView& nav);
    ~DmrRxView();
    void focus() override;
    std::string title() const override { return "DMR RX"; };

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{};
    app_settings::SettingsManager settings_{"rx_dmr", app_settings::Mode::RX};

    // Tier II strictly alternates TS1/TS2 every burst; neither Slot Type
    // nor Full LC carries a per-burst slot-number field at this layer,
    // so this free-running counter is the fallback when CACH isn't
    // available (very start of reception, before enough bit history
    // exists, or genuine Tier I DMO which has no CACH at all). It has no
    // absolute reference of its own: a missed/unsynced burst, or simply
    // starting reception mid-cycle, desyncs it from reality until CACH's
    // TC bit resynchronizes it in on_data_dmr() (see there) or the next
    // carrier drop resets it outright.
    bool next_slot_is_ts2_{false};
    SlotLcState slot_lc_[2]{};  // index 0 = TS1, 1 = TS2

    // CACH-carried Short LC is a completely separate PDU/FEC path from
    // the burst content above (see dmr_cach.hpp) -- Tier II only, no
    // relation to next_slot_is_ts2_'s tracking.
    dmr::ShortLcAssembler short_lc_assembler_{};

    // A repeater's site beacon repeats constantly regardless of call
    // activity (far more often than Full LC/CSBK) -- update the display
    // only when the decoded value actually changes, not on every repeat,
    // to avoid spamming an unchanging value.
    bool has_shown_site_{false};
    uint8_t last_site_id_{0};
    uint8_t last_rest_channel_{0};

    // Same change-gating for Motorola Capacity Plus CSBKO=0x3B Adjacent
    // Sites (see dmr_csbk.hpp) -- a neighbor-site list broadcast
    // periodically alongside this site's own ID/rest channel above.
    bool has_shown_adjacent_{false};
    dmr::CapacityPlusAdjacentSites last_adjacent_{};

    // Same again for CSBKO=0x3E Channel Status (see dmr_csbk.hpp) --
    // indexed by the report's own `ts` field (which timeslot it
    // concerns), since a system can broadcast one independently per
    // slot.
    bool has_shown_channel_status_[2]{false, false};
    dmr::CapacityPlusChannelStatus last_channel_status_[2]{};

    // Connect Plus Adjacent Sites (FID=0x06 CSBKO=0x01, see
    // dmr_csbk.hpp) -- same change-gating rationale as Capacity Plus's
    // own Adjacent Sites above. Channel Grant/Voice Grant/Data Grant/
    // Slot Termination aren't gated at all -- they're transient,
    // per-call events (fire once or a few times as a call starts/ends),
    // not a constantly-repeating beacon, so every occurrence is logged.
    bool has_shown_connect_plus_adjacent_{false};
    dmr::ConnectPlusAdjacentSites last_connect_plus_adjacent_{};

    // Embedded Signalling (Talker Alias/GPS Info/redundant Full LC from
    // voice frames B-F, see dmr_embedded_lc.hpp's EmbeddedSignallingAssembler
    // and DmrSymbolProcessor::arm_embedded_signalling()). Deliberately a
    // SINGLE instance, not per-slot -- the M4 front end that feeds this
    // only tracks one armed sequence at a time regardless of slot (see
    // arm_embedded_signalling()'s own "Known limitation" comment), so
    // matching that here rather than pretending per-slot tracking exists.
    dmr::EmbeddedSignallingAssembler embedded_signalling_assembler_{};

    RxFrequencyField field_frequency{{UI_POS_X(0), UI_POS_Y(0)}, nav_};
    RFAmpField field_rf_amp{{UI_POS_X(13), UI_POS_Y(0)}};
    LNAGainField field_lna{{UI_POS_X(15), UI_POS_Y(0)}};
    VGAGainField field_vga{{UI_POS_X(18), UI_POS_Y(0)}};
    RSSI rssi{{UI_POS_X_RIGHT(9), UI_POS_Y(0), UI_POS_WIDTH(9), 4}};
    Channel channel{{UI_POS_X_RIGHT(9), UI_POS_Y(0) + 5, UI_POS_WIDTH(9), 4}};

    Text text_cc{{UI_POS_X(0), UI_POS_Y(1), UI_POS_WIDTH(15), UI_POS_HEIGHT(1)}, "CC:  --"};       // color code
    Text text_dt{{UI_POS_X(15), UI_POS_Y(1), UI_POS_WIDTH(15), UI_POS_HEIGHT(1)}, "DT:  --"};      // slot data type (raw)
    Text text_sync{{UI_POS_X(0), UI_POS_Y(2), UI_POS_WIDTH(15), UI_POS_HEIGHT(1)}, "SYNC: ----"};  // matched sync word
    // Repeater/site ID, from Short LC via CACH (Tier II only -- see
    // dmr_cach.hpp). Currently only Motorola Capacity Plus/Cap Max's
    // SLCO=0xF site beacon is parsed; other systems' site ID mechanisms
    // (standard ETSI Sys_Parms, Connect Plus) aren't wired up yet.
    Text text_site{{UI_POS_X(15), UI_POS_Y(2), UI_POS_WIDTH(15), UI_POS_HEIGHT(1)}, "SITE: --"};

    // Talkgroup/talker ID per timeslot, from Full Link Control (see
    // SlotLcState/dmr_fulllc.hpp). Repeater/site ID isn't decoded yet
    // (needs the Motorola-specific CSBK opcode this repeater's control
    // channel actually uses -- CSBK FID/opcode show up in the console
    // below in the meantime, to find out which).
    Text text_ts1{{UI_POS_X(0), UI_POS_Y(3), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, "TS1: --"};
    Text text_ts2{{UI_POS_X(0), UI_POS_Y(4), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, "TS2: --"};

    Console console{{UI_POS_X(0), UI_POS_Y(5) + 8, UI_POS_MAXWIDTH, screen_height - (UI_POS_Y(6) + 8)}};

    DmrChannelDecoder decoder{};

    void on_data_dmr(const DmrBurstMessage&);
    void on_data_embedded_signalling(const DmrEmbeddedSignallingMessage&);
    void on_data_debug(const DmrDebugMessage&);
    void on_freqchg(int64_t freq);

    // Writes to the on-screen console AND the global app_debug_log ring
    // buffer (see app_debug_log.hpp) -- queryable over the USB-serial
    // shell via `applog`, so a live capture doesn't need a screenframe/
    // screenshot round-trip just to see what this app's console has
    // logged, and survives past whatever currently fits on screen.
    void log(const std::string& line);

    MessageHandlerRegistration message_handler_packet{
        Message::ID::DmrBurst,
        [this](Message* const p) {
            this->on_data_dmr(*static_cast<const DmrBurstMessage*>(p));
        }};

    MessageHandlerRegistration message_handler_debug{
        Message::ID::DmrDebug,
        [this](Message* const p) {
            this->on_data_debug(*static_cast<const DmrDebugMessage*>(p));
        }};

    MessageHandlerRegistration message_handler_embedded_signalling{
        Message::ID::DmrEmbeddedSignalling,
        [this](Message* const p) {
            this->on_data_embedded_signalling(*static_cast<const DmrEmbeddedSignallingMessage*>(p));
        }};

    // Lets the shell's `setfreq` command retune this app remotely (see
    // usb_serial_shell.cpp's cmd_setfreq) -- otherwise unhandled
    // FreqChangeCommandMessages are simply ignored, same pattern as
    // e.g. SondeView::on_freqchg().
    MessageHandlerRegistration message_handler_freqchg{
        Message::ID::FreqChangeCommand,
        [this](Message* const p) {
            const auto message = static_cast<const FreqChangeCommandMessage*>(p);
            this->on_freqchg(message->freq);
        }};
};

}  // namespace ui::external_app::dmr_rx

#endif
