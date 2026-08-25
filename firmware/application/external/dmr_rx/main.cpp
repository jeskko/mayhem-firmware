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
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::dmr_rx {
void initialize_app(ui::NavigationView& nav) {
    nav.push<DmrRxView>();
}
}  // namespace ui::external_app::dmr_rx
extern "C" {

// Reuses the existing (previously unused) bitmap_icon_dmr asset from
// firmware/application/bitmap.hpp.
__attribute__((section(".external_app.app_dmr_rx.application_information"), used)) application_information_t _application_information_dmr_rx = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::dmr_rx::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "DMR",
    /*.bitmap_data = */ {
        0x00,
        0x00,
        0xFE,
        0x1F,
        0xFE,
        0x3F,
        0x0E,
        0x78,
        0x0E,
        0x70,
        0x0E,
        0x70,
        0x0E,
        0x70,
        0x0E,
        0x78,
        0xFE,
        0x3F,
        0xFE,
        0x1F,
        0x8E,
        0x07,
        0x0E,
        0x0F,
        0x0E,
        0x1E,
        0x0E,
        0x3C,
        0x0E,
        0x78,
        0x00,
        0x00,
    },
    /*.icon_color = */ ui::Color::orange().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = portapack::spi_flash::image_tag_dmrrx*/ {'P', 'D', 'M', 'R'},
    /*.m4_app_offset = */ 0x00000000,  // will be filled at compile time
};
}
