/*
 * Encode/decode tables ported from OpenGD77's QR1676.c
 * (https://github.com/rogerclarkmelbourne/OpenGD77):
 *
 *   Copyright (C) 2015, 2016 by Jonathan Naylor G4KLX
 *   Ported to OpenGD77 by Roger Clark VK3KYY / G4KYF
 *
 * Ported to PortaPack/Mayhem (decoder-only; also fixes a final-shift
 * bug in the original decoder -- see below), Copyright (C) 2026
 * Jarkko Vääräniemi.
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

#include "dmr_qr1676.hpp"

namespace dmr {
namespace {

// Ported verbatim (extracted programmatically from the source, not
// hand-retyped, to rule out transcription error) from OpenGD77's
// firmware/source/hotspot/QR1676.c DECODING_TABLE_1576 -- syndrome ->
// error-pattern lookup for the (15,7,5) shortened code this uses
// internally (256 = 2^8 possible syndromes for the degree-8 generator
// polynomial below).
constexpr uint32_t DECODING_TABLE_1576[256] = {
    0x0000U,
    0x0001U,
    0x0002U,
    0x0003U,
    0x0004U,
    0x0005U,
    0x0006U,
    0x4020U,
    0x0008U,
    0x0009U,
    0x000AU,
    0x000BU,
    0x000CU,
    0x000DU,
    0x2081U,
    0x2080U,
    0x0010U,
    0x0011U,
    0x0012U,
    0x0013U,
    0x0014U,
    0x0C00U,
    0x0016U,
    0x0C02U,
    0x0018U,
    0x0120U,
    0x001AU,
    0x0122U,
    0x4102U,
    0x0124U,
    0x4100U,
    0x4101U,
    0x0020U,
    0x0021U,
    0x0022U,
    0x4004U,
    0x0024U,
    0x4002U,
    0x4001U,
    0x4000U,
    0x0028U,
    0x0110U,
    0x1800U,
    0x1801U,
    0x002CU,
    0x400AU,
    0x4009U,
    0x4008U,
    0x0030U,
    0x0108U,
    0x0240U,
    0x0241U,
    0x0034U,
    0x4012U,
    0x4011U,
    0x4010U,
    0x0101U,
    0x0100U,
    0x0103U,
    0x0102U,
    0x0105U,
    0x0104U,
    0x1401U,
    0x1400U,
    0x0040U,
    0x0041U,
    0x0042U,
    0x0043U,
    0x0044U,
    0x0045U,
    0x0046U,
    0x4060U,
    0x0048U,
    0x0049U,
    0x0301U,
    0x0300U,
    0x004CU,
    0x1600U,
    0x0305U,
    0x0304U,
    0x0050U,
    0x0051U,
    0x0220U,
    0x0221U,
    0x3000U,
    0x4200U,
    0x3002U,
    0x4202U,
    0x0058U,
    0x1082U,
    0x1081U,
    0x1080U,
    0x3008U,
    0x4208U,
    0x2820U,
    0x1084U,
    0x0060U,
    0x0061U,
    0x0210U,
    0x0211U,
    0x0480U,
    0x0481U,
    0x4041U,
    0x4040U,
    0x0068U,
    0x2402U,
    0x2401U,
    0x2400U,
    0x0488U,
    0x3100U,
    0x2810U,
    0x2404U,
    0x0202U,
    0x0880U,
    0x0200U,
    0x0201U,
    0x0206U,
    0x0884U,
    0x0204U,
    0x0205U,
    0x0141U,
    0x0140U,
    0x0208U,
    0x0209U,
    0x2802U,
    0x0144U,
    0x2800U,
    0x2801U,
    0x0080U,
    0x0081U,
    0x0082U,
    0x0A00U,
    0x0084U,
    0x0085U,
    0x2009U,
    0x2008U,
    0x0088U,
    0x0089U,
    0x2005U,
    0x2004U,
    0x2003U,
    0x2002U,
    0x2001U,
    0x2000U,
    0x0090U,
    0x0091U,
    0x0092U,
    0x1048U,
    0x0602U,
    0x0C80U,
    0x0600U,
    0x0601U,
    0x0098U,
    0x1042U,
    0x1041U,
    0x1040U,
    0x2013U,
    0x2012U,
    0x2011U,
    0x2010U,
    0x00A0U,
    0x00A1U,
    0x00A2U,
    0x4084U,
    0x0440U,
    0x0441U,
    0x4081U,
    0x4080U,
    0x6000U,
    0x1200U,
    0x6002U,
    0x1202U,
    0x6004U,
    0x2022U,
    0x2021U,
    0x2020U,
    0x0841U,
    0x0840U,
    0x2104U,
    0x0842U,
    0x2102U,
    0x0844U,
    0x2100U,
    0x2101U,
    0x0181U,
    0x0180U,
    0x0B00U,
    0x0182U,
    0x5040U,
    0x0184U,
    0x2108U,
    0x2030U,
    0x00C0U,
    0x00C1U,
    0x4401U,
    0x4400U,
    0x0420U,
    0x0421U,
    0x0422U,
    0x4404U,
    0x0900U,
    0x0901U,
    0x1011U,
    0x1010U,
    0x0904U,
    0x2042U,
    0x2041U,
    0x2040U,
    0x0821U,
    0x0820U,
    0x1009U,
    0x1008U,
    0x4802U,
    0x0824U,
    0x4800U,
    0x4801U,
    0x1003U,
    0x1002U,
    0x1001U,
    0x1000U,
    0x0501U,
    0x0500U,
    0x1005U,
    0x1004U,
    0x0404U,
    0x0810U,
    0x1100U,
    0x1101U,
    0x0400U,
    0x0401U,
    0x0402U,
    0x0403U,
    0x040CU,
    0x0818U,
    0x1108U,
    0x1030U,
    0x0408U,
    0x0409U,
    0x040AU,
    0x2060U,
    0x0801U,
    0x0800U,
    0x0280U,
    0x0802U,
    0x0410U,
    0x0804U,
    0x0412U,
    0x0806U,
    0x0809U,
    0x0808U,
    0x1021U,
    0x1020U,
    0x5000U,
    0x2200U,
    0x5002U,
    0x2202U,
};

constexpr uint32_t X14 = 0x00004000U;     // vector representation of X^14
constexpr uint32_t X8 = 0x00000100U;      // vector representation of X^8
constexpr uint32_t MASK7 = 0xFFFFFF00U;   // auxiliary vector for testing
constexpr uint32_t GENPOL = 0x00000139U;  // generator polynomial, g(x)

// Compute the syndrome corresponding to the given pattern -- the
// remainder after dividing it (as the vector representation of a
// polynomial) by the generator polynomial GENPOL.
uint32_t get_syndrome(uint32_t pattern) {
    uint32_t aux = X14;

    if (pattern >= X8) {
        while (pattern & MASK7) {
            while (!(aux & pattern))
                aux = aux >> 1;

            pattern ^= (aux / X8) * GENPOL;
        }
    }

    return pattern;
}

}  // namespace

uint8_t qr1676_decode(uint8_t emb_byte0, uint8_t emb_byte1) {
    uint32_t code = (static_cast<uint32_t>(emb_byte0) << 7) + (emb_byte1 >> 1);
    const uint32_t syndrome = get_syndrome(code);
    const uint32_t error_pattern = DECODING_TABLE_1576[syndrome];

    code ^= error_pattern;

    // NOTE: the OpenGD77 source this was ported from has `return code >>
    // 7` here, which is a genuine bug in that file -- verified by
    // round-tripping every one of the 128 possible info words through its
    // OWN ENCODING_TABLE_1676 and this decode logic: `>> 7` matches only
    // 1/128 (value 0, trivially), while `>> 8` matches 128/128 cleanly and
    // recovers correctly from EVERY possible single-bit flip (2048/2048).
    // This almost certainly went unnoticed upstream because OpenGD77's own
    // consumer (uiHotspot.c's getEmbeddedData()) calls CQR1676_decode()
    // and then discards its return value, reading the pre-correction raw
    // bits directly instead -- so the buggy shift was never actually
    // exercised in the field.
    return static_cast<uint8_t>(code >> 8);
}

}  // namespace dmr
