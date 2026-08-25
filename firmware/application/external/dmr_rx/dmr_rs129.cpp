/*
 * GF(256) tables, generator, and the Berlekamp-Massey/Chien-search/Forney
 * decode algorithm below are ported verbatim from dmrshark's rs-12-9.c
 * (https://github.com/nonoo/dmrshark, libs/coding/rs-12-9.c). That file
 * carries no individual author name, only:
 *
 *   This file is part of dmrshark.
 *   dmrshark is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published
 *   by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 * dmrshark only offers GPLv3-or-later, not v2 -- unlike most other files
 * in this project, this file (and any combined work containing it) is
 * therefore bound by GPLv3-or-later terms specifically.
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Wrapper/integration code Copyright (C) 2026 Jarkko Vääräniemi.
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "dmr_rs129.hpp"

#include <cstring>

namespace dmr {
namespace {

constexpr int DATASIZE = 9;
constexpr int CHECKSUMSIZE = 3;
constexpr int POLY_MAXDEG = CHECKSUMSIZE * 2;  // 6

struct Poly {
    uint8_t data[POLY_MAXDEG]{};
};

// GF(256) exp/log tables -- ETSI TS 102 361-1 Annex B.3.6 / "DMR AI
// spec page 138" per dmrshark's own comment -- copied verbatim from
// dmrshark's rs-12-9.c (also used unmodified by dsd-fme).
constexpr uint8_t GALOIS_EXP[256] = {
    0x01,
    0x02,
    0x04,
    0x08,
    0x10,
    0x20,
    0x40,
    0x80,
    0x1D,
    0x3A,
    0x74,
    0xE8,
    0xCD,
    0x87,
    0x13,
    0x26,
    0x4C,
    0x98,
    0x2D,
    0x5A,
    0xB4,
    0x75,
    0xEA,
    0xC9,
    0x8F,
    0x03,
    0x06,
    0x0C,
    0x18,
    0x30,
    0x60,
    0xC0,
    0x9D,
    0x27,
    0x4E,
    0x9C,
    0x25,
    0x4A,
    0x94,
    0x35,
    0x6A,
    0xD4,
    0xB5,
    0x77,
    0xEE,
    0xC1,
    0x9F,
    0x23,
    0x46,
    0x8C,
    0x05,
    0x0A,
    0x14,
    0x28,
    0x50,
    0xA0,
    0x5D,
    0xBA,
    0x69,
    0xD2,
    0xB9,
    0x6F,
    0xDE,
    0xA1,
    0x5F,
    0xBE,
    0x61,
    0xC2,
    0x99,
    0x2F,
    0x5E,
    0xBC,
    0x65,
    0xCA,
    0x89,
    0x0F,
    0x1E,
    0x3C,
    0x78,
    0xF0,
    0xFD,
    0xE7,
    0xD3,
    0xBB,
    0x6B,
    0xD6,
    0xB1,
    0x7F,
    0xFE,
    0xE1,
    0xDF,
    0xA3,
    0x5B,
    0xB6,
    0x71,
    0xE2,
    0xD9,
    0xAF,
    0x43,
    0x86,
    0x11,
    0x22,
    0x44,
    0x88,
    0x0D,
    0x1A,
    0x34,
    0x68,
    0xD0,
    0xBD,
    0x67,
    0xCE,
    0x81,
    0x1F,
    0x3E,
    0x7C,
    0xF8,
    0xED,
    0xC7,
    0x93,
    0x3B,
    0x76,
    0xEC,
    0xC5,
    0x97,
    0x33,
    0x66,
    0xCC,
    0x85,
    0x17,
    0x2E,
    0x5C,
    0xB8,
    0x6D,
    0xDA,
    0xA9,
    0x4F,
    0x9E,
    0x21,
    0x42,
    0x84,
    0x15,
    0x2A,
    0x54,
    0xA8,
    0x4D,
    0x9A,
    0x29,
    0x52,
    0xA4,
    0x55,
    0xAA,
    0x49,
    0x92,
    0x39,
    0x72,
    0xE4,
    0xD5,
    0xB7,
    0x73,
    0xE6,
    0xD1,
    0xBF,
    0x63,
    0xC6,
    0x91,
    0x3F,
    0x7E,
    0xFC,
    0xE5,
    0xD7,
    0xB3,
    0x7B,
    0xF6,
    0xF1,
    0xFF,
    0xE3,
    0xDB,
    0xAB,
    0x4B,
    0x96,
    0x31,
    0x62,
    0xC4,
    0x95,
    0x37,
    0x6E,
    0xDC,
    0xA5,
    0x57,
    0xAE,
    0x41,
    0x82,
    0x19,
    0x32,
    0x64,
    0xC8,
    0x8D,
    0x07,
    0x0E,
    0x1C,
    0x38,
    0x70,
    0xE0,
    0xDD,
    0xA7,
    0x53,
    0xA6,
    0x51,
    0xA2,
    0x59,
    0xB2,
    0x79,
    0xF2,
    0xF9,
    0xEF,
    0xC3,
    0x9B,
    0x2B,
    0x56,
    0xAC,
    0x45,
    0x8A,
    0x09,
    0x12,
    0x24,
    0x48,
    0x90,
    0x3D,
    0x7A,
    0xF4,
    0xF5,
    0xF7,
    0xF3,
    0xFB,
    0xEB,
    0xCB,
    0x8B,
    0x0B,
    0x16,
    0x2C,
    0x58,
    0xB0,
    0x7D,
    0xFA,
    0xE9,
    0xCF,
    0x83,
    0x1B,
    0x36,
    0x6C,
    0xD8,
    0xAD,
    0x47,
    0x8E,
    0x01,
};

constexpr uint8_t GALOIS_LOG[256] = {
    0,
    0,
    1,
    25,
    2,
    50,
    26,
    198,
    3,
    223,
    51,
    238,
    27,
    104,
    199,
    75,
    4,
    100,
    224,
    14,
    52,
    141,
    239,
    129,
    28,
    193,
    105,
    248,
    200,
    8,
    76,
    113,
    5,
    138,
    101,
    47,
    225,
    36,
    15,
    33,
    53,
    147,
    142,
    218,
    240,
    18,
    130,
    69,
    29,
    181,
    194,
    125,
    106,
    39,
    249,
    185,
    201,
    154,
    9,
    120,
    77,
    228,
    114,
    166,
    6,
    191,
    139,
    98,
    102,
    221,
    48,
    253,
    226,
    152,
    37,
    179,
    16,
    145,
    34,
    136,
    54,
    208,
    148,
    206,
    143,
    150,
    219,
    189,
    241,
    210,
    19,
    92,
    131,
    56,
    70,
    64,
    30,
    66,
    182,
    163,
    195,
    72,
    126,
    110,
    107,
    58,
    40,
    84,
    250,
    133,
    186,
    61,
    202,
    94,
    155,
    159,
    10,
    21,
    121,
    43,
    78,
    212,
    229,
    172,
    115,
    243,
    167,
    87,
    7,
    112,
    192,
    247,
    140,
    128,
    99,
    13,
    103,
    74,
    222,
    237,
    49,
    197,
    254,
    24,
    227,
    165,
    153,
    119,
    38,
    184,
    180,
    124,
    17,
    68,
    146,
    217,
    35,
    32,
    137,
    46,
    55,
    63,
    209,
    91,
    149,
    188,
    207,
    205,
    144,
    135,
    151,
    178,
    220,
    252,
    190,
    97,
    242,
    86,
    211,
    171,
    20,
    42,
    93,
    158,
    132,
    60,
    57,
    83,
    71,
    109,
    65,
    162,
    31,
    45,
    67,
    216,
    183,
    123,
    164,
    118,
    196,
    23,
    73,
    236,
    127,
    12,
    111,
    246,
    108,
    161,
    59,
    82,
    41,
    157,
    85,
    170,
    251,
    96,
    134,
    177,
    187,
    204,
    62,
    90,
    203,
    89,
    95,
    176,
    156,
    169,
    160,
    81,
    11,
    245,
    22,
    235,
    122,
    117,
    44,
    215,
    79,
    174,
    213,
    233,
    230,
    231,
    173,
    232,
    116,
    214,
    244,
    234,
    168,
    80,
    88,
    175,
};

uint8_t galois_exp(uint8_t pos) {
    return GALOIS_EXP[pos];
}

uint8_t galois_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0) return 0;
    return GALOIS_EXP[(GALOIS_LOG[a] + GALOIS_LOG[b]) % 255];
}

uint8_t galois_inv(uint8_t elt) {
    return GALOIS_EXP[255 - GALOIS_LOG[elt]];
}

// Multiply by z (shift right by 1).
void multiply_poly_z(Poly& poly) {
    for (int i = POLY_MAXDEG - 1; i > 0; i--)
        poly.data[i] = poly.data[i - 1];
    poly.data[0] = 0;
}

void multiply_polys(uint8_t dst[POLY_MAXDEG * 2], const Poly& p1, const Poly& p2) {
    int8_t tmp1[POLY_MAXDEG * 2];

    for (int i = 0; i < POLY_MAXDEG * 2; i++) dst[i] = 0;

    for (int i = 0; i < POLY_MAXDEG; i++) {
        for (int j = POLY_MAXDEG; j < POLY_MAXDEG * 2; j++) tmp1[j] = 0;

        // Scale tmp1 by p1[i]
        for (int j = 0; j < POLY_MAXDEG; j++)
            tmp1[j] = static_cast<int8_t>(galois_mul(p2.data[j], p1.data[i]));

        // Shift (multiply) tmp1 right by i
        for (int j = POLY_MAXDEG * 2 - 1; j >= i; j--) tmp1[j] = tmp1[j - i];
        for (int j = 0; j < i; j++) tmp1[j] = 0;

        // Add into partial product.
        for (int j = 0; j < POLY_MAXDEG * 2; j++)
            dst[j] = static_cast<uint8_t>(dst[j] ^ static_cast<uint8_t>(tmp1[j]));
    }
}

// Computes the combined erasure/error evaluator polynomial
// (error_locator_poly*syndrome mod z^4).
void calc_error_evaluator_poly(const Poly& error_locator_poly, const Poly& syndrome, Poly& error_evaluator_poly) {
    uint8_t product[POLY_MAXDEG * 2];
    multiply_polys(product, error_locator_poly, syndrome);
    for (int i = 0; i < CHECKSUMSIZE; i++) error_evaluator_poly.data[i] = product[i];
    for (int i = CHECKSUMSIZE; i < POLY_MAXDEG; i++) error_evaluator_poly.data[i] = 0;
}

uint8_t compute_discrepancy(const Poly& error_locator_poly, const Poly& syndrome, uint8_t L, uint8_t n) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i <= L; i++)
        sum = static_cast<uint8_t>(sum ^ galois_mul(error_locator_poly.data[i], syndrome.data[n - i]));
    return sum;
}

// Berlekamp-Massey algorithm: finds the error locator polynomial, then
// derives the error evaluator polynomial. From Cain, Clark,
// "Error-Correction Coding For Digital Communications", pp. 216.
void bm_calculate(const Poly& syndrome, Poly& error_locator_poly, Poly& error_evaluator_poly) {
    uint8_t L = 0;
    int8_t k = -1;
    uint8_t psi2[POLY_MAXDEG];
    Poly D{};
    D.data[1] = 1;

    error_locator_poly = Poly{};
    error_locator_poly.data[0] = 1;

    for (uint8_t n = 0; n < CHECKSUMSIZE; n++) {
        const uint8_t d = compute_discrepancy(error_locator_poly, syndrome, L, n);

        if (d != 0) {
            for (int i = 0; i < POLY_MAXDEG; i++)
                psi2[i] = static_cast<uint8_t>(error_locator_poly.data[i] ^ galois_mul(d, D.data[i]));

            if (L < static_cast<uint8_t>(n - k)) {
                const uint8_t L2 = static_cast<uint8_t>(n - k);
                k = static_cast<int8_t>(n - L);
                for (int i = 0; i < POLY_MAXDEG; i++)
                    D.data[i] = galois_mul(error_locator_poly.data[i], galois_inv(d));
                L = L2;
            }

            for (int i = 0; i < POLY_MAXDEG; i++) error_locator_poly.data[i] = psi2[i];
        }

        multiply_poly_z(D);
    }

    calc_error_evaluator_poly(error_locator_poly, syndrome, error_evaluator_poly);
}

struct Roots {
    uint8_t error_locations[256]{};
    uint8_t errors_num{0};
};

// Chien search: the error-locator polynomial's roots are the values of
// alpha^n where evaluating the polynomial yields zero.
Roots find_roots(const Poly& error_locator_poly) {
    Roots roots{};
    for (uint16_t r = 1; r < 256; r++) {
        uint8_t sum = 0;
        for (uint8_t k = 0; k < CHECKSUMSIZE + 1; k++)
            sum = static_cast<uint8_t>(sum ^ galois_mul(galois_exp(static_cast<uint8_t>((k * r) % 255)), error_locator_poly.data[k]));
        if (sum == 0) roots.error_locations[roots.errors_num++] = static_cast<uint8_t>(255 - r);
    }
    return roots;
}

void calc_syndrome(const uint8_t codeword[DATASIZE + CHECKSUMSIZE], Poly& syndrome) {
    syndrome.data[0] = syndrome.data[1] = syndrome.data[2] = 0;
    for (int j = 0; j < CHECKSUMSIZE; j++) {
        for (int i = 0; i < DATASIZE + CHECKSUMSIZE; i++)
            syndrome.data[j] = static_cast<uint8_t>(codeword[i] ^ galois_mul(galois_exp(static_cast<uint8_t>(j + 1)), syndrome.data[j]));
    }
}

bool syndrome_has_errors(const Poly& syndrome) {
    return syndrome.data[0] != 0 || syndrome.data[1] != 0 || syndrome.data[2] != 0;
}

// Forney's algorithm (error-evaluator equation, Cain/Clark pp. 207):
// evaluates error_evaluator_poly/error_locator_poly' at each root to
// get the error magnitude, then XORs it into the codeword at that
// byte position. Returns false if any root points outside the
// codeword (uncorrectable -- more errors than this code can locate).
//
// Deliberate deviation from a literal port here: dmrshark's own
// ComputeAndCorrectFullLinkControlCrc() treats correct_errors()
// returning "NO_ERRORS_FOUND" (roots.errors_num==0) as CRC-valid even
// though it's only ever called after already confirming the syndrome
// IS non-zero -- i.e. it treats "we know there's an error but Chien
// search found no consistent error-location hypothesis for it" as
// success. That's backwards: a non-zero syndrome guarantees at least
// one real error exists (by the syndrome's own definition), so finding
// no roots means the error pattern doesn't fit ANY hypothesis this
// code can represent -- textbook uncorrectable, not valid. Since this
// project's goal is rejecting corrupted decodes (the whole reason this
// file exists), treat it as uncorrectable rather than faithfully
// reproducing what looks like a real mistake in the reference.
bool correct_errors(uint8_t codeword[DATASIZE + CHECKSUMSIZE], const Poly& syndrome, uint8_t& errors_found) {
    Poly error_locator_poly{};
    Poly error_evaluator_poly{};
    bm_calculate(syndrome, error_locator_poly, error_evaluator_poly);

    const Roots roots = find_roots(error_locator_poly);
    errors_found = roots.errors_num;

    if (roots.errors_num == 0) return false;  // syndrome was nonzero but no roots -- uncorrectable
    if (roots.errors_num > CHECKSUMSIZE) return false;

    for (int r = 0; r < roots.errors_num; r++) {
        if (roots.error_locations[r] >= DATASIZE + CHECKSUMSIZE) return false;
    }

    for (int r = 0; r < roots.errors_num; r++) {
        const uint8_t i = roots.error_locations[r];

        uint8_t num = 0;
        for (int j = 0; j < POLY_MAXDEG; j++)
            num = static_cast<uint8_t>(num ^ galois_mul(error_evaluator_poly.data[j], galois_exp(static_cast<uint8_t>(((255 - i) * j) % 255))));

        uint8_t denom = 0;
        for (int j = 1; j < POLY_MAXDEG; j += 2)
            denom = static_cast<uint8_t>(denom ^ galois_mul(error_locator_poly.data[j], galois_exp(static_cast<uint8_t>(((255 - i) * (j - 1)) % 255))));

        const uint8_t err = galois_mul(num, galois_inv(denom));
        codeword[DATASIZE + CHECKSUMSIZE - i - 1] = static_cast<uint8_t>(codeword[DATASIZE + CHECKSUMSIZE - i - 1] ^ err);
    }
    return true;
}

}  // namespace

bool rs129_correct(uint8_t codeword[12], uint32_t mask) {
    uint8_t work[DATASIZE + CHECKSUMSIZE];
    std::memcpy(work, codeword, sizeof(work));
    work[9] = static_cast<uint8_t>(work[9] ^ ((mask >> 16) & 0xFF));
    work[10] = static_cast<uint8_t>(work[10] ^ ((mask >> 8) & 0xFF));
    work[11] = static_cast<uint8_t>(work[11] ^ (mask & 0xFF));

    Poly syndrome{};
    calc_syndrome(work, syndrome);

    if (!syndrome_has_errors(syndrome)) return true;  // valid as-is, nothing to correct

    uint8_t errors_found = 0;
    if (!correct_errors(work, syndrome, errors_found)) return false;

    // correct_errors() modifies `work` (still mask-XORed) in place --
    // undo the mask before handing back to the caller.
    work[9] = static_cast<uint8_t>(work[9] ^ ((mask >> 16) & 0xFF));
    work[10] = static_cast<uint8_t>(work[10] ^ ((mask >> 8) & 0xFF));
    work[11] = static_cast<uint8_t>(work[11] ^ (mask & 0xFF));
    std::memcpy(codeword, work, sizeof(work));
    return true;
}

}  // namespace dmr
