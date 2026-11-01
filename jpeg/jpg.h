#ifndef JPG_H
#define JPG_H

#include <stdbool.h>
#include <stdint.h>
#include "../pkg/vector/vector.h"

#define MARKERS_LEN 2

// Start of Frame markers, non-differential, Huffman coding
#define SOF0 0xC0 // Baseline DCT
#define SOF1 0xC1 // Extended sequential DCT
#define SOF2 0xC2 // Progressive DCT
#define SOF3 0xC3 // Lossless (sequential)

// Start of Frame markers, differential, Huffman coding
#define SOF5 0xC5 // Differential sequential DCT
#define SOF6 0xC6 // Differential progressive DCT
#define SOF7 0xC7 // Differential lossless (sequential)

// Start of Frame markers, non-differential, arithmetic coding
#define SOF9 0xC9 // Extended sequential DCT
#define SOF10 0xCA // Progressive DCT
#define SOF11 0xCB // Lossless (sequential)

// Start of Frame markers, differential, arithmetic coding
#define SOF13 0xCD // Differential sequential DCT
#define SOF14 0xCE // Differential progressive DCT
#define SOF15 0xCF // Differential lossless (sequential)

// Restart interval Markers
#define RST0 0xD0
#define RST1 0xD1
#define RST2 0xD2
#define RST3 0xD3
#define RST4 0xD4
#define RST5 0xD5
#define RST6 0xD6
#define RST7 0xD7

// Other Markers
#define SOI 0xD8 // start of image marker
#define DQT 0xDB // Define quantization table
#define DRI 0xDD // Define restart interval
#define DHT 0xC4 // Define Huffman Table(s)
#define SOS 0xDA // start of scan marker
#define EOI 0xD9 // end of image
#define DAC 0xCC // arithmetic coding


// Misc Markers
#define COM 0xFE // comment
#define DNL 0xDC // Define Number of Lines
#define DHP 0xDE // Define Hierarchical Progression
#define EXP 0xDF // Expand Reference Component(s)
#define JPG0 0xF0
#define JPG13 0xFD
#define TEM 0x01
#define IGN 0xFF // ignore


// APPN Markers
#define APP0 0xE0
#define APP15 0xEF

#define QUAN_VALUES_N 64 // quantization values
#define QUAN_TABLES_N 4  // quantization tables
#define COLORS_N 3 // number of color channels

#define HUFFMAN_TABLES_N 4
#define HUFFMAN_MAX_N_OF_CODES 16  // number of codes of each length (might be less if encoder didn't use some symbols)
#define HUFFMAN_DC_TABLE_SYMBOLS_N 12
#define HUFFMAN_AC_TABLE_SYMBOLS_N 162
#define HUFFMAN_AC_TABLE_COEFFICIENTS_N 10

#define MCU_W_H 8 // MCU's width and height
#define MCU_VALUES_N MCU_W_H * MCU_W_H

#define BMP_ROW_ALIGNMENT 4   // BMP spec: each scan line is DWORD-aligned (each row's byte length must come out to a multiple of 4)


// inverse dct factors
#define IDCT_M0 (2.0 * cos(1.0 / 16.0 * 2.0 * M_PI))
#define IDCT_M1 (2.0 * cos(2.0 / 16.0 * 2.0 * M_PI))
#define IDCT_M3 IDCT_M1
#define IDCT_M5 (2.0 * cos(3.0 / 16.0 * 2.0 * M_PI))
#define IDCT_M2 (IDCT_M0 - IDCT_M5)
#define IDCT_M4 (IDCT_M0 + IDCT_M5)

#define IDCT_S0 (cos(0.0 / 16.0 * M_PI) / sqrt(8.0))
#define IDCT_S1 (cos(1.0 / 16.0 * M_PI) / 2.0)
#define IDCT_S2 (cos(2.0 / 16.0 * M_PI) / 2.0)
#define IDCT_S3 (cos(3.0 / 16.0 * M_PI) / 2.0)
#define IDCT_S4 (cos(4.0 / 16.0 * M_PI) / 2.0)
#define IDCT_S5 (cos(5.0 / 16.0 * M_PI) / 2.0)
#define IDCT_S6 (cos(6.0 / 16.0 * M_PI) / 2.0)
#define IDCT_S7 (cos(7.0 / 16.0 * M_PI) / 2.0)


extern const uint8_t zigZagMap[];


typedef struct QuantizationTable {
    uint8_t table[QUAN_VALUES_N];
    bool set;
} QuantizationTable;

typedef struct ColorComponent {
    uint8_t horizontal_sampling_factor;
    uint8_t vertical_sampling_factor;
    uint8_t quantization_table_id;
    uint8_t huffman_dc_table_id;
    uint8_t huffman_ac_table_id;

    bool used;
} ColorComponent;

/*
 * Canonical Huffman table, stored close to the way DHT transmits it.
 *
 * DHT(Define Huffman Table marker) gives 16 counts (how many codes are 1 bit long, 2 bits, ... 16 bits),
 * followed by a flat list of symbols ordered by increasing code length.
 * symbols[] is that flat list; offsets[] records where each length's run starts:
 *
 *     symbols[offsets[i] .. offsets[i + 1] - 1]
 *
 * are the symbols whose code is (i + 1) bits long. offsets has one extra entry
 * so the final run has an end marker -- offsets[HUFFMAN_MAX_N_OF_CODES] is the
 * total symbol count. A run can be empty (no codes of that length), and the
 * symbols array is usually only partly filled, which is why the end of a run
 * cannot be inferred from the array size.
 *
 * codes[] holds the actual bit patterns. They are never transmitted: the counts
 * alone determine them (see gen_codes). Within one length the codes are
 * consecutive integers; moving to the next length appends a 0 bit, i.e. shifts
 * left by one. For counts {0,1,5,1,...} and symbols 0,1,2,...:
 *
 *     len 2:  symbol 0     -> 00
 *     len 3:  symbols 1-5  -> 010 011 100 101 110
 *     len 4:  symbol 6     -> 1110
 *
 * Decoding reads one bit at a time and, after i + 1 bits, compares against just
 * the codes of that length; the prefix property means at most one can match.
 *
 * Codes are up to 16 bits wide, so codes[] must be wider than a byte. offsets[]
 * and symbols[] are indices and symbol values, both of which fit in a byte.
 */
typedef struct HuffmanTable {
    uint8_t offsets[HUFFMAN_MAX_N_OF_CODES + 1];
    uint8_t symbols[HUFFMAN_AC_TABLE_SYMBOLS_N]; // this will cover both ac and dc usage
    uint16_t codes[HUFFMAN_AC_TABLE_SYMBOLS_N];

    bool set;
} HuffmanTable;

typedef struct Header {
    bool valid;

    QuantizationTable quantization_tables[QUAN_TABLES_N];


    uint8_t frame_type;
    int64_t height;
    int64_t width;
    uint8_t num_components;
    bool zero_based;

    ColorComponent color_components[COLORS_N];

    int64_t restart_interval;

    HuffmanTable huffman_dc_tables[HUFFMAN_TABLES_N];
    HuffmanTable huffman_ac_tables[HUFFMAN_TABLES_N];

    uint8_t start_of_selection;
    uint8_t end_of_selection;
    uint8_t successive_approximation_high;
    uint8_t successive_approximation_low;

    Vector* huffman_data;
} Header;

typedef struct MCU {
    union {
        int32_t y[MCU_VALUES_N];
        int32_t r[MCU_VALUES_N];
    };

    union {
        int32_t cb[MCU_VALUES_N];
        int32_t g[MCU_VALUES_N];
    };

    union {
        int32_t cr[MCU_VALUES_N];
        int32_t b[MCU_VALUES_N];
    };
} MCU;


Header* read_jpg(const char* filename);
void dispose_header(Header* header);
void print_header(const Header* header);
MCU* decode_huffman_data(Header* header);
int dequantize(Header* header, MCU* mcus);
int inverse_dct(const Header* header, MCU* mcus);
void YCbCr_to_RGB(const Header* header, MCU* mcus);
int write_bitmap(const Header* header, const MCU* mcus, const char* filename);

#endif
