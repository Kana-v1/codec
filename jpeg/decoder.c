#include "jpg.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../pkg/bitreader/bitreader.h"

static bool read_markers(FILE* file, Header* header, unsigned char markers[MARKERS_LEN]);
static void read_appn(FILE* file, Header* header);

static void read_quantization_table(FILE* file, Header* header);
static void read_start_of_frame(FILE* file, Header* header);
static void read_restart_interval(FILE* file, Header* header);
static void read_huffman_table(FILE* file, Header* header);
static void read_start_of_scan(FILE* file, Header* header);
static void read_comment(FILE* file, Header* header);
static void skip_marker(FILE* file, Header* header);
static bool validate_header_info(const Header* header);

static void gen_codes(HuffmanTable* table);
static bool decode_mcu_component(
    BitReader* bit_reader,
    MCU* mcu,
    uint8_t color_component_idx,
    int32_t* prev_dcs,
    const HuffmanTable* ac_table,
    const HuffmanTable* dc_table
);
static uint8_t get_next_symbol(BitReader* reader, const HuffmanTable* table);
static int32_t* get_mcu_component(MCU* mcu, uint8_t color_component_idx);

static int8_t dequantize_mcu_component(QuantizationTable* qtable, MCU* mcu, uint8_t color_component_idx);
static int8_t inverse_dct_component(MCU* mcu, uint8_t color_component_idx);
static void YCbCr_to_RGB_MCU(const Header* header, MCU* mcu, MCU* cbcr, uint32_t v, uint32_t h);

static uint8_t read_len(FILE* file, int64_t* out);
static uint8_t read_bytes(FILE* file, unsigned char* out, uint8_t size);

static int write_int(FILE* file, int32_t v);
static int write_short(FILE* file, int16_t v);

Header* read_jpg(const char* filename) {
    Header* header = {0};

    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("failed to open file\n");
        return 0;
    }

    header = calloc(1, sizeof(Header));
    if (!header) {
        goto exit_OK;
    }

    header->huffman_data = calloc(1, sizeof(Vector));

    unsigned char markers[MARKERS_LEN] = {0};
    fread(&markers, sizeof(markers), 1, file);

    if (markers[0] == 0xFF && markers[1] == SOI) {
        header->valid = true;
    }

    bool read_next = true;
    while (header->valid && read_next) {
        read_next = read_markers(file, header, markers);
    }

exit_OK:
    if (file) fclose(file);
    return header;
}

void dispose_header(Header* header) {
    vector_dispose(header->huffman_data);
    free(header);
}

void print_header(const Header* header) {
    if (header == 0) return;

    // Quantization tables
    {
        printf("\n============DQT============\n");
        for (uint8_t i = 0; i < QUAN_TABLES_N; i++) {
            if (!header->quantization_tables[i].set) {
                continue;
            }

            printf("Table ID: %d\n", i);
            printf("Table Data:");

            for (uint8_t j = 0; j < QUAN_VALUES_N; j++) {
                if (j % (uint8_t)sqrt(QUAN_VALUES_N) == 0) {
                    printf("\n");
                }

                printf("%d ", header->quantization_tables[i].table[j]);
            }

            printf("\n\n");
        }
    }

    // Frame markers
    {
        printf("\n============SOF============\n");
        printf("Frame Type: 0x%X\n", header->frame_type);
        printf("Height: %ld\n", header->height);
        printf("Width: %ld\n", header->width);
        printf("Color components:\n");
        for (uint8_t i = 0; i < header->num_components; i++) {
            printf("Component ID: %d\n", i + 1);
            printf("Horizontal Sampling Factor: %d\n", (uint8_t)header->color_components[i].horizontal_sampling_factor);
            printf("Vertical Sampling Factor: %d\n", (uint8_t)header->color_components[i].vertical_sampling_factor);
            printf("Quantization Table ID: %d\n", (uint8_t)header->color_components[i].quantization_table_id);
            printf("\n");
        }
    }

    // Huffman tables
    {
        printf("\n============DHT============\n");
        printf("DC Tables: \n");
        for (uint8_t i = 0; i < HUFFMAN_TABLES_N; i++) {
            const HuffmanTable htable = header->huffman_dc_tables[i];
            if (!htable.set) {
                continue;
            }

            printf("Table ID: %d\n", i);
            printf("Symbols:\n");
            for (uint8_t j = 0; j < HUFFMAN_MAX_N_OF_CODES; j++) {
                printf("%d: ", j + 1);
                for (uint8_t k = htable.offsets[j]; k < htable.offsets[j + 1]; k++) {
                    printf("%x ", htable.symbols[k]);
                }

                printf("\n");
            }
        }

        printf("AC Tables: \n");
        for (uint8_t i = 0; i < HUFFMAN_TABLES_N; i++) {
            const HuffmanTable htable = header->huffman_ac_tables[i];
            if (!htable.set) {
                continue;
            }

            printf("Table ID: %d\n", i);
            printf("Symbols:\n");
            for (uint8_t j = 0; j < HUFFMAN_MAX_N_OF_CODES; j++) {
                printf("%d:", j + 1);
                for (uint8_t k = htable.offsets[j]; k < htable.offsets[j + 1]; k++) {
                    printf("%x ", htable.symbols[k]);
                }

                printf("\n");
            }
        }
    }

    // SOS
    {
        printf("\n============SOS============\n");
        printf("Start of Selection: %d\n", header->start_of_selection);
        printf("End of Selection: %d\n", header->end_of_selection);
        printf("Successive Approximation High: %d\n", header->successive_approximation_high);
        printf("Successive Approximation Low: %d\n", header->successive_approximation_low);
        printf("Color components:\n");
        for (uint8_t i = 0; i < header->num_components; i++) {
            const ColorComponent* cur_component = &header->color_components[i];
            printf("Component ID: %d\n", i + 1);
            printf("Huffman DC Table ID: %d\n", cur_component->huffman_dc_table_id);
            printf("Huffman AC Table ID: %d\n", cur_component->huffman_ac_table_id);
        }

        printf("Length of Huffman data: %ld\n", header->huffman_data->len);
    }

    // Restart interval
    {
        printf("\n============DRI============\n");
        printf("Restart interval: %ld\n", header->restart_interval);
    }


    printf("\n\n");
}

MCU* decode_huffman_data(Header* const header) {
    uint32_t mcu_height = header->mcu_height_real;
    uint32_t mcu_width = header->mcu_width_real;

    MCU* mcus = calloc((size_t)mcu_width * mcu_height, sizeof(MCU));
    if (!mcus) {
        return 0;
    }

    for (uint8_t i = 0; i < HUFFMAN_TABLES_N; i++) {
        gen_codes(&header->huffman_dc_tables[i]);
        gen_codes(&header->huffman_ac_tables[i]);
    }

    BitReader* bit_reader = calloc(1, sizeof(BitReader));
    if (!bit_reader) {
        goto exit_ERR;
    }

    bit_reader->data = header->huffman_data->data;
    bit_reader->data_len = header->huffman_data->len;

    int32_t prev_DCs[COLORS_N] = {0};

    mcu_height = header->mcu_height;
    mcu_width = header->mcu_width;

    int64_t restart_interval = header->restart_interval * header->horizontal_sampling_factor * header->vertical_sampling_factor; // convert from block number measurement to the "units of MCUs"

    uint64_t mcu_count = 0;
    for (uint32_t y = 0; y < mcu_height; y += header->vertical_sampling_factor) {
        for (uint32_t x = 0; x < mcu_width; x += header->horizontal_sampling_factor) {
            if (mcu_count != 0 && restart_interval != 0 && mcu_count % restart_interval == 0) {
                for (uint8_t j = 0; j < COLORS_N; j++) {
                    prev_DCs[j] = 0;
                }

                skip_byte(bit_reader);
            }

            // read Y1  Y2  Y3  Y4  Cb  Cr(for 4:2:0 subsampling)
            for (uint8_t i = 0; i < header->num_components; i++) {
                for (uint32_t v = 0; v < header->color_components[i].vertical_sampling_factor; v++) {
                    for (uint32_t h = 0; h < header->color_components[i].horizontal_sampling_factor; h++) {
                        const uint32_t pos = (y + v) * header->mcu_width_real + (x + h);

                        if (!decode_mcu_component(
                            bit_reader,
                            &mcus[pos],
                            i,
                            prev_DCs,
                            &header->huffman_ac_tables[header->color_components[i].huffman_ac_table_id],
                            &header->huffman_dc_tables[header->color_components[i].huffman_dc_table_id]
                        )) {
                            goto exit_ERR;
                        }
                    }
                }
            }

            mcu_count++;
        }
    }

exit:
    free(bit_reader);
    return mcus;

exit_ERR:
    free(mcus);
    mcus = 0;
    goto exit;
}

int dequantize(Header* header, MCU* mcus) {
    for (uint32_t y = 0; y < header->mcu_height; y += header->vertical_sampling_factor) {
        for (uint32_t x = 0; x < header->mcu_width; x += header->horizontal_sampling_factor) {
            for (uint8_t i = 0; i < header->num_components; i++) {
                for (uint32_t v = 0; v < header->color_components[i].vertical_sampling_factor; v++) {
                    for (uint32_t h = 0; h < header->color_components[i].horizontal_sampling_factor; h++) {
                        const uint32_t pos = (y + v) * header->mcu_width_real + (x + h);

                        if (dequantize_mcu_component(
                                &header->quantization_tables[header->color_components[i].quantization_table_id],
                                &mcus[pos],
                                i
                            ) < 0) {
                            return 1;
                        }
                    }
                }
            }
        }
    }

    return 0;
}

int inverse_dct(const Header* header, MCU* mcus) {
    for (uint32_t y = 0; y < header->mcu_height; y += header->vertical_sampling_factor) {
        for (uint32_t x = 0; x < header->mcu_width; x += header->horizontal_sampling_factor) {
            for (uint8_t i = 0; i < header->num_components; i++) {
                for (uint32_t v = 0; v < header->color_components[i].vertical_sampling_factor; v++) {
                    for (uint32_t h = 0; h < header->color_components[i].horizontal_sampling_factor; h++) {
                        const uint32_t pos = (y + v) * header->mcu_width_real + (x + h);

                        if (inverse_dct_component(&mcus[pos], i) < 0) {
                            return -1;
                        }
                    }
                }
            }
        }
    }

    return 0;
}

void YCbCr_to_RGB(const Header* header, MCU* mcus) {
    for (uint32_t y = 0; y < header->mcu_height; y += header->vertical_sampling_factor) {
        for (uint32_t x = 0; x < header->mcu_width; x += header->horizontal_sampling_factor) {
            MCU* cbcr = &mcus[y * header->mcu_width_real + x];

            for (int32_t v = header->vertical_sampling_factor - 1; v >= 0; v--) {
                for (int32_t h = header->horizontal_sampling_factor - 1; h >= 0; h--) {
                    const uint32_t pos = (y + v) * header->mcu_width_real + (x + h);
                    MCU* mcu = &mcus[pos];

                    YCbCr_to_RGB_MCU(header, mcu, cbcr, v, h);
                }
            }
        }
    }
}

int write_bitmap(const Header* const header, const MCU* const mcus, const char* filename) {
    FILE* file = fopen(filename, "wb");
    if (!file) {
        printf("failed to open file\n");
        return 1;
    }

    int res = 0;

    const uint32_t padding_size = header->width % BMP_ROW_ALIGNMENT;
    const uint32_t file_size = 14 + // first part of the header
                               12 + // second part of the header
                               header->height * header->width * COLORS_N + // size of pixel array
                               padding_size * header->height; // total number of padding bytes

    // write metadata
    {
        if (!fwrite("BM", sizeof(char), 2, file)) {
            goto exit_ERR;
        }

        if (write_int(file, (int32_t)file_size)) {
            goto exit_ERR;
        }

        // unused values
        if (write_int(file, 0)) {
            goto exit_ERR;
        }

        // start addr of the pixel array
        if (write_int(file, 0x1A)) {
            goto exit_ERR;
        }

        // DAB header size
        if (write_int(file, 12)) {
            goto exit_ERR;
        }

        if (write_short(file, (int16_t)header->width)) {
            goto exit_ERR;
        }

        if (write_short(file, (int16_t)header->height)) {
            goto exit_ERR;
        }

        // number of plains
        if (write_short(file, 1)) {
            goto exit_ERR;
        }

        // number of bits per pixel
        if (write_short(file, 24)) {
            goto exit_ERR;
        }
    }

    for (int64_t y = header->height - 1; y >= 0; y--) {
        const uint64_t mcu_row = y / MCU_W_H;
        const uint64_t pixel_row = y % MCU_W_H;

        for (uint64_t x = 0; x < header->width; x++) {
            const uint64_t mcu_column = x / MCU_W_H;
            const uint64_t pixel_column = x % MCU_W_H;

            const uint64_t mcu_idx = mcu_row * header->mcu_width_real + mcu_column;
            const uint64_t pixel_idx = pixel_row * MCU_W_H + pixel_column;

            const uint8_t bgr[3] = {
                (uint8_t)mcus[mcu_idx].b[pixel_idx],
                (uint8_t)mcus[mcu_idx].g[pixel_idx],
                (uint8_t)mcus[mcu_idx].r[pixel_idx],
            };
            if (!fwrite(bgr, sizeof(uint8_t), 3, file)) {
                goto exit_ERR;
            }
        }

        // add padding
        const uint8_t pad[BMP_ROW_ALIGNMENT] = {0};
        if (padding_size && fwrite(pad, 1, padding_size, file) != padding_size) {
            goto exit_ERR;
        }
    }

    goto exit;


exit_ERR:
    res = 1;

exit:
    fclose(file);
    return res;
}


static bool read_markers(FILE* file, Header* header, unsigned char markers[MARKERS_LEN]) {
    if (!fread(markers, MARKERS_LEN, 1, file)) {
        printf("failed to read %d bytes from the file; EOF\n", MARKERS_LEN);
        goto exit_ERR;
    }

no_read_start:
    if (markers[0] != 0xFF) {
        goto exit_ERR;
    }

    if (markers[1] >= APP0 && markers[1] <= APP15) {
        read_appn(file, header);
        goto exit_OK;
    }

    // markers to skip
    if ((markers[1] > JPG0 && markers[1] < JPG13) ||
        markers[1] == DNL ||
        markers[1] == DHP ||
        markers[1] == EXP) {
        skip_marker(file, header);
        goto exit_OK;
    }

    // SOF0 (baseline) is the only frame type supported; reject the rest
    if ((markers[1] >= SOF1 && markers[1] <= SOF3) ||
        (markers[1] >= SOF5 && markers[1] <= SOF7) ||
        (markers[1] >= SOF9 && markers[1] <= SOF11) ||
        (markers[1] >= SOF13 && markers[1] <= SOF15)) {
        printf("SOF marker not supported: 0x%x\n", markers[1]);
        goto exit_ERR;
    }

    if (markers[1] >= RST0 && markers[1] <= RST7) {
        printf("RSTN detected before SOS\n");
        goto exit_ERR;
    }

    switch (markers[1]) {
        case DQT:
            read_quantization_table(file, header);
            goto exit_OK;

        case SOF0:
            header->frame_type = SOF0;
            read_start_of_frame(file, header);
            goto exit_OK;

        case DRI:
            read_restart_interval(file, header);
            goto exit_OK;

        case DHT:
            read_huffman_table(file, header);
            goto exit_OK;

        case SOS:
            read_start_of_scan(file, header);
            break;

        case COM:
            read_comment(file, header);
            goto exit_OK;

        case TEM: // TEM has no size
            goto exit_OK;

        case IGN:
            // just replace marker with the new value. Upper half (markers[0]) remains the same
            if (!read_bytes(file, &markers[1], 1)) {
                goto exit_ERR;
            }
            goto no_read_start;

        case SOI:
            printf("embedded JPGs are not supported\n");
            goto exit_ERR;

        case EOI:
            printf("EOI occurred before SOI\n");
            goto exit_ERR;

        case DAC:
            printf("arithmetic coding is not supported\n");
            goto exit_ERR;


        default:
            printf("unknown marker: %x\n", markers[1]);
            goto exit_ERR;
    }

    // after SOS
    if (!read_bytes(file, &markers[1], 1)) {
        goto exit_ERR;
    }

    // read compressed image data
    while (true) {
        markers[0] = markers[1];
        if (!read_bytes(file, &markers[1], 1)) {
            goto exit_ERR;
        }

        // if marker is found
        if (markers[0] == IGN) {
            if (markers[1] == EOI) {
                // the whole file is parsed, nothing left to read
                return false;
            }

            // 0xFF00 means put a literal 0xFF in image data and ignore 0x00
            if (markers[1] == 0x00) {
                if (vector_push(header->huffman_data, markers[0])) {
                    goto exit_ERR;
                }
                // overwrite 0x00 with next byte
                if (!read_bytes(file, &markers[1], 1)) {
                    goto exit_ERR;
                }
                continue;
            }

            // restart marker
            if (markers[1] >= RST0 && markers[1] <= RST7) {
                // overwrite marker with next byte
                if (!read_bytes(file, &markers[1], 1)) {
                    goto exit_ERR;
                }

                continue;
            }

            // ignore multiple 0xFF's in a row
            if (markers[1] == IGN) {
                // do nothing
                continue;
            }

            // here markers[0] == 0xFF but markers[1] is invalid marker value
            printf("invalid marker during compressed data scan: 0x%x\n", markers[1]);
            goto exit_ERR;
        }

        // markers[0] is actual data
        if (vector_push(header->huffman_data, markers[0])) {
            goto exit_ERR;
        }
    }

    if (!validate_header_info(header)) {
        goto exit_ERR;
    }


exit_ERR:
    header->valid = false;
    return false;

exit_OK:
    return true;
}

static void read_appn(FILE* file, Header* header) {
    printf("reading appn markers\n");

    int64_t len = 0;
    if (!read_len(file, &len) || len < 0) {
        header->valid = false;
        return;
    }

    fseek(file, len - MARKERS_LEN, SEEK_CUR);
}

static void read_quantization_table(FILE* file, Header* header) {
    printf("reading quantization table\n");

    int64_t len = 0;
    if (!read_len(file, &len) || len < 0) {
        header->valid = false;
        return;
    }

    len -= MARKERS_LEN;

    while (len > 0) {
        unsigned char tableInfo = 0;
        if (!read_bytes(file, &tableInfo, 1)) {
            header->valid = false;
            return;
        }

        len -= 1;
        const unsigned char table_id = tableInfo & 0x0F;
        const unsigned char precision = tableInfo >> 4;

        if (table_id > 3) {
            printf("invalid quantization table ID: %d\n", (uint8_t)(table_id));
            header->valid = false;
            break;
        }

        QuantizationTable* curQuanTable = &header->quantization_tables[table_id];
        curQuanTable->set = true;

        if (precision != 0) {
            int64_t tmp = 0;
            for (uint8_t i = 0; i < QUAN_VALUES_N; i++) {
                if (!read_len(file, &tmp) || tmp < 0) {
                    header->valid = false;
                    return;
                }

                curQuanTable->table[zigZagMap[i]] = tmp;
            }

            len -= QUAN_VALUES_N * 2;

            continue;
        }

        unsigned char tmp;
        for (uint8_t i = 0; i < QUAN_VALUES_N; i++) {
            if (!read_bytes(file, &tmp, 1)) {
                header->valid = false;
                return;
            }

            curQuanTable->table[zigZagMap[i]] = tmp;
        }

        len -= QUAN_VALUES_N;
    }

    if (len) {
        printf("DQT invalid\n");
        header->valid = false;
    }
}

static void read_start_of_frame(FILE* file, Header* header) {
    printf("reading start of frame\n");

    if (header->num_components != 0) {
        printf("Multiple SOF detected\n");
        goto exit_ERR;
    }

    int64_t len = 0;
    if (!read_len(file, &len) || len < 0) {
        goto exit_ERR;
    }

    unsigned char precision = 0;
    if (!read_bytes(file, &precision, 1)) {
        goto exit_ERR;
    }

    if (precision != 8) {
        printf("Invalid precision %d\n", precision);
        goto exit_ERR;
    }

    if (!read_len(file, &header->height)) {
        goto exit_ERR;
    }

    if (!read_len(file, &header->width)) {
        goto exit_ERR;
    }

    header->mcu_height = (header->height + MCU_W_H - 1) / MCU_W_H; // ceil(header->height / MCU_H)
    header->mcu_width = (header->width + MCU_W_H - 1) / MCU_W_H; // ceil(header->width / MCU_W)
    header->mcu_height_real = header->mcu_height;
    header->mcu_width_real = header->mcu_width;

    if (!read_bytes(file, &header->num_components, 1)) {
        goto exit_ERR;
    }

    if (header->num_components == 4 || header->num_components == 0) {
        printf("unsupported number of components %d\n", header->num_components);
        goto exit_ERR;
    }

    for (uint8_t i = 0; i < header->num_components; i++) {
        unsigned char component_id = 0;
        if (!read_bytes(file, &component_id, 1)) {
            goto exit_ERR;
        }

        if (component_id == 0) {
            header->zero_based = true;
        }

        if (header->zero_based) {
            component_id++;
        }

        if (component_id == 4 || component_id == 5) {
            printf("YIQ color mode is not supported\n");
            goto exit_ERR;
        }

        // component id starts from 1, not from 0
        if (component_id == 0 || component_id > 3) {
            printf("invalid component id %d\n", component_id);
            goto exit_ERR;
        }

        ColorComponent* component = &header->color_components[component_id - 1];
        if (component->used) {
            printf("duplicate color component ID\n");
            goto exit_ERR;
        }

        component->used = true;
        unsigned char sampling_factor = 0;
        if (!read_bytes(file, &sampling_factor, 1)) {
            goto exit_ERR;
        }

        component->horizontal_sampling_factor = sampling_factor >> 4;
        component->vertical_sampling_factor = sampling_factor & 0x0F;

        if (component_id == 1) {
            // luminance channel
            if ((component->horizontal_sampling_factor != 1 && component->horizontal_sampling_factor != 2) ||
                (component->vertical_sampling_factor != 1 && component->vertical_sampling_factor != 2)) {
                printf("Sampling factors not supported\n");
                goto exit_ERR;
            }

            if (component->horizontal_sampling_factor == 2 && header->mcu_width % 2 == 1) {
                header->mcu_width_real++;
            }

            if (component->vertical_sampling_factor == 2 && header->mcu_height % 2 == 1) {
                header->mcu_height_real++;
            }

            header->horizontal_sampling_factor = component->horizontal_sampling_factor;
            header->vertical_sampling_factor = component->vertical_sampling_factor;
        } else {
            // chrominance channels
            if (component->horizontal_sampling_factor != 1 || component->vertical_sampling_factor != 1) {
                printf("Sampling factors not supported\n");
                goto exit_ERR;
            }
        }


        if (!read_bytes(file, &component->quantization_table_id, 1)) {
            goto exit_ERR;
        }
        if (component->quantization_table_id > 3) {
            printf("invalid quantization table ID in frame components\n");
            goto exit_ERR;
        }
    }

    // 8 - number of bytes we read from the start (precision, num_components, etc)
    if (len - 8 - (COLORS_N * header->num_components) != 0) {
        printf("SOF invalid\n");
        goto exit_ERR;
    }

    return;

exit_ERR:
    header->valid = false;
}

static void read_restart_interval(FILE* file, Header* header) {
    printf("reading restart interval\n");

    int64_t len = 0;
    if (!read_len(file, &len) || len < 0) {
        goto exit_ERR;
    }

    if (!read_len(file, &header->restart_interval)) {
        goto exit_ERR;
    }

    if (len - 4 != 0) {
        printf("DRI is invalid\n");
        goto exit_ERR;
    }

    return;

exit_ERR:
    header->valid = false;
}

static void read_huffman_table(FILE* file, Header* header) {
    printf("reading huffman table\n");

    int64_t len = 0;
    if (!read_len(file, &len) || len < 0) {
        goto exit_ERR;
    }

    len -= MARKERS_LEN;

    unsigned char table_info = 0;
    unsigned char table_id = 0;
    bool is_ac_table = false;

    while (len > 0) {
        if (!read_bytes(file, &table_info, 1)) {
            goto exit_ERR;
        }

        table_id = table_info & 0x0F;
        is_ac_table = table_info >> 4;

        if (table_id > HUFFMAN_TABLES_N - 1) {
            printf("invalid table id %d\n", table_id);
            goto exit_ERR;
        }

        HuffmanTable* htable = 0;
        if (is_ac_table) {
            htable = &header->huffman_ac_tables[table_id];
        } else {
            htable = &header->huffman_dc_tables[table_id];
        }

        htable->set = true;

        unsigned char symbols_num[HUFFMAN_MAX_N_OF_CODES] = {0};
        if (!read_bytes(file, symbols_num, HUFFMAN_MAX_N_OF_CODES)) {
            goto exit_ERR;
        }

        uint64_t cur_symbol_start_idx = 0;
        for (uint8_t i = 0; i < HUFFMAN_MAX_N_OF_CODES; i++) {
            htable->offsets[i] = cur_symbol_start_idx;
            cur_symbol_start_idx += symbols_num[i];
        }
        htable->offsets[HUFFMAN_MAX_N_OF_CODES] = cur_symbol_start_idx;

        if (cur_symbol_start_idx > HUFFMAN_AC_TABLE_SYMBOLS_N) {
            printf("too many symbols in Huffman table\n");
            goto exit_ERR;
        }

        for (uint64_t i = 0; i < cur_symbol_start_idx; i++) {
            if (!read_bytes(file, &htable->symbols[i], 1)) {
                goto exit_ERR;
            }
        }

        len -= 1 + HUFFMAN_MAX_N_OF_CODES + (int64_t)cur_symbol_start_idx; // 1 byte for table info
    }

    if (len != 0) {
        printf("DHT invalid\n");
        goto exit_ERR;
    }

    return;

exit_ERR:
    header->valid = false;
}

static void read_start_of_scan(FILE* file, Header* header) {
    printf("reading start of scan\n");

    if (header->num_components == 0) {
        printf("SOS detected before the SOF\n");
        goto exit_ERR;
    }

    int64_t len = 0;
    if (!read_len(file, &len) || len < 0) {
        goto exit_ERR;
    }

    for (uint8_t i = 0; i < COLORS_N; i++) {
        header->color_components[i].used = false;
    }

    unsigned char num_components = 0;
    if (!read_bytes(file, &num_components, 1)) {
        goto exit_ERR;
    }

    for (uint8_t i = 0; i < num_components; i++) {
        unsigned char component_id = 0;
        if (!read_bytes(file, &component_id, 1)) {
            goto exit_ERR;
        }

        if (header->zero_based) {
            component_id++;
        }

        if (component_id > header->num_components || component_id < 1) {
            printf("invalid color component ID: %d\n", component_id);
            goto exit_ERR;
        }

        ColorComponent* component = &header->color_components[component_id - 1];
        if (component->used) {
            printf("duplicate color component\n");
            goto exit_ERR;
        }

        component->used = true;

        unsigned char huffman_table_ids = 0;
        if (!read_bytes(file, &huffman_table_ids, 1)) {
            goto exit_ERR;
        }

        component->huffman_dc_table_id = huffman_table_ids >> 4;
        component->huffman_ac_table_id = huffman_table_ids & 0x0F;

        if (component->huffman_ac_table_id >= HUFFMAN_TABLES_N) {
            printf("invalid huffman ac table id: %d\n", component->huffman_ac_table_id);
            goto exit_ERR;
        }
        if (component->huffman_dc_table_id >= HUFFMAN_TABLES_N) {
            printf("invalid huffman dc table id: %d\n", component->huffman_dc_table_id);
            goto exit_ERR;
        }
    }

    unsigned char remaining_fields[3] = {0};
    if (!read_bytes(file, remaining_fields, 3)) {
        goto exit_ERR;
    }

    header->start_of_selection = remaining_fields[0];
    header->end_of_selection = remaining_fields[1];
    header->successive_approximation_high = remaining_fields[2] >> 4;
    header->successive_approximation_low = remaining_fields[2] & 0x0F;

    if (header->start_of_selection != 0 || header->end_of_selection != 63) {
        printf("invalid spectral selection\n");
        goto exit_ERR;
    }

    if (header->successive_approximation_high != 0 || header->successive_approximation_low != 0) {
        printf("invalid successive approximation\n");
        goto exit_ERR;
    }

    if (len - 6 - (2 * num_components) != 0) {
        printf("SOS invalid\n");
        goto exit_ERR;
    }

    return;

exit_ERR:
    header->valid = false;
}

static void read_comment(FILE* file, Header* header) {
    printf("reading COM Marker\n");

    int64_t len = 0;
    if (!read_len(file, &len) || len < 0) {
        header->valid = false;
        return;
    }

    fseek(file, len - MARKERS_LEN, SEEK_CUR);
}

static void skip_marker(FILE* file, Header* header) {
    read_comment(file, header);
}

static bool validate_header_info(const Header* header) {
    if (header->num_components != COLORS_N) {
        printf("%d color components given (1 or 3 required)\n", header->num_components);
        return false;
    }

    for (uint8_t i = 0; i < header->num_components; i++) {
        const ColorComponent* cur_color_component = &header->color_components[i];

        if (!header->quantization_tables[cur_color_component->quantization_table_id].set) {
            printf("color component using uninitialized quantization table\n");
            return false;
        }

        if (!header->huffman_ac_tables[cur_color_component->huffman_ac_table_id].set) {
            printf("color component using uninitialized ac Huffman table\n");
            return false;
        }

        if (!header->huffman_dc_tables[cur_color_component->huffman_dc_table_id].set) {
            printf("color component using uninitialized dc Huffman table\n");
            return false;
        }
    }

    return true;
}

static void gen_codes(HuffmanTable* table) {
    if (!table->set) {
        return;
    }

    uint16_t code = 0;

    for (uint8_t i = 0; i < HUFFMAN_MAX_N_OF_CODES; i++) {
        // i == current_code_length - 1
        for (uint8_t j = table->offsets[i]; j < table->offsets[i + 1]; j++) {
            table->codes[j] = code;
            code += 1;
        }

        code <<= 1; // append 0 to the right of the code candidate
    }
}

static bool decode_mcu_component(
    BitReader* bit_reader,
    MCU* mcu,
    const uint8_t color_component_idx,
    int32_t* prev_dcs,
    const HuffmanTable* ac_table,
    const HuffmanTable* dc_table
) {
    int32_t* color_component = get_mcu_component(mcu, color_component_idx);
    if (!color_component) {
        return false;
    }

    // write down DC coefficient
    {
        const uint8_t dc_coef_len = get_next_symbol(bit_reader, dc_table);
        if (dc_coef_len == 0xFF) {
            printf("invalid DC value\n");
            return false;
        }
        if (dc_coef_len >= HUFFMAN_DC_TABLE_SYMBOLS_N) {
            printf("DC coefficient length is too large\n");
            return false;
        }

        int32_t dc_coef = read_bits(bit_reader, dc_coef_len);
        if (dc_coef < 0) {
            printf("invalid DC value\n");
            return false;
        }

        // in JPEG when bits are starting with '0' (000, 001, etc) it means it's negative value. 001 = -6; 100 = 4;
        // check MSB == 0 (whether coef < 2^(L - 1))?
        if (dc_coef_len != 0 && dc_coef < (1 << (dc_coef_len - 1))) {
            dc_coef -= (1 << dc_coef_len) - 1; // bottom half (MSB = 0) is the negative values; stored offset by 2^L - 1
        }

        // block's average brightness
        color_component[0] = dc_coef + prev_dcs[color_component_idx];
        prev_dcs[color_component_idx] = color_component[0];
    }

    // write down AC coefficient
    {
        for (uint8_t i = 1; i < MCU_VALUES_N;) {
            const uint8_t ac_symbol = get_next_symbol(bit_reader, ac_table);
            if (ac_symbol == 0xFF) {
                printf("invalid AC value\n");
                return false;
            }

            // 0x00 means fill remainer of component with 0
            if (ac_symbol == 0x00) {
                for (; i < MCU_VALUES_N; i++) {
                    color_component[zigZagMap[i]] = 0;
                }

                break;
            }

            // read next component coefficient
            uint8_t ac_num_zeroes = ac_symbol >> 4;
            const uint8_t ac_coef_len = ac_symbol & 0x0F;
            int32_t ac_coef = 0;

            // 0xF0 symbol means skip 16 0's
            if (ac_symbol == 0xF0) {
                ac_num_zeroes = 16;
            }

            if (i + ac_num_zeroes > MCU_VALUES_N) {
                printf("Zero run-length exceeded MCU\n");
                return false;
            }

            for (uint8_t j = 0; j < ac_num_zeroes; j++, i++) {
                color_component[zigZagMap[i]] = 0;
            }

            if (ac_coef_len > HUFFMAN_AC_TABLE_COEFFICIENTS_N) {
                printf("AC coefficient length is too big\n");
                return false;
            }

            if (ac_coef_len == 0) {
                continue;
            }

            // a coefficient follows the run, so there has to be a slot left for it
            if (i >= MCU_VALUES_N) {
                printf("AC coefficient exceeded MCU\n");
                return false;
            }

            ac_coef = read_bits(bit_reader, ac_coef_len);
            if (ac_coef < 0) {
                printf("invalid AC value\n");
                return false;
            }

            // in JPEG when bits are starting with '0' (000, 001, etc) it means it's negative value. 001 = -6; 100 = 4;
            if (ac_coef < (1 << (ac_coef_len - 1))) {
                // MSB == 0 (whether coef < 2^(L - 1))?
                ac_coef -= (1 << ac_coef_len) - 1;
                // bottom half (MSB = 0) is the negative values; stored offset by 2^L - 1
            }

            color_component[zigZagMap[i++]] = ac_coef;
        }
    }

    return true;
}

static uint8_t get_next_symbol(BitReader* reader, const HuffmanTable* table) {
    uint16_t current_code = 0;
    for (uint8_t i = 0; i < HUFFMAN_MAX_N_OF_CODES; i++) {
        const int8_t bit = read_bit(reader);
        if (bit < 0) {
            goto exit_err;
        }

        current_code = (current_code << 1) | bit;
        for (uint8_t j = table->offsets[i]; j < table->offsets[i + 1]; j++) {
            if (current_code == table->codes[j]) {
                return table->symbols[j];
            }
        }
    }

exit_err:
    return 0xFF; // 0xFF is not a valid value
}

static int32_t* get_mcu_component(MCU* mcu, const uint8_t color_component_idx) {
    switch (color_component_idx) {
        case 0:
            return mcu->r;
        case 1:
            return mcu->g;
        case 2:
            return mcu->b;
        default:
            return 0;
    }
}

static int8_t dequantize_mcu_component(QuantizationTable* qtable, MCU* mcu, const uint8_t color_component_idx) {
    int32_t* color_component = get_mcu_component(mcu, color_component_idx);
    if (color_component == 0) {
        return -1;
    }

    for (uint8_t i = 0; i < MCU_VALUES_N; i++) {
        color_component[i] *= qtable->table[i];
    }

    return 0;
}

static int8_t inverse_dct_component(MCU* mcu, const uint8_t color_component_idx) {
    int32_t* color_component = get_mcu_component(mcu, color_component_idx);
    if (color_component == 0) {
        return -1;
    }

    // AAN algorithm
    for (uint8_t col = 0; col < MCU_W_H; col++) {
        const float g[8] = {
            color_component[0 * MCU_W_H + col] * IDCT_S0,
            color_component[4 * MCU_W_H + col] * IDCT_S4,
            color_component[2 * MCU_W_H + col] * IDCT_S2,
            color_component[6 * MCU_W_H + col] * IDCT_S6,
            color_component[5 * MCU_W_H + col] * IDCT_S5,
            color_component[1 * MCU_W_H + col] * IDCT_S1,
            color_component[7 * MCU_W_H + col] * IDCT_S7,
            color_component[3 * MCU_W_H + col] * IDCT_S3,
        };

        const float f[8] = {
            g[0],
            g[1],
            g[2],
            g[3],
            g[4] - g[7],
            g[5] + g[6],
            g[5] - g[6],
            g[4] + g[7],
        };

        const float e[9] = {
            f[0],
            f[1],
            f[2] - f[3],
            f[2] + f[3],
            f[4],
            f[5] - f[7],
            f[6],
            f[5] + f[7],
            f[4] + f[6],
        };

        const float d[9] = {
            e[0],
            e[1],
            e[2] * IDCT_M1,
            e[3],
            e[4] * IDCT_M2,
            e[5] * IDCT_M3,
            e[6] * IDCT_M4,
            e[7],
            e[8] * IDCT_M5,
        };

        const float c[9] = {
            d[0] + d[1],
            d[0] - d[1],
            d[2] - d[3],
            d[3],
            d[4] + d[8],
            d[5] + d[7],
            d[6] - d[8],
            d[7],
            (d[5] + d[7]) - (d[6] - d[8]), // c[5] - c[6]
        };

        const float b[9] = {
            c[0] + c[3],
            c[1] + c[2],
            c[1] - c[2],
            c[0] - c[3],
            c[4] - c[8],
            c[8],
            c[6] - c[7],
            c[7]
        };

        color_component[0 * 8 + col] = b[0] + b[7];
        color_component[1 * 8 + col] = b[1] + b[6];
        color_component[2 * 8 + col] = b[2] + b[5];
        color_component[3 * 8 + col] = b[3] + b[4];
        color_component[4 * 8 + col] = b[3] - b[4];
        color_component[5 * 8 + col] = b[2] - b[5];
        color_component[6 * 8 + col] = b[1] - b[6];
        color_component[7 * 8 + col] = b[0] - b[7];
    }

    for (uint8_t row = 0; row < MCU_W_H; row++) {
        const float g[8] = {
            color_component[row * MCU_W_H + 0] * IDCT_S0,
            color_component[row * MCU_W_H + 4] * IDCT_S4,
            color_component[row * MCU_W_H + 2] * IDCT_S2,
            color_component[row * MCU_W_H + 6] * IDCT_S6,
            color_component[row * MCU_W_H + 5] * IDCT_S5,
            color_component[row * MCU_W_H + 1] * IDCT_S1,
            color_component[row * MCU_W_H + 7] * IDCT_S7,
            color_component[row * MCU_W_H + 3] * IDCT_S3,
        };

        const float f[8] = {
            g[0],
            g[1],
            g[2],
            g[3],
            g[4] - g[7],
            g[5] + g[6],
            g[5] - g[6],
            g[4] + g[7],
        };

        const float e[9] = {
            f[0],
            f[1],
            f[2] - f[3],
            f[2] + f[3],
            f[4],
            f[5] - f[7],
            f[6],
            f[5] + f[7],
            f[4] + f[6],
        };

        const float d[9] = {
            e[0],
            e[1],
            e[2] * IDCT_M1,
            e[3],
            e[4] * IDCT_M2,
            e[5] * IDCT_M3,
            e[6] * IDCT_M4,
            e[7],
            e[8] * IDCT_M5,
        };

        const float c[9] = {
            d[0] + d[1],
            d[0] - d[1],
            d[2] - d[3],
            d[3],
            d[4] + d[8],
            d[5] + d[7],
            d[6] - d[8],
            d[7],
            (d[5] + d[7]) - (d[6] - d[8]), // c[5] - c[6]
        };

        const float b[9] = {
            c[0] + c[3],
            c[1] + c[2],
            c[1] - c[2],
            c[0] - c[3],
            c[4] - c[8],
            c[8],
            c[6] - c[7],
            c[7]
        };

        color_component[row * MCU_W_H + 0] = b[0] + b[7];
        color_component[row * MCU_W_H + 1] = b[1] + b[6];
        color_component[row * MCU_W_H + 2] = b[2] + b[5];
        color_component[row * MCU_W_H + 3] = b[3] + b[4];
        color_component[row * MCU_W_H + 4] = b[3] - b[4];
        color_component[row * MCU_W_H + 5] = b[2] - b[5];
        color_component[row * MCU_W_H + 6] = b[1] - b[6];
        color_component[row * MCU_W_H + 7] = b[0] - b[7];
    }

    return 0;
}

// basic and straightforward version of inverting dct
static int8_t inverse_dct_component_slow(MCU* mcu, const uint8_t color_component_idx) {
    int32_t* color_component = get_mcu_component(mcu, color_component_idx);
    if (color_component == 0) {
        return -1;
    }

    int32_t result[MCU_VALUES_N] = {0};

    // loop through all mcu values
    for (uint8_t y = 0; y < MCU_W_H; y++) {
        for (uint8_t x = 0; x < MCU_W_H; x++) {
            double sum = 0.0;
            // loop through each pixel in mcu
            for (uint8_t v = 0; v < MCU_W_H; v++) {
                for (uint8_t u = 0; u < MCU_W_H; u++) {
                    const double cv = v == 0 ? 1.0 / sqrt(2) : 1.0;
                    const double cu = u == 0 ? 1.0 / sqrt(2) : 1.0;

                    sum += cv * cu * color_component[v * MCU_W_H + u] *
                            cos((2 * x + 1) * u * M_PI / 16) *
                            cos((2 * y + 1) * v * M_PI / 16);
                }
            }

            sum /= 4;
            result[y * MCU_W_H + x] = (int32_t)sum;
        }
    }

    for (uint8_t i = 0; i < MCU_VALUES_N; i++) {
        color_component[i] = result[i];
    }

    return 0;
}

static void YCbCr_to_RGB_MCU(const Header* header, MCU* mcu, MCU* cbcr, uint32_t v, uint32_t h) {
    for (int8_t y = MCU_W_H - 1; y >= 0; y--) {
        for (int8_t x = MCU_W_H - 1; x >= 0; x--) {
            const uint8_t pixel = y * MCU_W_H + x;

            const uint8_t cbcr_pixel_row = y / header->vertical_sampling_factor + // make y to be [0; 3] instead of [0; 7]
                                           4 * v; // make y [4; 7] for non zero v to "stretch" color across all cells
            const uint8_t cbcr_pixel_col = x / header->horizontal_sampling_factor +
                                           4 * h;
            const uint8_t cbcr_pixel = cbcr_pixel_row * MCU_W_H + cbcr_pixel_col;

            // + 128 because after inverse dct YCbCr values end up in the range [-128; 127] instead of [0; 255]
            // we could add 128 during inverse dct but it would require different consts
            int32_t rgb[COLORS_N] = {
                mcu->y[pixel] + 1.402f * cbcr->cr[cbcr_pixel] + 128, // r
                mcu->y[pixel] - 0.344f * cbcr->cb[cbcr_pixel] - 0.714f * cbcr->cr[cbcr_pixel] + 128, // g
                mcu->y[pixel] + 1.772f * cbcr->cb[cbcr_pixel] + 128, // b
            };

            for (uint8_t j = 0; j < COLORS_N; j++) {
                if (rgb[j] < 0) rgb[j] = 0;
                if (rgb[j] > 255) rgb[j] = 255;
            }

            mcu->r[pixel] = rgb[0];
            mcu->g[pixel] = rgb[1];
            mcu->b[pixel] = rgb[2];
        }
    }
}

static uint8_t read_len(FILE* file, int64_t* out) {
    unsigned char length[MARKERS_LEN] = {0};
    if (!read_bytes(file, length, MARKERS_LEN)) {
        return 0;
    }

    *out = (int64_t)((length[0] << 8) + length[1]); // (0x01 << 8) + 0x2C = 0x0100 + 0x2C = 0x012C = 300

    return 1;
}

static uint8_t read_bytes(FILE* file, unsigned char* out, const uint8_t size) {
    if (!fread(out, size, 1, file)) {
        printf("failed to read %d bytes from the file\n", size);
        return 0;
    }

    return 1;
}

// write 4-byte integer in little-endian
static int write_int(FILE* file, const int32_t v) {
    const uint8_t bytes[4] = {
        (v >> 0) & 0xFF,
        (v >> 8) & 0xFF,
        (v >> 16) & 0xFF,
        (v >> 24) & 0xFF,
    };

    if (fwrite(bytes, 1, sizeof(bytes), file) != sizeof(bytes)) {
        return 1;
    }

    return 0;
}

// write 2-byte integer in little-endian
static int write_short(FILE* file, const int16_t v) {
    const uint8_t bytes[2] = {
        (v >> 0) & 0xFF,
        (v >> 8) & 0xFF,
    };

    if (fwrite(bytes, 1, sizeof(bytes), file) != sizeof(bytes)) {
        return 1;
    }

    return 0;
}
