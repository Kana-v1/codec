#include "bitreader.h"

int8_t read_bit(BitReader* reader) {
    if (reader->byte_idx >= reader->data_len) {
        return -1;
    }

    const int8_t bit = (reader->data[reader->byte_idx] >> (7 - reader->bit_idx)) & 1;

    reader->bit_idx++;
    if (reader->bit_idx == 8) {
        reader->bit_idx = 0;
        reader->byte_idx++;
    }

    return bit;
}

int32_t read_bits(BitReader* reader, const uint32_t n) {
    int32_t bits = 0;

    for (uint32_t i = 0; i < n; i++) {
        const int8_t bit = read_bit(reader);
        if (bit < 0) {
            return -1;
        }

        bits = (bits << 1) | bit;
    }

    return bits;
}

void skip_byte(BitReader* reader) {
    // already aligned on a byte boundary: the next read starts a fresh byte anyway
    if (reader->bit_idx == 0) {
        return;
    }

    reader->bit_idx = 0;
    reader->byte_idx++;
}
