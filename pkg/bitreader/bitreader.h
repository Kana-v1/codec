#ifndef ENCODER_BITREADER_H
#define ENCODER_BITREADER_H
#include <stdint.h>

typedef struct  {
    uint64_t byte_idx; // byte the next read comes from
    uint8_t bit_idx;   // bit within that byte, 0 = most significant

    uint8_t* data;
    uint64_t data_len;
} BitReader;

int8_t read_bit(BitReader* reader);
int32_t read_bits(BitReader* reader, uint32_t n);
// if there are bits remaining, advance to the 0th bit of the next byte
void skip_byte(BitReader* reader);

#endif //ENCODER_BITREADER_H
