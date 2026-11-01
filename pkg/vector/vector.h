//
// Created by maxim on 8/16/26.
//

#ifndef ENCODER_VECTOR_H
#define ENCODER_VECTOR_H
#include <stdint.h>


typedef struct {
    uint8_t* data;

    uint64_t len;
    uint64_t cap;
} Vector;

uint8_t vector_push(Vector* v, uint8_t data);
void vector_dispose(Vector* v);

#endif //ENCODER_VECTOR_H
