//
// Created by maxim on 8/16/26.
//

#include "vector.h"

#include <stdlib.h>

uint8_t vector_push(Vector* v, const uint8_t data) {
    if (v->len == v->cap) {
        const uint64_t new_cap = v->cap ? v->cap * 2 : 8;

        uint8_t* new_arr = realloc(v->data, new_cap * sizeof(*v->data));
        if (!new_arr) {
            return 1;
        }

        v->cap = new_cap;
        v->data = new_arr;
    }

    v->data[v->len++] = data;

    return 0;
}

void vector_dispose(Vector* v) {
    free(v->data);
    free(v);
}
