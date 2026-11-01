#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jpeg/jpg.h"

static const char* const JPG_FILE_PATH = "";

int main() {
    if (JPG_FILE_PATH[0] == '\0') {
        printf("Set JPG_FILE_PATH in main.c to the JPEG file you want to decode\n");
        return 1;
    }

    const char* jpg_files[] = {JPG_FILE_PATH};
    for (size_t i = 0; i < sizeof(jpg_files) / sizeof(jpg_files[0]); i++) {
        Header* header = read_jpg(jpg_files[i]);
        if (!header) {
            printf("Could not read jpg file\n");
            continue;
        }

        if (!header->valid) {
            printf("JPEG file %s is invalid\n", jpg_files[i]);
            goto clear;
        }

        // print_header(header);

        MCU* mcus = decode_huffman_data(header);
        if (!mcus) {
            goto clear;
        }

        if (dequantize(header, mcus)) {
            goto clear;
        }

        if (inverse_dct(header, mcus) < 0) {
            goto clear;
        }

        YCbCr_to_RGB(header, mcus);

        // write bitmap to the .bmp file
        {
            const char* dot_pos = strrchr(jpg_files[i], '.');
            const size_t file_name_len = dot_pos ? (size_t)(dot_pos - jpg_files[i]) : strlen(jpg_files[i]);

            char* bmp_file_name = malloc(file_name_len + 5); // ".bmp" + '\0'
            if (!bmp_file_name) {
                goto clear;
            }

            memcpy(bmp_file_name, jpg_files[i], file_name_len);
            strcpy(bmp_file_name + file_name_len, ".bmp");

            if (write_bitmap(header, mcus, bmp_file_name)) {
                printf("failed to write bitmap");
                goto clear;
            }

            free(bmp_file_name);
        }


        free(mcus);

    clear:
        dispose_header(header);
    }

    return 0;
}
