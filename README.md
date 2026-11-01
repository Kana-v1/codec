# JPEG decoder in C

A baseline JPEG decoder written from scratch in C11. It has no dependencies beyond the C standard library and libm. It reads a `.jpg` file and writes the decoded image as a 24-bit `.bmp`.

## How it works

1. **Parse the markers.** Reads quantization tables (DQT), Huffman tables (DHT), the frame header (SOF0), the scan header (SOS), the restart interval (DRI), comments and APPn segments, and validates them.
2. **Decode the entropy-coded data.** Builds canonical Huffman codes from the DHT code-length counts, then decodes the scan bit by bit into 8×8 blocks (MCUs). Handles byte stuffing (`0xFF00`) and restart markers.
3. **Dequantize.** Multiplies each coefficient by its quantization table entry; coefficients are placed back in natural order using the zig-zag table.
4. **Inverse DCT.** Uses the AAN fast algorithm. A direct, slower reference implementation is kept in the code for comparison.
5. **Colour conversion and output.** Converts YCbCr to RGB and writes a BMP, padding each row to a multiple of 4 bytes as the format requires.

## Supported input

- Baseline JPEG (SOF0), 8-bit precision
- Grayscale (1 component) or YCbCr colour (3 components)
- No chroma subsampling: every component must have 1×1 sampling factors (4:4:4)

Progressive, lossless, arithmetic-coded and 4-component (CMYK) files are rejected with an error message.

## Build and run

Requires CMake 3.10+ and a C11 compiler.

```sh
cmake -B build
cmake --build build
./bin/main
```

Set the input file in `main.c` (`JPG_FILE_PATH`) before building. The decoded image is written next to it with a `.bmp` extension.

## Layout

- `main.c`: runs the decoding pipeline for each input file
- `jpeg/decoder.c`, `jpeg/jpg.h`: marker parsing, Huffman decoding, dequantization, IDCT, colour conversion and BMP output
- `jpeg/const.c`: zig-zag order table
- `pkg/bitreader`: reads the entropy-coded data bit by bit
- `pkg/vector`: small dynamic array used to collect the scan data
