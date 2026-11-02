# JPEG decoder in C

A baseline JPEG decoder written from scratch in C11. It has no dependencies beyond the C standard library and libm. It reads a `.jpg` file and writes the decoded image as a 24-bit `.bmp`

## Build and run

Requires CMake 3.10+ and a C11 compiler.

```sh
cmake -B build
cmake --build build
./bin/main
```
