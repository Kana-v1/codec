cmake_minimum_required(VERSION 3.10)
project(encoder C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_executable(main main.c
        jpeg/decoder.c
        jpeg/const.c
        pkg/bitreader/bitreader.c
        pkg/bitreader/bitreader.h
        pkg/vector/vector.c
        pkg/vector/vector.h
)

target_link_libraries(main m) # libm, for cos/sqrt
