add_library(sdcard STATIC
    sdcard.c
)

target_include_directories(sdcard PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(sdcard
    pico_stdlib
    hardware_spi
    hardware_i2c
)
