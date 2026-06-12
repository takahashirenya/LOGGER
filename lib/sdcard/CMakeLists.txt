add_library(sdcard STATIC
    sdcard.c
    ${PICO_SDK_PATH}/lib/tinyusb/lib/fatfs/source/ff.c
    ${PICO_SDK_PATH}/lib/tinyusb/lib/fatfs/source/ffsystem.c
    ${PICO_SDK_PATH}/lib/tinyusb/lib/fatfs/source/ffunicode.c
)

target_include_directories(sdcard PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/tinyusb/lib/fatfs/source
)

target_link_libraries(sdcard
    pico_stdlib
    hardware_spi
    hardware_i2c
)
