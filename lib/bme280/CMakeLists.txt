add_library(bme280 STATIC
    bme280.c
)

target_include_directories(bme280 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(bme280
    hardware_i2c
    pico_stdlib
)