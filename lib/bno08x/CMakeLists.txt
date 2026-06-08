add_library(bno08x
    bno08x.c
)

target_include_directories(bno08x PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(bno08x
    pico_stdlib
    hardware_i2c
)