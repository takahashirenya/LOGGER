add_library(SDP610 STATIC
    SDP610.c
)

target_include_directories(SDP610 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(SDP610
    hardware_i2c
    pico_stdlib
    m
)
