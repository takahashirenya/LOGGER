add_library( mcp2515 STATIC
    mcp2515.c
)

target_include_directories(mcp2515 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(mcp2515
    hardware_spi
    hardware_gpio
    pico_stdlib
)