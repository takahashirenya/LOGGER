# lib/bmp581/CMakeLists.txt
add_library(bmp581 STATIC
    bmp581.c
)

# このライブラリを使う人にヘッダを公開
target_include_directories(bmp581 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# ドライバ内で I2C を使うなら（pico-sdk の i2c API）
target_link_libraries(bmp581
    hardware_i2c
    pico_stdlib      # 必要に応じて
)
