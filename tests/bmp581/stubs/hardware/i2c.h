#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct i2c_inst { int unused; } i2c_inst_t;
int i2c_write_blocking(i2c_inst_t *, uint8_t, const uint8_t *, size_t, bool);
int i2c_read_blocking(i2c_inst_t *, uint8_t, uint8_t *, size_t, bool);
