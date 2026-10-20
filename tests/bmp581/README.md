# BMP581 driver tests

Run from the repository root with a host C compiler (no Pico hardware required):

```sh
cc -std=c11 -Wall -Wextra -Werror -Itests/bmp581/stubs -Ilib/bmp581 tests/bmp581/test_bmp581.c lib/bmp581/bmp581.c -o bmp581_test
./bmp581_test
```

The I2C mock checks register encodings, configuration after reinitialization,
invalid parameters, short I2C writes, NORMAL mode OSR fallback, and data-ready
timeouts. It does not verify physical sensor behavior or actual sample rates.
