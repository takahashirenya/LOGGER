# BMP581 oversampling

`LOGGER.c` defines `bmp581_config`, shared by startup and error recovery.
Pressure and temperature OSR can each be selected from 1x to 128x using
`BMP581_OSR_P_*` / `BMP581_OSR_T_*` constants (register codes, not multipliers).

The default is pressure **16x**, temperature **1x**, CONTINUOUS mode. Bosch lists
about **87 Hz typical** for this combination; the logger still targets 50 Hz.
The NORMAL-mode ODR field is ignored in CONTINUOUS mode.

For highest resolution, change the configuration to:

```c
static const bmp581_config_t bmp581_config = {
    .osr_t = BMP581_OSR_T_8,
    .osr_p = BMP581_OSR_P_128,
    .odr_code = BMP581_ODR_CODE_100_299_HZ, // ignored in CONTINUOUS
    .pwr_mode = BMP581_PWR_MODE_CONTINUOUS,
};
```

Bosch lists about **12 Hz typical** at pressure 128x / temperature 8x. This is
not 480 Hz. Since `bmp581_main()` waits for fresh data in the shared sensor
loop, this option also slows the overall loop. Change the acquisition design
if other sensors must continue to be sampled at 50 Hz independently.

`bmp581_init()` retains its original pressure 4x / temperature 4x / NORMAL
100.299 Hz settings for existing callers. New callers use
`bmp581_init_with_config()`. NORMAL mode checks the effective OSR register and
returns false if the chip silently falls back for an invalid OSR/ODR setting.
The data-ready wait accepts slower high-OSR conversions, up to 250 ms.

These rates are datasheet typical values, not measured firmware performance.
See Bosch BMP581 datasheet revision 1.13, sections 4.3 and 4.4, Table 9:
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf
