px4_add_module(
    MODULE drivers__chiusura_bracci
    MAIN esp32_stick_i2c
    SRCS esp32_stick_i2c.cpp
    DEPENDS
        px4_platform
)