"""Defines for M4 targets."""

# STM32Cube defines for the M4 core.
M4_STM_DEFINES = [
    "STM32H755xx",
    "USE_HAL_DRIVER",
    "CORE_CM4",
    "USE_HAL_I2C_REGISTER_CALLBACKS=0",
    "USE_HAL_HRTIM_REGISTER_CALLBACKS=0",
    'STM32CUBE_HEADER=\\"stm32h7xx.h\\"',
]
