message("stm32l0xx.cmake")

set(CPU_PARAMETERS ${CPU_PARAMETERS}
    -mthumb
    -mcpu=cortex-m0plus
    -mfloat-abi=soft
)