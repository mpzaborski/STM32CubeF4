cmake_minimum_required(VERSION 3.15.3)

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
project(STM32F401RE-Nucleo)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(PROGRAM_CMD ST-LINK_CLI.exe)


set(STM32CUBEMX_GENERATED_FILES
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
        ../../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
        Src/main.c
        Src/stm32f4xx_hal_msp.c
        Src/stm32f4xx_it.c
        Src/system_stm32f4xx.c
		Src/bme280_port.c
        stm32-ssd1306/ssd1306/ssd1306.c
        stm32-ssd1306/ssd1306/ssd1306_fonts.c
		BME280_driver/bme280.c
        SW4STM32/startup_stm32f401xe.s)

set(EXECUTABLE ${PROJECT_NAME}.elf)
add_executable(${EXECUTABLE} ${STM32CUBEMX_GENERATED_FILES})

target_compile_definitions(${EXECUTABLE} PRIVATE
        -DUSE_HAL_DRIVER
        -DSTM32F413xx
        -DSSD1306_USE_I2C
        -DSSD1306_I2C_PORT=I2cHandle
        -DSTM32F4
        -DBME280_32BIT_ENABLE
        )

target_include_directories(${EXECUTABLE} PRIVATE
        Inc
        stm32-ssd1306/ssd1306
		BME280_driver
        ../../../Drivers/STM32F4xx_HAL_Driver/Inc
        ../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
        ../../../Drivers/CMSIS/Include
        ../../../Drivers/BSP/STM32F401-Discovery
        )

target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard

        -fdata-sections
        -ffunction-sections

        -Wall

        $<$<CONFIG:Debug>:-Og>
        )

target_link_options(${EXECUTABLE} PRIVATE
        -T${CMAKE_SOURCE_DIR}/SW4STM32/STM32F4xx-Nucleo/STM32F401VEHx_FLASH.ld
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -specs=nano.specs
        -specs=nosys.specs
        -lc
        -lm
        -lnosys
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        )

# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE})

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)

add_custom_target(program)
add_dependencies(program ${EXECUTABLE})
add_custom_command(TARGET program
    # On Visual Studio Generators, run before any other rules are executed within the target. On other generators, run just before PRE_LINK commands
    POST_BUILD
    COMMAND ${PROGRAM_CMD} -ME -P ${PROJECT_NAME}.bin 0x08000000 -V -HardRst
    COMMENT "This command will be executed before building bar"
)

add_custom_target(erase
    COMMAND ${PROGRAM_CMD} -ME -HardRst
    COMMENT "This command will be executed before building bar"
)
