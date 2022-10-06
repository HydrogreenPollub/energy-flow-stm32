################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Hydrogreen/Energy_flow.c \
../Hydrogreen/fans.c \
../Hydrogreen/hydrogreen.c \
../Hydrogreen/measurements.c \
../Hydrogreen/motor.c \
../Hydrogreen/pidController.c \
../Hydrogreen/rs485.c \
../Hydrogreen/timers.c \
../Hydrogreen/watchdog.c 

OBJS += \
./Hydrogreen/Energy_flow.o \
./Hydrogreen/fans.o \
./Hydrogreen/hydrogreen.o \
./Hydrogreen/measurements.o \
./Hydrogreen/motor.o \
./Hydrogreen/pidController.o \
./Hydrogreen/rs485.o \
./Hydrogreen/timers.o \
./Hydrogreen/watchdog.o 

C_DEPS += \
./Hydrogreen/Energy_flow.d \
./Hydrogreen/fans.d \
./Hydrogreen/hydrogreen.d \
./Hydrogreen/measurements.d \
./Hydrogreen/motor.d \
./Hydrogreen/pidController.d \
./Hydrogreen/rs485.d \
./Hydrogreen/timers.d \
./Hydrogreen/watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
Hydrogreen/Energy_flow.o: ../Hydrogreen/Energy_flow.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/Energy_flow.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Hydrogreen/fans.o: ../Hydrogreen/fans.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/fans.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Hydrogreen/hydrogreen.o: ../Hydrogreen/hydrogreen.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/hydrogreen.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Hydrogreen/measurements.o: ../Hydrogreen/measurements.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/measurements.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Hydrogreen/motor.o: ../Hydrogreen/motor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/motor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Hydrogreen/pidController.o: ../Hydrogreen/pidController.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/pidController.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Hydrogreen/rs485.o: ../Hydrogreen/rs485.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/rs485.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Hydrogreen/timers.o: ../Hydrogreen/timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/timers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Hydrogreen/watchdog.o: ../Hydrogreen/watchdog.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Program Files/PLIKI/STM32CubeIDE/workspace_1.4.0/TEST_ENERGY_FLOW/Test_EnergyFlow1/Hydrogreen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Hydrogreen/watchdog.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

