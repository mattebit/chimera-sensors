################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Eagle_TRT.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/Eagle_TRT.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/Eagle_TRT.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F446xx '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Working Codes/New_with_library/FRONTAL/Inc" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Working Codes/New_with_library/FRONTAL/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Working Codes/New_with_library/FRONTAL/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Working Codes/New_with_library/FRONTAL/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Working Codes/New_with_library/FRONTAL/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


