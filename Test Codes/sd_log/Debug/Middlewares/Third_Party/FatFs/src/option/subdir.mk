################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/option/syscall.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/option/%.o: ../Middlewares/Third_Party/FatFs/src/option/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F446xx '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Test Codes/sd_log/Inc" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Test Codes/sd_log/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Test Codes/sd_log/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Test Codes/sd_log/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Test Codes/sd_log/Middlewares/Third_Party/FatFs/src" -I"/media/usr/label/Programmazione/Github/Eagle/fenice-sensors/Test Codes/sd_log/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


