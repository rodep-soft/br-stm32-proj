################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/bno08x/Src/bno08x.c \
../Drivers/bno08x/Src/sh2_hal_stm32.c 

OBJS += \
./Drivers/bno08x/Src/bno08x.o \
./Drivers/bno08x/Src/sh2_hal_stm32.o 

C_DEPS += \
./Drivers/bno08x/Src/bno08x.d \
./Drivers/bno08x/Src/sh2_hal_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/bno08x/Src/%.o Drivers/bno08x/Src/%.su Drivers/bno08x/Src/%.cyclo: ../Drivers/bno08x/Src/%.c Drivers/bno08x/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Drivers/bno08x/Inc -I../Drivers/sh2/Inc -I../protocols -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-bno08x-2f-Src

clean-Drivers-2f-bno08x-2f-Src:
	-$(RM) ./Drivers/bno08x/Src/bno08x.cyclo ./Drivers/bno08x/Src/bno08x.d ./Drivers/bno08x/Src/bno08x.o ./Drivers/bno08x/Src/bno08x.su ./Drivers/bno08x/Src/sh2_hal_stm32.cyclo ./Drivers/bno08x/Src/sh2_hal_stm32.d ./Drivers/bno08x/Src/sh2_hal_stm32.o ./Drivers/bno08x/Src/sh2_hal_stm32.su

.PHONY: clean-Drivers-2f-bno08x-2f-Src

