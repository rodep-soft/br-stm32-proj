################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../protocols/bno085_can.c 

OBJS += \
./protocols/bno085_can.o 

C_DEPS += \
./protocols/bno085_can.d 


# Each subdirectory must supply rules for building sources it contributes
protocols/%.o protocols/%.su protocols/%.cyclo: ../protocols/%.c protocols/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Drivers/bno08x/Inc -I../Drivers/sh2/Inc -I../protocols -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-protocols

clean-protocols:
	-$(RM) ./protocols/bno085_can.cyclo ./protocols/bno085_can.d ./protocols/bno085_can.o ./protocols/bno085_can.su

.PHONY: clean-protocols

