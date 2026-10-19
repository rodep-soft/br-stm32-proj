################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/sh2/Src/euler.c \
../Drivers/sh2/Src/sh2.c \
../Drivers/sh2/Src/sh2_SensorValue.c \
../Drivers/sh2/Src/sh2_util.c \
../Drivers/sh2/Src/shtp.c 

OBJS += \
./Drivers/sh2/Src/euler.o \
./Drivers/sh2/Src/sh2.o \
./Drivers/sh2/Src/sh2_SensorValue.o \
./Drivers/sh2/Src/sh2_util.o \
./Drivers/sh2/Src/shtp.o 

C_DEPS += \
./Drivers/sh2/Src/euler.d \
./Drivers/sh2/Src/sh2.d \
./Drivers/sh2/Src/sh2_SensorValue.d \
./Drivers/sh2/Src/sh2_util.d \
./Drivers/sh2/Src/shtp.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/sh2/Src/%.o Drivers/sh2/Src/%.su Drivers/sh2/Src/%.cyclo: ../Drivers/sh2/Src/%.c Drivers/sh2/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Drivers/bno08x/Inc -I../Drivers/sh2/Inc -I../protocols -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-sh2-2f-Src

clean-Drivers-2f-sh2-2f-Src:
	-$(RM) ./Drivers/sh2/Src/euler.cyclo ./Drivers/sh2/Src/euler.d ./Drivers/sh2/Src/euler.o ./Drivers/sh2/Src/euler.su ./Drivers/sh2/Src/sh2.cyclo ./Drivers/sh2/Src/sh2.d ./Drivers/sh2/Src/sh2.o ./Drivers/sh2/Src/sh2.su ./Drivers/sh2/Src/sh2_SensorValue.cyclo ./Drivers/sh2/Src/sh2_SensorValue.d ./Drivers/sh2/Src/sh2_SensorValue.o ./Drivers/sh2/Src/sh2_SensorValue.su ./Drivers/sh2/Src/sh2_util.cyclo ./Drivers/sh2/Src/sh2_util.d ./Drivers/sh2/Src/sh2_util.o ./Drivers/sh2/Src/sh2_util.su ./Drivers/sh2/Src/shtp.cyclo ./Drivers/sh2/Src/shtp.d ./Drivers/sh2/Src/shtp.o ./Drivers/sh2/Src/shtp.su

.PHONY: clean-Drivers-2f-sh2-2f-Src

