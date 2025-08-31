################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Input/input.c 

OBJS += \
./Input/input.o 

C_DEPS += \
./Input/input.d 


# Each subdirectory must supply rules for building sources it contributes
Input/%.o Input/%.su Input/%.cyclo: ../Input/%.c Input/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F334x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"D:/Projects/60KV/PRGM/60KV/Button" -I"D:/Projects/60KV/PRGM/60KV/Eeprom" -I"D:/Projects/60KV/PRGM/60KV/Lcd" -I"D:/Projects/60KV/PRGM/60KV/Menu" -I"D:/Projects/60KV/PRGM/60KV/Input" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Input

clean-Input:
	-$(RM) ./Input/input.cyclo ./Input/input.d ./Input/input.o ./Input/input.su

.PHONY: clean-Input

