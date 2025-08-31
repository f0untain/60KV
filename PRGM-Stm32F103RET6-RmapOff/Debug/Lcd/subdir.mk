################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lcd/lcd.c 

OBJS += \
./Lcd/lcd.o 

C_DEPS += \
./Lcd/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Lcd/%.o Lcd/%.su Lcd/%.cyclo: ../Lcd/%.c Lcd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Projects/VS/Stm32F103RET6/Stm32F103RET6/Button" -I"D:/Projects/VS/Stm32F103RET6/Stm32F103RET6/Eeprom" -I"D:/Projects/VS/Stm32F103RET6/Stm32F103RET6/Lcd" -I"D:/Projects/VS/Stm32F103RET6/Stm32F103RET6/Menu" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lcd

clean-Lcd:
	-$(RM) ./Lcd/lcd.cyclo ./Lcd/lcd.d ./Lcd/lcd.o ./Lcd/lcd.su

.PHONY: clean-Lcd

