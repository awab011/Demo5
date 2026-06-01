################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BIOS/COM/FDCAN.c \
../Core/Src/BIOS/COM/i2c.c \
../Core/Src/BIOS/COM/spi.c \
../Core/Src/BIOS/COM/uart.c 

OBJS += \
./Core/Src/BIOS/COM/FDCAN.o \
./Core/Src/BIOS/COM/i2c.o \
./Core/Src/BIOS/COM/spi.o \
./Core/Src/BIOS/COM/uart.o 

C_DEPS += \
./Core/Src/BIOS/COM/FDCAN.d \
./Core/Src/BIOS/COM/i2c.d \
./Core/Src/BIOS/COM/spi.d \
./Core/Src/BIOS/COM/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/BIOS/COM/%.o Core/Src/BIOS/COM/%.su Core/Src/BIOS/COM/%.cyclo: ../Core/Src/BIOS/COM/%.c Core/Src/BIOS/COM/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H735xx -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-BIOS-2f-COM

clean-Core-2f-Src-2f-BIOS-2f-COM:
	-$(RM) ./Core/Src/BIOS/COM/FDCAN.cyclo ./Core/Src/BIOS/COM/FDCAN.d ./Core/Src/BIOS/COM/FDCAN.o ./Core/Src/BIOS/COM/FDCAN.su ./Core/Src/BIOS/COM/i2c.cyclo ./Core/Src/BIOS/COM/i2c.d ./Core/Src/BIOS/COM/i2c.o ./Core/Src/BIOS/COM/i2c.su ./Core/Src/BIOS/COM/spi.cyclo ./Core/Src/BIOS/COM/spi.d ./Core/Src/BIOS/COM/spi.o ./Core/Src/BIOS/COM/spi.su ./Core/Src/BIOS/COM/uart.cyclo ./Core/Src/BIOS/COM/uart.d ./Core/Src/BIOS/COM/uart.o ./Core/Src/BIOS/COM/uart.su

.PHONY: clean-Core-2f-Src-2f-BIOS-2f-COM

