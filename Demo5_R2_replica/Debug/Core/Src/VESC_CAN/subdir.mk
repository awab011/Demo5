################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/VESC_CAN/bldc_interface.c \
../Core/Src/VESC_CAN/buffer.c \
../Core/Src/VESC_CAN/crc.c \
../Core/Src/VESC_CAN/packet.c \
../Core/Src/VESC_CAN/vesc_fdcan.c \
../Core/Src/VESC_CAN/vesc_interface.c \
../Core/Src/VESC_CAN/vesc_uart.c 

OBJS += \
./Core/Src/VESC_CAN/bldc_interface.o \
./Core/Src/VESC_CAN/buffer.o \
./Core/Src/VESC_CAN/crc.o \
./Core/Src/VESC_CAN/packet.o \
./Core/Src/VESC_CAN/vesc_fdcan.o \
./Core/Src/VESC_CAN/vesc_interface.o \
./Core/Src/VESC_CAN/vesc_uart.o 

C_DEPS += \
./Core/Src/VESC_CAN/bldc_interface.d \
./Core/Src/VESC_CAN/buffer.d \
./Core/Src/VESC_CAN/crc.d \
./Core/Src/VESC_CAN/packet.d \
./Core/Src/VESC_CAN/vesc_fdcan.d \
./Core/Src/VESC_CAN/vesc_interface.d \
./Core/Src/VESC_CAN/vesc_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/VESC_CAN/%.o Core/Src/VESC_CAN/%.su Core/Src/VESC_CAN/%.cyclo: ../Core/Src/VESC_CAN/%.c Core/Src/VESC_CAN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H735xx -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-VESC_CAN

clean-Core-2f-Src-2f-VESC_CAN:
	-$(RM) ./Core/Src/VESC_CAN/bldc_interface.cyclo ./Core/Src/VESC_CAN/bldc_interface.d ./Core/Src/VESC_CAN/bldc_interface.o ./Core/Src/VESC_CAN/bldc_interface.su ./Core/Src/VESC_CAN/buffer.cyclo ./Core/Src/VESC_CAN/buffer.d ./Core/Src/VESC_CAN/buffer.o ./Core/Src/VESC_CAN/buffer.su ./Core/Src/VESC_CAN/crc.cyclo ./Core/Src/VESC_CAN/crc.d ./Core/Src/VESC_CAN/crc.o ./Core/Src/VESC_CAN/crc.su ./Core/Src/VESC_CAN/packet.cyclo ./Core/Src/VESC_CAN/packet.d ./Core/Src/VESC_CAN/packet.o ./Core/Src/VESC_CAN/packet.su ./Core/Src/VESC_CAN/vesc_fdcan.cyclo ./Core/Src/VESC_CAN/vesc_fdcan.d ./Core/Src/VESC_CAN/vesc_fdcan.o ./Core/Src/VESC_CAN/vesc_fdcan.su ./Core/Src/VESC_CAN/vesc_interface.cyclo ./Core/Src/VESC_CAN/vesc_interface.d ./Core/Src/VESC_CAN/vesc_interface.o ./Core/Src/VESC_CAN/vesc_interface.su ./Core/Src/VESC_CAN/vesc_uart.cyclo ./Core/Src/VESC_CAN/vesc_uart.d ./Core/Src/VESC_CAN/vesc_uart.o ./Core/Src/VESC_CAN/vesc_uart.su

.PHONY: clean-Core-2f-Src-2f-VESC_CAN

