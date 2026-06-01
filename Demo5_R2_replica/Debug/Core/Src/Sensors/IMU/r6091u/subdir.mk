################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Sensors/IMU/r6091u/r6091u.c 

OBJS += \
./Core/Src/Sensors/IMU/r6091u/r6091u.o 

C_DEPS += \
./Core/Src/Sensors/IMU/r6091u/r6091u.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Sensors/IMU/r6091u/%.o Core/Src/Sensors/IMU/r6091u/%.su Core/Src/Sensors/IMU/r6091u/%.cyclo: ../Core/Src/Sensors/IMU/r6091u/%.c Core/Src/Sensors/IMU/r6091u/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H735xx -DUSE_PWR_LDO_SUPPLY -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Drivers/BSP/Components/lan8742 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Sensors-2f-IMU-2f-r6091u

clean-Core-2f-Src-2f-Sensors-2f-IMU-2f-r6091u:
	-$(RM) ./Core/Src/Sensors/IMU/r6091u/r6091u.cyclo ./Core/Src/Sensors/IMU/r6091u/r6091u.d ./Core/Src/Sensors/IMU/r6091u/r6091u.o ./Core/Src/Sensors/IMU/r6091u/r6091u.su

.PHONY: clean-Core-2f-Src-2f-Sensors-2f-IMU-2f-r6091u

