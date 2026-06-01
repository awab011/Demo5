################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/LwIP/system/OS/subdir.mk
-include Middlewares/Third_Party/LwIP/src/netif/ppp/subdir.mk
-include Middlewares/Third_Party/LwIP/src/netif/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/ipv6/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/ipv4/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/subdir.mk
-include Middlewares/Third_Party/LwIP/src/apps/mqtt/subdir.mk
-include Middlewares/Third_Party/LwIP/src/api/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include LWIP/Target/subdir.mk
-include LWIP/App/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/Components/lan8742/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/User/Forest/subdir.mk
-include Core/Src/Sensors/WTIMU63/subdir.mk
-include Core/Src/Sensors/TFmini/subdir.mk
-include Core/Src/Sensors/PWMEncoder/subdir.mk
-include Core/Src/Sensors/LASER/subdir.mk
-include Core/Src/Sensors/IMU/r6091u/subdir.mk
-include Core/Src/Sensors/IMU/BMM350_IMU/subdir.mk
-include Core/Src/Sensors/Benewake/subdir.mk
-include Core/Src/Platform/subdir.mk
-include Core/Src/Drivers/myswerve/subdir.mk
-include Core/Src/Drivers/Swerve/subdir.mk
-include Core/Src/Drivers/PSx_Interface/subdir.mk
-include Core/Src/Drivers/PMW3901/subdir.mk
-include Core/Src/Drivers/Can_Bridge/subdir.mk
-include Core/Src/Drivers/subdir.mk
-include Core/Src/BIOS/COM/ESP/subdir.mk
-include Core/Src/BIOS/COM/subdir.mk
-include Core/Src/BIOS/subdir.mk
-include Core/Src/Algorithms/trajectory_control/subdir.mk
-include Core/Src/Algorithms/PathProfile/subdir.mk
-include Core/Src/Algorithms/PID/subdir.mk
-include Core/Src/Algorithms/Navigator/subdir.mk
-include Core/Src/Algorithms/Moving_Average/subdir.mk
-include Core/Src/Algorithms/MODN/subdir.mk
-include Core/Src/Algorithms/KalmanFilter/subdir.mk
-include Core/Src/Algorithms/KF/subdir.mk
-include Core/Src/Algorithms/Inverse_Kinematics/subdir.mk
-include Core/Src/Algorithms/BEZIER/subdir.mk
-include Core/Src/Algorithms/ABT/subdir.mk
-include Core/Src/Actuators/VESC_FDCAN/subdir.mk
-include Core/Src/Actuators/Robomaster/subdir.mk
-include Core/Src/Actuators/J60/subdir.mk
-include Core/Src/Actuators/Cybergear/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := H7_R2
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
H7_R2.elf \

MAP_FILES += \
H7_R2.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
H7_R2.list \


# All Target
all: main-build

# Main-build Target
main-build: H7_R2.elf secondary-outputs

# Tool invocations
H7_R2.elf H7_R2.map: $(OBJS) $(USER_OBJS) /Users/awabelgubshawi/RBC_workspace/Demo5_R2_replica/STM32H735VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "H7_R2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"/Users/awabelgubshawi/RBC_workspace/Demo5_R2_replica/STM32H735VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="H7_R2.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

H7_R2.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "H7_R2.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) H7_R2.elf H7_R2.list H7_R2.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
