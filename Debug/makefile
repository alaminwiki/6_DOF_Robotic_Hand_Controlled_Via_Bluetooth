################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
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


BUILD_ARTIFACT_NAME := 6_DOF_Robotic_Hand_Slave
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
6_DOF_Robotic_Hand_Slave.elf \

MAP_FILES += \
6_DOF_Robotic_Hand_Slave.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
6_DOF_Robotic_Hand_Slave.list \

OBJCOPY_HEX += \
6_DOF_Robotic_Hand_Slave.hex \

OBJCOPY_BIN += \
6_DOF_Robotic_Hand_Slave.bin \


# All Target
all: main-build

# Main-build Target
main-build: 6_DOF_Robotic_Hand_Slave.elf secondary-outputs

# Tool invocations
6_DOF_Robotic_Hand_Slave.elf 6_DOF_Robotic_Hand_Slave.map: $(OBJS) $(USER_OBJS) E:\STM32\ Project\ Folder\ 2\6_DOF_Robotic_Hand_Bluetooth_Controlled_001\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "6_DOF_Robotic_Hand_Slave.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"E:\STM32 Project Folder 2\6_DOF_Robotic_Hand_Bluetooth_Controlled_001\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="6_DOF_Robotic_Hand_Slave.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

6_DOF_Robotic_Hand_Slave.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "6_DOF_Robotic_Hand_Slave.list"
	@echo 'Finished building: $@'
	@echo ' '

6_DOF_Robotic_Hand_Slave.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "6_DOF_Robotic_Hand_Slave.hex"
	@echo 'Finished building: $@'
	@echo ' '

6_DOF_Robotic_Hand_Slave.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "6_DOF_Robotic_Hand_Slave.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) 6_DOF_Robotic_Hand_Slave.bin 6_DOF_Robotic_Hand_Slave.elf 6_DOF_Robotic_Hand_Slave.hex 6_DOF_Robotic_Hand_Slave.list 6_DOF_Robotic_Hand_Slave.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
