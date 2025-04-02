################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include rtos_freertos/stm32f4_discovery/project/mdk_bootloader/Objects/subdir.mk
-include rtos_freertos/stm32f4_discovery/project/mdk/Objects/subdir.mk
-include rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/subdir.mk
-include rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Class/CDC/Src/subdir.mk
-include rtos_freertos/stm32f4_discovery/hal/subdir.mk
-include rtos_freertos/stm32f4_discovery/drivers/USB_HOST/Target/subdir.mk
-include rtos_freertos/stm32f4_discovery/drivers/USB_HOST/App/subdir.mk
-include rtos_freertos/stm32f4_discovery/drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include rtos_freertos/stm32f4_discovery/drivers/BSP/subdir.mk
-include rtos_freertos/stm32f4_discovery/bootloader/subdir.mk
-include rtos_freertos/stm32f4_discovery/application/subdir.mk
-include rtos_freertos/common/osal/subdir.mk
-include module_sample/xport/subdir.mk
-include module_sample/widget_interaction_test/subdir.mk
-include module_sample/widget/subdir.mk
-include module_sample/waypoint_v3/subdir.mk
-include module_sample/waypoint_v2/subdir.mk
-include module_sample/utils/subdir.mk
-include module_sample/upgrade/subdir.mk
-include module_sample/time_sync/subdir.mk
-include module_sample/power_management/subdir.mk
-include module_sample/positioning/subdir.mk
-include module_sample/perception/subdir.mk
-include module_sample/payload_collaboration/subdir.mk
-include module_sample/mop_channel/subdir.mk
-include module_sample/liveview/subdir.mk
-include module_sample/hms/subdir.mk
-include module_sample/gimbal_manager/subdir.mk
-include module_sample/gimbal_emu/subdir.mk
-include module_sample/flight_control/subdir.mk
-include module_sample/fc_subscription/subdir.mk
-include module_sample/data_transmission/subdir.mk
-include module_sample/camera_manager/subdir.mk
-include module_sample/camera_emu/subdir.mk
-include gremsy_lib/som/subdir.mk
-include gremsy_lib/mavlink/subdir.mk
-include gremsy_lib/gimbal/subdir.mk
-include gremsy_lib/camera/subdir.mk
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


BUILD_ARTIFACT_NAME := gPort_Gremsy_2
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
gPort_Gremsy_2.elf \

MAP_FILES += \
gPort_Gremsy_2.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
gPort_Gremsy_2.list \

OBJCOPY_HEX += \
gPort_Gremsy_2.hex \


# All Target
all: main-build

# Main-build Target
main-build: gPort_Gremsy_2.elf secondary-outputs

# Tool invocations
gPort_Gremsy_2.elf gPort_Gremsy_2.map: $(OBJS) $(USER_OBJS) D:\gPort_PSDKv350_Flir\samples\sample_c\platform\rtos_freertos\stm32f4_discovery\project\armgcc\STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "gPort_Gremsy_2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\gPort_PSDKv350_Flir\samples\sample_c\platform\rtos_freertos\stm32f4_discovery\project\armgcc\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="gPort_Gremsy_2.map" -Wl,--gc-sections -static -L"D:\gPort_PSDKv350_Flir\psdk_lib\lib\arm-none-eabi-gcc" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

gPort_Gremsy_2.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "gPort_Gremsy_2.list"
	@echo 'Finished building: $@'
	@echo ' '

gPort_Gremsy_2.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "gPort_Gremsy_2.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout gPort_Gremsy_2.elf gPort_Gremsy_2.hex gPort_Gremsy_2.list gPort_Gremsy_2.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
