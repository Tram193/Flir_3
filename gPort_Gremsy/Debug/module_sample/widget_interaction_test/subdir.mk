################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget_interaction_test/file_binary_array_list_en.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget_interaction_test/test_widget_interaction.c 

OBJS += \
./module_sample/widget_interaction_test/file_binary_array_list_en.o \
./module_sample/widget_interaction_test/test_widget_interaction.o 

C_DEPS += \
./module_sample/widget_interaction_test/file_binary_array_list_en.d \
./module_sample/widget_interaction_test/test_widget_interaction.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/widget_interaction_test/file_binary_array_list_en.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget_interaction_test/file_binary_array_list_en.c module_sample/widget_interaction_test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/widget_interaction_test/file_binary_array_list_en.c_includes.args"
module_sample/widget_interaction_test/test_widget_interaction.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget_interaction_test/test_widget_interaction.c module_sample/widget_interaction_test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/widget_interaction_test/test_widget_interaction.c_includes.args"

clean: clean-module_sample-2f-widget_interaction_test

clean-module_sample-2f-widget_interaction_test:
	-$(RM) ./module_sample/widget_interaction_test/file_binary_array_list_en.cyclo ./module_sample/widget_interaction_test/file_binary_array_list_en.d ./module_sample/widget_interaction_test/file_binary_array_list_en.o ./module_sample/widget_interaction_test/file_binary_array_list_en.su ./module_sample/widget_interaction_test/test_widget_interaction.cyclo ./module_sample/widget_interaction_test/test_widget_interaction.d ./module_sample/widget_interaction_test/test_widget_interaction.o ./module_sample/widget_interaction_test/test_widget_interaction.su

.PHONY: clean-module_sample-2f-widget_interaction_test

