################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget/file_binary_array_list_en.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget/test_widget.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget/test_widget_speaker.c 

OBJS += \
./module_sample/widget/file_binary_array_list_en.o \
./module_sample/widget/test_widget.o \
./module_sample/widget/test_widget_speaker.o 

C_DEPS += \
./module_sample/widget/file_binary_array_list_en.d \
./module_sample/widget/test_widget.d \
./module_sample/widget/test_widget_speaker.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/widget/file_binary_array_list_en.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget/file_binary_array_list_en.c module_sample/widget/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/widget/file_binary_array_list_en.c_includes.args"
module_sample/widget/test_widget.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget/test_widget.c module_sample/widget/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/widget/test_widget.c_includes.args"
module_sample/widget/test_widget_speaker.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/widget/test_widget_speaker.c module_sample/widget/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/widget/test_widget_speaker.c_includes.args"

clean: clean-module_sample-2f-widget

clean-module_sample-2f-widget:
	-$(RM) ./module_sample/widget/file_binary_array_list_en.cyclo ./module_sample/widget/file_binary_array_list_en.d ./module_sample/widget/file_binary_array_list_en.o ./module_sample/widget/file_binary_array_list_en.su ./module_sample/widget/test_widget.cyclo ./module_sample/widget/test_widget.d ./module_sample/widget/test_widget.o ./module_sample/widget/test_widget.su ./module_sample/widget/test_widget_speaker.cyclo ./module_sample/widget/test_widget_speaker.d ./module_sample/widget/test_widget_speaker.o ./module_sample/widget/test_widget_speaker.su

.PHONY: clean-module_sample-2f-widget

