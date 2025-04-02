################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/mop_channel/test_mop_channel.c 

OBJS += \
./module_sample/mop_channel/test_mop_channel.o 

C_DEPS += \
./module_sample/mop_channel/test_mop_channel.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/mop_channel/test_mop_channel.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/mop_channel/test_mop_channel.c module_sample/mop_channel/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/mop_channel/test_mop_channel.c_includes.args"

clean: clean-module_sample-2f-mop_channel

clean-module_sample-2f-mop_channel:
	-$(RM) ./module_sample/mop_channel/test_mop_channel.cyclo ./module_sample/mop_channel/test_mop_channel.d ./module_sample/mop_channel/test_mop_channel.o ./module_sample/mop_channel/test_mop_channel.su

.PHONY: clean-module_sample-2f-mop_channel

