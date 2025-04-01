################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/hms/test_hms.c 

OBJS += \
./module_sample/hms/test_hms.o 

C_DEPS += \
./module_sample/hms/test_hms.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/hms/test_hms.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/hms/test_hms.c module_sample/hms/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/hms/test_hms.c_includes.args"

clean: clean-module_sample-2f-hms

clean-module_sample-2f-hms:
	-$(RM) ./module_sample/hms/test_hms.cyclo ./module_sample/hms/test_hms.d ./module_sample/hms/test_hms.o ./module_sample/hms/test_hms.su

.PHONY: clean-module_sample-2f-hms

