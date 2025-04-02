################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/flight_control/test_flight_control.c 

OBJS += \
./module_sample/flight_control/test_flight_control.o 

C_DEPS += \
./module_sample/flight_control/test_flight_control.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/flight_control/test_flight_control.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/flight_control/test_flight_control.c module_sample/flight_control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/flight_control/test_flight_control.c_includes.args"

clean: clean-module_sample-2f-flight_control

clean-module_sample-2f-flight_control:
	-$(RM) ./module_sample/flight_control/test_flight_control.cyclo ./module_sample/flight_control/test_flight_control.d ./module_sample/flight_control/test_flight_control.o ./module_sample/flight_control/test_flight_control.su

.PHONY: clean-module_sample-2f-flight_control

