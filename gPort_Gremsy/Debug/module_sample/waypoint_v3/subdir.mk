################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/waypoint_v3/test_waypoint_v3.c 

OBJS += \
./module_sample/waypoint_v3/test_waypoint_v3.o 

C_DEPS += \
./module_sample/waypoint_v3/test_waypoint_v3.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/waypoint_v3/test_waypoint_v3.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/waypoint_v3/test_waypoint_v3.c module_sample/waypoint_v3/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/waypoint_v3/test_waypoint_v3.c_includes.args"

clean: clean-module_sample-2f-waypoint_v3

clean-module_sample-2f-waypoint_v3:
	-$(RM) ./module_sample/waypoint_v3/test_waypoint_v3.cyclo ./module_sample/waypoint_v3/test_waypoint_v3.d ./module_sample/waypoint_v3/test_waypoint_v3.o ./module_sample/waypoint_v3/test_waypoint_v3.su

.PHONY: clean-module_sample-2f-waypoint_v3

