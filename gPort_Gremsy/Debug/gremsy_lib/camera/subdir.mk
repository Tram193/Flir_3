################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/gremsy_lib/camera/gsdk_flir_camera.c 

OBJS += \
./gremsy_lib/camera/gsdk_flir_camera.o 

C_DEPS += \
./gremsy_lib/camera/gsdk_flir_camera.d 


# Each subdirectory must supply rules for building sources it contributes
gremsy_lib/camera/gsdk_flir_camera.o: D:/gPort_PSDKv350_Flir/gremsy_lib/camera/gsdk_flir_camera.c gremsy_lib/camera/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"gremsy_lib/camera/gsdk_flir_camera.c_includes.args"

clean: clean-gremsy_lib-2f-camera

clean-gremsy_lib-2f-camera:
	-$(RM) ./gremsy_lib/camera/gsdk_flir_camera.cyclo ./gremsy_lib/camera/gsdk_flir_camera.d ./gremsy_lib/camera/gsdk_flir_camera.o ./gremsy_lib/camera/gsdk_flir_camera.su

.PHONY: clean-gremsy_lib-2f-camera

