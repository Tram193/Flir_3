################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/gremsy_lib/gimbal/gsdk_gimbal.c 

OBJS += \
./gremsy_lib/gimbal/gsdk_gimbal.o 

C_DEPS += \
./gremsy_lib/gimbal/gsdk_gimbal.d 


# Each subdirectory must supply rules for building sources it contributes
gremsy_lib/gimbal/gsdk_gimbal.o: D:/gPort_PSDKv350_Flir/gremsy_lib/gimbal/gsdk_gimbal.c gremsy_lib/gimbal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"gremsy_lib/gimbal/gsdk_gimbal.c_includes.args"

clean: clean-gremsy_lib-2f-gimbal

clean-gremsy_lib-2f-gimbal:
	-$(RM) ./gremsy_lib/gimbal/gsdk_gimbal.cyclo ./gremsy_lib/gimbal/gsdk_gimbal.d ./gremsy_lib/gimbal/gsdk_gimbal.o ./gremsy_lib/gimbal/gsdk_gimbal.su

.PHONY: clean-gremsy_lib-2f-gimbal

