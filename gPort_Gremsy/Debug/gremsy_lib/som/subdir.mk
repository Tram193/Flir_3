################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/gremsy_lib/som/som_communication.c \
D:/gPort_PSDKv350_Flir/gremsy_lib/som/som_hardware_reset.c 

OBJS += \
./gremsy_lib/som/som_communication.o \
./gremsy_lib/som/som_hardware_reset.o 

C_DEPS += \
./gremsy_lib/som/som_communication.d \
./gremsy_lib/som/som_hardware_reset.d 


# Each subdirectory must supply rules for building sources it contributes
gremsy_lib/som/som_communication.o: D:/gPort_PSDKv350_Flir/gremsy_lib/som/som_communication.c gremsy_lib/som/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"gremsy_lib/som/som_communication.c_includes.args"
gremsy_lib/som/som_hardware_reset.o: D:/gPort_PSDKv350_Flir/gremsy_lib/som/som_hardware_reset.c gremsy_lib/som/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"gremsy_lib/som/som_hardware_reset.c_includes.args"

clean: clean-gremsy_lib-2f-som

clean-gremsy_lib-2f-som:
	-$(RM) ./gremsy_lib/som/som_communication.cyclo ./gremsy_lib/som/som_communication.d ./gremsy_lib/som/som_communication.o ./gremsy_lib/som/som_communication.su ./gremsy_lib/som/som_hardware_reset.cyclo ./gremsy_lib/som/som_hardware_reset.d ./gremsy_lib/som/som_hardware_reset.o ./gremsy_lib/som/som_hardware_reset.su

.PHONY: clean-gremsy_lib-2f-som

