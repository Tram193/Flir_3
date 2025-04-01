################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/application/application.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/application/main.c 

OBJS += \
./rtos_freertos/stm32f4_discovery/application/application.o \
./rtos_freertos/stm32f4_discovery/application/main.o 

C_DEPS += \
./rtos_freertos/stm32f4_discovery/application/application.d \
./rtos_freertos/stm32f4_discovery/application/main.d 


# Each subdirectory must supply rules for building sources it contributes
rtos_freertos/stm32f4_discovery/application/application.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/application/application.c rtos_freertos/stm32f4_discovery/application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/application/application.c_includes.args"
rtos_freertos/stm32f4_discovery/application/main.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/application/main.c rtos_freertos/stm32f4_discovery/application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/application/main.c_includes.args"

clean: clean-rtos_freertos-2f-stm32f4_discovery-2f-application

clean-rtos_freertos-2f-stm32f4_discovery-2f-application:
	-$(RM) ./rtos_freertos/stm32f4_discovery/application/application.cyclo ./rtos_freertos/stm32f4_discovery/application/application.d ./rtos_freertos/stm32f4_discovery/application/application.o ./rtos_freertos/stm32f4_discovery/application/application.su ./rtos_freertos/stm32f4_discovery/application/main.cyclo ./rtos_freertos/stm32f4_discovery/application/main.d ./rtos_freertos/stm32f4_discovery/application/main.o ./rtos_freertos/stm32f4_discovery/application/main.su

.PHONY: clean-rtos_freertos-2f-stm32f4_discovery-2f-application

