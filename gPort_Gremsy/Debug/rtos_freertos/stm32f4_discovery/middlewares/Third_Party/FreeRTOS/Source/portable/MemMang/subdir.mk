################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c_includes.args"

clean: clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang

clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang:
	-$(RM) ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.d ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.su

.PHONY: clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang

