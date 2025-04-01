################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.o \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.o \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.o 

C_DEPS += \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.d \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.d \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.c rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.c rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.c rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.c rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.c rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.c rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.c_includes.args"

clean: clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source

clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source:
	-$(RM) ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.d ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.o ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/croutine.su ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.d ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.o ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/event_groups.su ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.d ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.o ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/list.su ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.d ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.o ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/queue.su ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/stream_buffer.su ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.d ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.o ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/tasks.su ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.d ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.o ./rtos_freertos/stm32f4_discovery/middlewares/Third_Party/FreeRTOS/Source/timers.su

.PHONY: clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-Third_Party-2f-FreeRTOS-2f-Source

