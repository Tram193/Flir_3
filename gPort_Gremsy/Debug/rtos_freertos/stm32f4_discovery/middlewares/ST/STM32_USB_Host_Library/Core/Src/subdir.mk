################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c 

OBJS += \
./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.o \
./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.o \
./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.o \
./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.o 

C_DEPS += \
./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.d \
./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.d \
./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.d \
./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.d 


# Each subdirectory must supply rules for building sources it contributes
rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c_includes.args"
rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.o: D:/gPort_PSDKv350_Flir/samples/sample_c/platform/rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c_includes.args"

clean: clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-ST-2f-STM32_USB_Host_Library-2f-Core-2f-Src

clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-ST-2f-STM32_USB_Host_Library-2f-Core-2f-Src:
	-$(RM) ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.d ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.o ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.su ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.d ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.o ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.su ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.d ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.o ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.su ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.cyclo ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.d ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.o ./rtos_freertos/stm32f4_discovery/middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.su

.PHONY: clean-rtos_freertos-2f-stm32f4_discovery-2f-middlewares-2f-ST-2f-STM32_USB_Host_Library-2f-Core-2f-Src

