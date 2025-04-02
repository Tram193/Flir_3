################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.c 

OBJS += \
./module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.o \
./module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.o \
./module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.o 

C_DEPS += \
./module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.d \
./module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.d \
./module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.c module_sample/camera_emu/dji_media_file_manage/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.c_includes.args"
module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.c module_sample/camera_emu/dji_media_file_manage/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.c_includes.args"
module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.c module_sample/camera_emu/dji_media_file_manage/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.c_includes.args"

clean: clean-module_sample-2f-camera_emu-2f-dji_media_file_manage

clean-module_sample-2f-camera_emu-2f-dji_media_file_manage:
	-$(RM) ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.cyclo ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.d ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.o ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_core.su ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.cyclo ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.d ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.o ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_jpg.su ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.cyclo ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.d ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.o ./module_sample/camera_emu/dji_media_file_manage/dji_media_file_mp4.su

.PHONY: clean-module_sample-2f-camera_emu-2f-dji_media_file_manage

