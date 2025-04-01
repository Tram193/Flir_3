################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_buffer.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_file.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_md5.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_misc.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_time.c 

OBJS += \
./module_sample/utils/util_buffer.o \
./module_sample/utils/util_file.o \
./module_sample/utils/util_md5.o \
./module_sample/utils/util_misc.o \
./module_sample/utils/util_time.o 

C_DEPS += \
./module_sample/utils/util_buffer.d \
./module_sample/utils/util_file.d \
./module_sample/utils/util_md5.d \
./module_sample/utils/util_misc.d \
./module_sample/utils/util_time.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/utils/util_buffer.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_buffer.c module_sample/utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/utils/util_buffer.c_includes.args"
module_sample/utils/util_file.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_file.c module_sample/utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/utils/util_file.c_includes.args"
module_sample/utils/util_md5.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_md5.c module_sample/utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/utils/util_md5.c_includes.args"
module_sample/utils/util_misc.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_misc.c module_sample/utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/utils/util_misc.c_includes.args"
module_sample/utils/util_time.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/utils/util_time.c module_sample/utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/utils/util_time.c_includes.args"

clean: clean-module_sample-2f-utils

clean-module_sample-2f-utils:
	-$(RM) ./module_sample/utils/util_buffer.cyclo ./module_sample/utils/util_buffer.d ./module_sample/utils/util_buffer.o ./module_sample/utils/util_buffer.su ./module_sample/utils/util_file.cyclo ./module_sample/utils/util_file.d ./module_sample/utils/util_file.o ./module_sample/utils/util_file.su ./module_sample/utils/util_md5.cyclo ./module_sample/utils/util_md5.d ./module_sample/utils/util_md5.o ./module_sample/utils/util_md5.su ./module_sample/utils/util_misc.cyclo ./module_sample/utils/util_misc.d ./module_sample/utils/util_misc.o ./module_sample/utils/util_misc.su ./module_sample/utils/util_time.cyclo ./module_sample/utils/util_time.d ./module_sample/utils/util_time.o ./module_sample/utils/util_time.su

.PHONY: clean-module_sample-2f-utils

