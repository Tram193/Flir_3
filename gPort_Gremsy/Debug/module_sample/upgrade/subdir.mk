################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/upgrade/test_upgrade.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/upgrade/test_upgrade_common_file_transfer.c \
D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/upgrade/test_upgrade_platform_opt.c 

OBJS += \
./module_sample/upgrade/test_upgrade.o \
./module_sample/upgrade/test_upgrade_common_file_transfer.o \
./module_sample/upgrade/test_upgrade_platform_opt.o 

C_DEPS += \
./module_sample/upgrade/test_upgrade.d \
./module_sample/upgrade/test_upgrade_common_file_transfer.d \
./module_sample/upgrade/test_upgrade_platform_opt.d 


# Each subdirectory must supply rules for building sources it contributes
module_sample/upgrade/test_upgrade.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/upgrade/test_upgrade.c module_sample/upgrade/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/upgrade/test_upgrade.c_includes.args"
module_sample/upgrade/test_upgrade_common_file_transfer.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/upgrade/test_upgrade_common_file_transfer.c module_sample/upgrade/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/upgrade/test_upgrade_common_file_transfer.c_includes.args"
module_sample/upgrade/test_upgrade_platform_opt.o: D:/gPort_PSDKv350_Flir/samples/sample_c/module_sample/upgrade/test_upgrade_platform_opt.c module_sample/upgrade/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VETx -DSYSTEM_ARCH_RTOS -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_BOOTLOADER=1 -DFLIR_CAM=1 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" @"module_sample/upgrade/test_upgrade_platform_opt.c_includes.args"

clean: clean-module_sample-2f-upgrade

clean-module_sample-2f-upgrade:
	-$(RM) ./module_sample/upgrade/test_upgrade.cyclo ./module_sample/upgrade/test_upgrade.d ./module_sample/upgrade/test_upgrade.o ./module_sample/upgrade/test_upgrade.su ./module_sample/upgrade/test_upgrade_common_file_transfer.cyclo ./module_sample/upgrade/test_upgrade_common_file_transfer.d ./module_sample/upgrade/test_upgrade_common_file_transfer.o ./module_sample/upgrade/test_upgrade_common_file_transfer.su ./module_sample/upgrade/test_upgrade_platform_opt.cyclo ./module_sample/upgrade/test_upgrade_platform_opt.d ./module_sample/upgrade/test_upgrade_platform_opt.o ./module_sample/upgrade/test_upgrade_platform_opt.su

.PHONY: clean-module_sample-2f-upgrade

