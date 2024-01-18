################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Users/Collin/Documents/Wheel\ line\ movers/nRF52_wheelLine_accel_SDK17/nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags/nrf_atflags.c 

C_DEPS += \
./nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags/nrf_atflags.d 

OBJS += \
./nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags/nrf_atflags.o 


# Each subdirectory must supply rules for building sources it contributes
nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags/nrf_atflags.o: D:/Users/Collin/Documents/Wheel\ line\ movers/nRF52_wheelLine_accel_SDK17/nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags/nrf_atflags.c nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -march=armv7e-m -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -munaligned-access -fomit-frame-pointer -O0 -fmessage-length=0 -ffunction-sections -fdata-sections -fno-builtin -fmerge-constants -fno-strict-aliasing -fshort-enums -Wunused -Wuninitialized -Wall -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -Wdouble-promotion -g3 -DCONFIG_GPIO_AS_PINRESET -DFLOAT_ABI_HARD -DNRF52 -DNRF52832_XXAA -DNRF52_PAN_74 -DNRF_SD_BLE_API_VERSION=7 -DS112 -DSOFTDEVICE_PRESENT -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF52_common/segger_RTT_v788j/Config" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF52_common/segger_RTT_v788j/RTT" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/delay" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/util" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/softdevice/common" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/softdevice/mbr/headers" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/softdevice/s112/headers" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/softdevice/s112/headers/nrf52" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/toolchain/arm" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/toolchain/cmsis/dsp/Include" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/toolchain/cmsis/include" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/integration/nrfx" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/drivers" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/drivers/include" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/hal" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/helpers" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/mdk" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/soc" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/log" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/experimental_section_vars" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/fstorage" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_fifo" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/strerror" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/log/src" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/memobj" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/balloc" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/external/fprintf" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/ringbuf" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/ble/common" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/ble/ble_advertising" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/ble/ble_services/ble_nus" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/ble/nrf_ble_gq" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/ble/nrf_ble_gatt" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/ble/nrf_ble_qwr" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/libraries/timer" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/components/ble/ble_link_ctx_manager" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/src" -I"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/config" -std=gnu11 -Wbad-function-cast -MMD -MP -MF"nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags/nrf_atflags.d" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


