################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/soc/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/mdk/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/hal/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/drivers/src/prs/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/drivers/src/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/external/fprintf/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/softdevice/common/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/util/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/timer/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/strerror/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/ringbuf/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/memobj/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/log/src/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/fstorage/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/experimental_section_vars/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/balloc/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_flags/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic_fifo/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/libraries/atomic/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/nrf_ble_qwr/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/nrf_ble_gq/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/nrf_ble_gatt/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/common/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/ble_services/ble_nus/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/ble_radio_notification/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/ble_racp/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/ble_link_ctx_manager/subdir.mk
-include nRF5_SDK_17.1.0_ddde560_custom/components/ble/ble_advertising/subdir.mk
-include nRF52_common/segger_RTT_v788j/Syscalls/subdir.mk
-include nRF52_common/segger_RTT_v788j/RTT/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(CCM_DEPS)),)
-include $(CCM_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CXXM_DEPS)),)
-include $(CXXM_DEPS)
endif
ifneq ($(strip $(C++M_DEPS)),)
-include $(C++M_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := wheelLineRemote_FW
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
wheelLineRemote_FW.hex \

SECONDARY_SIZE += \
wheelLineRemote_FW.siz \


# All Target
all: main-build

# Main-build Target
main-build: wheelLineRemote_FW.elf secondary-outputs

# Tool invocations
wheelLineRemote_FW.elf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU Arm Cross C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -march=armv7e-m -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -munaligned-access -fomit-frame-pointer -O0 -fmessage-length=0 -ffunction-sections -fdata-sections -fno-builtin -fmerge-constants -fno-strict-aliasing -fshort-enums -Wunused -Wuninitialized -Wall -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -Wdouble-promotion -g3 -T ../wheelLineRemote_FW_gcc_nrf52.ld -Xlinker --gc-sections -L"D:/Users/Collin/Documents/Wheel line movers/nrf52_wheelLine_accel/nRF52_wheelLine_accel_SDK17/wheelLineRemote_FW/../nRF5_SDK_17.1.0_ddde560_custom/modules/nrfx/mdk" -Wl,-Map,"wheelLineRemote_FW.map" --specs=nosys.specs -v -mabi=aapcs -o "wheelLineRemote_FW.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

wheelLineRemote_FW.hex: wheelLineRemote_FW.elf makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GNU Arm Cross Create Flash Image'
	arm-none-eabi-objcopy -O ihex "wheelLineRemote_FW.elf"  "wheelLineRemote_FW.hex"
	@echo 'Finished building: $@'
	@echo ' '

wheelLineRemote_FW.siz: wheelLineRemote_FW.elf makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: GNU Arm Cross Print Size'
	arm-none-eabi-size --format=berkeley "wheelLineRemote_FW.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CCM_DEPS)$(C_UPPER_DEPS)$(SECONDARY_SIZE)$(CXXM_DEPS)$(C_DEPS)$(CC_DEPS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(SECONDARY_FLASH)$(ASM_DEPS)$(S_UPPER_DEPS)$(C++M_DEPS)$(CPP_DEPS) wheelLineRemote_FW.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents main-build

-include ../makefile.targets
