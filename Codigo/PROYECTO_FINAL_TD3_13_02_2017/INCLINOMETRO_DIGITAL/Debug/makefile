################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: INCLINOMETRO_DIGITAL.axf

# Tool invocations
INCLINOMETRO_DIGITAL.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\acafa\Desktop\FACU\PROYECTO_TD3\lpc_board_nxp_lpcxpresso_1769\Debug" -L"C:\Users\acafa\Desktop\FACU\PROYECTO_TD3\lpc_chip_175x_6x\Debug" -L"C:\Users\acafa\Desktop\FACU\PROYECTO_TD3\freertos\Debug" -Xlinker -Map="INCLINOMETRO_DIGITAL.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m3 -mthumb -T "INCLINOMETRO_DIGITAL_Debug.ld" -o "INCLINOMETRO_DIGITAL.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) INCLINOMETRO_DIGITAL.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "INCLINOMETRO_DIGITAL.axf"; # arm-none-eabi-objcopy -v -O binary "INCLINOMETRO_DIGITAL.axf" "INCLINOMETRO_DIGITAL.bin" ; # checksum -p LPC1769 -d "INCLINOMETRO_DIGITAL.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
