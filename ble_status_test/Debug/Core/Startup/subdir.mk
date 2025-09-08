################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l100c6ux.s 

OBJS += \
./Core/Startup/startup_stm32l100c6ux.o 

S_DEPS += \
./Core/Startup/startup_stm32l100c6ux.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -DSTM32L100xC -c -IC:/Users/mds/STM32Cube/Repository/STM32Cube_FW_L1_V1.10.5/Drivers/CMSIS/Device/ST/STM32L1xx/Include -IC:/Users/mds/STM32Cube/Repository/STM32Cube_FW_L1_V1.10.5/Drivers/CMSIS/Include -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l100c6ux.d ./Core/Startup/startup_stm32l100c6ux.o

.PHONY: clean-Core-2f-Startup

