################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32f303retx.s 

OBJS += \
./Startup/startup_stm32f303retx.o 

S_DEPS += \
./Startup/startup_stm32f303retx.d 


# Each subdirectory must supply rules for building sources it contributes
Startup/%.o: ../Startup/%.s Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/piotr/OneDrive/Pulpit/ARM_Cortex/F303RE_DRIVERS/Inc" -I"C:/Users/piotr/OneDrive/Pulpit/ARM_Cortex/F303RE_DRIVERS/Drivers/Inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Startup

clean-Startup:
	-$(RM) ./Startup/startup_stm32f303retx.d ./Startup/startup_stm32f303retx.o

.PHONY: clean-Startup
