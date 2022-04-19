################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f413zhtx.s 

OBJS += \
./Core/Startup/startup_stm32f413zhtx.o 

S_DEPS += \
./Core/Startup/startup_stm32f413zhtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f413zhtx.o: ../Core/Startup/startup_stm32f413zhtx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I../Drivers/BSP/STM32F413H-Discovery -I../Drivers/STM32F4_SBUS-main/rc_input -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f413zhtx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

