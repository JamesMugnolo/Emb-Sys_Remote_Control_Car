################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32F4_SBUS-main/rc_input/rc_input_sbus.c 

OBJS += \
./Drivers/STM32F4_SBUS-main/rc_input/rc_input_sbus.o 

C_DEPS += \
./Drivers/STM32F4_SBUS-main/rc_input/rc_input_sbus.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F4_SBUS-main/rc_input/rc_input_sbus.o: ../Drivers/STM32F4_SBUS-main/rc_input/rc_input_sbus.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32F4_SBUS-main/rc_input -I../Drivers/BSP/STM32F413H-Discovery -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F4_SBUS-main/rc_input/rc_input_sbus.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

