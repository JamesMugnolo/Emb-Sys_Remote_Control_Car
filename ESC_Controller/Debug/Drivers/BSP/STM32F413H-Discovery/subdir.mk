################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery.c \
../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_audio.c \
../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_lcd.c \
../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_psram.c \
../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_qspi.c \
../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_sd.c \
../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_ts.c 

OBJS += \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery.o \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_audio.o \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_lcd.o \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_psram.o \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_qspi.o \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_sd.o \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_ts.o 

C_DEPS += \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery.d \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_audio.d \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_lcd.d \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_psram.d \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_qspi.d \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_sd.d \
./Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery.o: ../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32F4_SBUS-main/rc_input -I../Drivers/BSP/STM32F413H-Discovery -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_audio.o: ../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_audio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32F4_SBUS-main/rc_input -I../Drivers/BSP/STM32F413H-Discovery -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_audio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_lcd.o: ../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_lcd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32F4_SBUS-main/rc_input -I../Drivers/BSP/STM32F413H-Discovery -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_lcd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_psram.o: ../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_psram.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32F4_SBUS-main/rc_input -I../Drivers/BSP/STM32F413H-Discovery -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_psram.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_qspi.o: ../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_qspi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32F4_SBUS-main/rc_input -I../Drivers/BSP/STM32F413H-Discovery -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_qspi.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_sd.o: ../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_sd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32F4_SBUS-main/rc_input -I../Drivers/BSP/STM32F413H-Discovery -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_sd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_ts.o: ../Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_ts.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32F4_SBUS-main/rc_input -I../Drivers/BSP/STM32F413H-Discovery -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/STM32F413H-Discovery/stm32f413h_discovery_ts.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

