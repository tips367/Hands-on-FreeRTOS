################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F/%.o: ../Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -I"E:/Study/Projects/Embedded/RTOS_Projects/STM32_HelloWorld/CMSIS/core" -I"E:/Study/Projects/Embedded/RTOS_Projects/STM32_HelloWorld/Config" -I"E:/Study/Projects/Embedded/RTOS_Projects/STM32_HelloWorld/Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"E:/Study/Projects/Embedded/RTOS_Projects/STM32_HelloWorld/Third-Party/FreeRTOS/org/Source/include" -I"E:/Study/Projects/Embedded/RTOS_Projects/STM32_HelloWorld/CMSIS/device" -I"E:/Study/Projects/Embedded/RTOS_Projects/STM32_HelloWorld/inc" -I"E:/Study/Projects/Embedded/RTOS_Projects/STM32_HelloWorld/StdPeriph_Driver/inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


