################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/freertos/fsl_usart_freertos.c 

OBJS += \
./drivers/freertos/fsl_usart_freertos.o 

C_DEPS += \
./drivers/freertos/fsl_usart_freertos.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/freertos/%.o: ../drivers/freertos/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__LPC5411X__ -DCORE_M4 -DFSL_RTOS_FREE_RTOS -DCPU_LPC54114J256BD64 -DCPU_LPC54114J256BD64_cm4 -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -D__MULTICORE_MASTER -I../board -I../source -I../ -I../drivers -I../CMSIS -I../device -I../drivers/freertos -I../amazon-freertos/include -I../amazon-freertos/FreeRTOS/portable -I../startup -I../component/serial_manager -I../utilities -I../component/uart -I../component/lists -O0 -fno-common -g3 -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


