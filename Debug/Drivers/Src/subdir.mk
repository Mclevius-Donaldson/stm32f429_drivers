################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f429_gpio_driver.c 

OBJS += \
./Drivers/Src/stm32f429_gpio_driver.o 

C_DEPS += \
./Drivers/Src/stm32f429_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f429_gpio_driver.o: ../Drivers/Src/stm32f429_gpio_driver.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F429ZITx -DSTM32F4 -DNUCLEO_F429ZI -c -I../Inc -I"C:/Users/Ben Mia/OneDrive/Documents/Projects/STM32 Learning/New folder/stm32f429_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f429_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

