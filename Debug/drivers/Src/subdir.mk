################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f3xx_gpio_driver.c \
../drivers/Src/stm32f3xx_spi_driver.c \
../drivers/Src/stm32f3xx_timer_driver.c 

OBJS += \
./drivers/Src/stm32f3xx_gpio_driver.o \
./drivers/Src/stm32f3xx_spi_driver.o \
./drivers/Src/stm32f3xx_timer_driver.o 

C_DEPS += \
./drivers/Src/stm32f3xx_gpio_driver.d \
./drivers/Src/stm32f3xx_spi_driver.d \
./drivers/Src/stm32f3xx_timer_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F303VCTx -DSTM32 -DSTM32F3 -DSTM32F3DISCOVERY -c -I../Inc -I"E:/Projects/STMWorkSpace/stm32f3discovery_driver/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f3xx_gpio_driver.cyclo ./drivers/Src/stm32f3xx_gpio_driver.d ./drivers/Src/stm32f3xx_gpio_driver.o ./drivers/Src/stm32f3xx_gpio_driver.su ./drivers/Src/stm32f3xx_spi_driver.cyclo ./drivers/Src/stm32f3xx_spi_driver.d ./drivers/Src/stm32f3xx_spi_driver.o ./drivers/Src/stm32f3xx_spi_driver.su ./drivers/Src/stm32f3xx_timer_driver.cyclo ./drivers/Src/stm32f3xx_timer_driver.d ./drivers/Src/stm32f3xx_timer_driver.o ./drivers/Src/stm32f3xx_timer_driver.su

.PHONY: clean-drivers-2f-Src

