################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Studium_Software/Projektarbeit/_Final_Master_Light/SubGHz_Phy/Target/radio_board_if.c 

OBJS += \
./Application/User/SubGHz_Phy/Target/radio_board_if.o 

C_DEPS += \
./Application/User/SubGHz_Phy/Target/radio_board_if.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/SubGHz_Phy/Target/radio_board_if.o: D:/Studium_Software/Projektarbeit/_Final_Master_Light/SubGHz_Phy/Target/radio_board_if.c Application/User/SubGHz_Phy/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../../Utilities/trace/adv_trace -I../../Utilities/misc -I../../Utilities/sequencer -I../../Utilities/timer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/Third_Party/SubGHz_Phy -I../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../Drivers/CMSIS/Include -I"D:/Studium_Software/Projektarbeit/_Final_Master_Light/Drivers/BSP/STM32WLxx_LoRa_E5_mini" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Application-2f-User-2f-SubGHz_Phy-2f-Target

clean-Application-2f-User-2f-SubGHz_Phy-2f-Target:
	-$(RM) ./Application/User/SubGHz_Phy/Target/radio_board_if.d ./Application/User/SubGHz_Phy/Target/radio_board_if.o ./Application/User/SubGHz_Phy/Target/radio_board_if.su

.PHONY: clean-Application-2f-User-2f-SubGHz_Phy-2f-Target

