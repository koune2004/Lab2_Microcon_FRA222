################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/BasicMathFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/BayesFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/CommonTables" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/ComplexMathFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/ControllerFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/DistanceFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/FastMathFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/FilteringFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/InterpolationFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/MatrixFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/QuaternionMathFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/StatisticsFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/SupportFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/SVMFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/TransformFunctions" -I"C:/Users/theet/Documents/GitHub/Lab2_Microcon_FRA222/Lab2_Microcon_FRA222/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

