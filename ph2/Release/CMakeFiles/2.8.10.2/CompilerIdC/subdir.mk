################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMakeFiles/2.8.10.2/CompilerIdC/CMakeCCompilerId.c 

OBJS += \
./CMakeFiles/2.8.10.2/CompilerIdC/CMakeCCompilerId.o 

C_DEPS += \
./CMakeFiles/2.8.10.2/CompilerIdC/CMakeCCompilerId.d 


# Each subdirectory must supply rules for building sources it contributes
CMakeFiles/2.8.10.2/CompilerIdC/%.o: ../CMakeFiles/2.8.10.2/CompilerIdC/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


