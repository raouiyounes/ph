################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../rfidTests/Vision.cpp \
../rfidTests/expe.cpp 

OBJS += \
./rfidTests/Vision.o \
./rfidTests/expe.o 

CPP_DEPS += \
./rfidTests/Vision.d \
./rfidTests/expe.d 


# Each subdirectory must supply rules for building sources it contributes
rfidTests/%.o: ../rfidTests/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


