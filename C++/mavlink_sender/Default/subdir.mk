################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../mavlinkWriter.cpp \
../mavlink_sender.cpp 

OBJS += \
./mavlinkWriter.o \
./mavlink_sender.o 

CPP_DEPS += \
./mavlinkWriter.d \
./mavlink_sender.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/Users/Maverick/Documents/workspace/TrackingSetup_Thomas/includes/mavlink/v1.0" -I/Users/Maverick/Documents/workspace/Libraries/libftdi1-1.2/src -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


