################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/MadgwickAHRS.c \
../src/MahonyAHRS.c \
../src/i2c.c \
../src/main.c \
../src/majvic.c \
../src/syscalls.c \
../src/system_stm32f10x.c \
../src/tm_stm32f4_i2c.c \
../src/tm_stm32f4_mpu6050.c 

OBJS += \
./src/MadgwickAHRS.o \
./src/MahonyAHRS.o \
./src/i2c.o \
./src/main.o \
./src/majvic.o \
./src/syscalls.o \
./src/system_stm32f10x.o \
./src/tm_stm32f4_i2c.o \
./src/tm_stm32f4_mpu6050.o 

C_DEPS += \
./src/MadgwickAHRS.d \
./src/MahonyAHRS.d \
./src/i2c.d \
./src/main.d \
./src/majvic.d \
./src/syscalls.d \
./src/system_stm32f10x.d \
./src/tm_stm32f4_i2c.d \
./src/tm_stm32f4_mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"/home/grigorii/Dev/GyroMan/StdPeriph_Driver/inc" -I"/home/grigorii/Dev/GyroMan/inc" -I"/home/grigorii/Dev/GyroMan/CMSIS/device" -I"/home/grigorii/Dev/GyroMan/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


