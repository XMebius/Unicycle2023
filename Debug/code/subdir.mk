################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/Kalman.c \
../code/balance.c \
../code/pid.c \
../code/servo.c \
../code/talk.c \
../code/tjrc_fontCode.c \
../code/tjrc_st7735.c 

OBJS += \
./code/Kalman.o \
./code/balance.o \
./code/pid.o \
./code/servo.o \
./code/talk.o \
./code/tjrc_fontCode.o \
./code/tjrc_st7735.o 

COMPILED_SRCS += \
./code/Kalman.src \
./code/balance.src \
./code/pid.src \
./code/servo.src \
./code/talk.src \
./code/tjrc_fontCode.src \
./code/tjrc_st7735.src 

C_DEPS += \
./code/Kalman.d \
./code/balance.d \
./code/pid.d \
./code/servo.d \
./code/talk.d \
./code/tjrc_fontCode.d \
./code/tjrc_st7735.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fY:/Unicycle/ReUnicycle/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


