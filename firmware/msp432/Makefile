SDK ?= $(CURDIR)/vendor/simplelink_msp432p4_sdk_3_40_01_02/simplelink_msp432p4_sdk_3_40_01_02
DEVICE = $(SDK)/source/ti/devices/msp432p4xx
CC = arm-none-eabi-gcc
FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c11 -Os -g3 -ffreestanding -ffunction-sections -fdata-sections -Wall -Wextra -Werror -D__MSP432P401R__ -DDeviceFamily_MSP432P401x -isystem $(SDK)/source -isystem $(SDK)/source/third_party/CMSIS/Include -Isrc
SOURCES = src/main.c src/motor.c src/diagnostics.c $(DEVICE)/startup_system_files/gcc/startup_msp432p401r_gcc.c $(DEVICE)/startup_system_files/system_msp432p401r.c
LINKER = $(SDK)/examples/nortos/MSP_EXP432P401R/driverlib/gpio_toggle_output/gcc/msp432p401r.lds
LIB = $(DEVICE)/driverlib/gcc/msp432p4xx_driverlib.a
.PHONY: all clean test
all: build/led.elf build/wheels.elf build/control.elf
build:
	mkdir -p $@
build/led.elf: $(SOURCES) src/board.h src/diagnostics.h Makefile | build
	$(CC) $(FLAGS) -DWHEEL_TEST=0 $(SOURCES) -nostdlib -Wl,-e,Reset_Handler,-T,$(LINKER),--gc-sections,-Map,$@.map $(LIB) -lgcc -o $@
	arm-none-eabi-objcopy -O binary $@ build/led.bin
	arm-none-eabi-size $@
build/wheels.elf: $(SOURCES) src/board.h src/diagnostics.h Makefile | build
	$(CC) $(FLAGS) -DWHEEL_TEST=1 $(SOURCES) -nostdlib -Wl,-e,Reset_Handler,-T,$(LINKER),--gc-sections,-Map,$@.map $(LIB) -lgcc -o $@
	arm-none-eabi-objcopy -O binary $@ build/wheels.bin
	arm-none-eabi-size $@
test: | build
	cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -DMOTOR_HOST_TEST -DCONTROL_HOST_TEST -Isrc -Itests -DROBOT_CALIBRATION_HEADER='"calibration_fixture.h"' tests/test_runtime.c src/control/core.c -o build/test_runtime
	./build/test_runtime
	cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -Isrc -Itests -include tests/mock_servo.h -DROBOT_CALIBRATION_HEADER='"calibration_fixture.h"' tests/test_servo.c src/control/servo.c src/control/core.c -o build/test_servo
	./build/test_servo
	cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -Isrc tests/test_core.c src/control/core.c -o build/test_core
	./build/test_core
	cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -Isrc tests/test_diagnostics.c src/diagnostics.c -o build/test_diagnostics
	./build/test_diagnostics
	cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -DMOTOR_HOST_TEST -Isrc -Itests tests/test_motor.c src/motor.c -o build/test_motor
	./build/test_motor
clean:
	rm -rf build

CONTROL_SOURCES = $(wildcard src/control/*.c) src/motor.c src/diagnostics.c $(DEVICE)/startup_system_files/gcc/startup_msp432p401r_gcc.c $(DEVICE)/startup_system_files/system_msp432p401r.c
build/control.elf: $(CONTROL_SOURCES) $(wildcard src/control/*.h) src/board.h Makefile | build
	$(CC) $(FLAGS) -Isrc/control $(CONTROL_SOURCES) -nostdlib -Wl,-e,Reset_Handler,-T,$(LINKER),--gc-sections,-Map,$@.map $(LIB) -lgcc -o $@
	arm-none-eabi-objcopy -O binary $@ build/control.bin
	arm-none-eabi-size $@

# Compile-only fixture exercises code excluded by the uncalibrated defaults.
# Synthetic calibration; deliberately not accepted by flash_msp432.sh.
build/control-fixture.elf: $(CONTROL_SOURCES) $(wildcard src/control/*.h) tests/calibration_fixture.h | build
	$(CC) $(FLAGS) -Isrc/control -Itests -DROBOT_CALIBRATION_HEADER='"calibration_fixture.h"' $(CONTROL_SOURCES) -nostdlib -Wl,-e,Reset_Handler,-T,$(LINKER),--gc-sections $(LIB) -lgcc -o $@
