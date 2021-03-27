EXECUTABLE=stm32f0_examples.elf
BIN=$(EXECUTABLE:.elf=.bin)

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
GDB=arm-none-eabi-gdb
SIZE=arm-none-eabi-size

CFLAGS=-g -O2 -mlittle-endian -mfloat-abi=soft -mthumb -flto
CFLAGS+=-mcpu=cortex-m0
CFLAGS+=--specs=nano.specs --specs=nosys.specs -fno-builtin-printf -u _printf_float
CFLAGS+=-Wall -Wno-address-of-packed-member -fno-strict-aliasing
CFLAGS+=-D USE_FULL_LL_DRIVER \
	-D STM32F051x8

CFLAGS+=-Wl,-T,./stm32f051r8_flash.ld

LDFLAGS+=-Wl,--start-group -lm -Wl,--end-group

SRC=

SRC+=./system_stm32f0xx.c

SRC+=./lib/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_rcc.c \
	./lib/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_gpio.c \
	./lib/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_usart.c \
	./lib/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_i2c.c \
	./lib/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_spi.c \
	./lib/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_tim.c \
	./lib/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_utils.c

SRC+=./isr.c \
	./main.c

CFLAGS+=-I./lib/CMSIS/Include
CFLAGS+=-I./lib/CMSIS/Device/ST/STM32F0xx/Include
CFLAGS+=-I./lib/STM32F0xx_HAL_Driver/Inc
CFLAGS+=-I./

STARTUP=./startup_stm32f051x8.s
STARTUP_OBJ=./startup_stm32f051x8.o

OBJS=$(SRC:.c=.o)
DEPEND=$(SRC:.c=.d)

all:$(BIN)

$(EXECUTABLE): $(STARTUP_OBJ) $(OBJS)
	@echo "LD" $@
	@$(CC) $(CFLAGS) $(OBJS) $(STARTUP_OBJ) $(LDFLAGS) -o $@

-include $(DEPEND)

%.bin: %.elf
	@echo 'OBJCOPY $@'
	@$(OBJCOPY) -O binary $^ $@

%.o: %.s 
	@echo "CC" $@
	@$(CC) $(CFLAGS) -c $< $(LDFLAGS) -o $@

%.o: %.c
	@echo "CC" $@
	@$(CC) $(CFLAGS) -MMD -MP -c $< $(LDFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN)
	rm -rf $(OBJS)
	rm -rf $(STARTUP_OBJ)
	rm -rf $(DEPEND)

flash:
	openocd -f interface/stlink.cfg \
	-f target/stm32f0x.cfg \
	-c "init" \
	-c "reset init" \
	-c "halt" \
	-c "flash write_image erase $(EXECUTABLE)" \
	-c "verify_image $(EXECUTABLE)" \
	-c "reset run" -c shutdown

openocd:
	openocd -s /opt/openocd/share/openocd/scripts/ -f ./gdb/openocd.cfg

gdbauto:
	cgdb -d $(GDB) -x ./gdb/openocd_gdb.gdb

size:
	$(SIZE)  $(EXECUTABLE)

.PHONY:all clean flash openocd gdbauto
