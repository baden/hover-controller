.PHONY: clean

CC = arm-none-eabi-gcc
OC = arm-none-eabi-objcopy
OS = arm-none-eabi-size
LD = arm-none-eabi-gcc

DEV_PATH = ./lib/device/mm32spin05

LD_SCRIPT = $(DEV_PATH)/mm32spin05.ld

SRC_DIR = src
SRCS = $(wildcard $(SRC_DIR)/*.c)

ASM_DIR = src
ASMS  = $(wildcard $(SRC_DIR)/*.s)

preOBJ  = $(SRCS:%.c=%.o)
preOBJ += $(ASMS:%.s=%.o)
OBJS = $(preOBJ:$(SRC_DIR)/%=%) startup_mm32spin06xx.o system_mm32spin06xx_s.o

INC  = ./lib/CMSIS

CFLAGS += -mcpu=cortex-m0 
CFLAGS += -mlittle-endian
CFLAGS += -mthumb
CFLAGS += -g

LDFLAGS += -mcpu=cortex-m0
LDFLAGS += -mlittle-endian
LDFLAGS += -mthumb
LDFLAGS += -T $(LD_SCRIPT)
LDFLAGS += -Wl,--gc-section

%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -I $(INC)

%.o: $(DEV_PATH)/%.s
	$(CC) $(CFLAGS) -c $< -I $(INC)

%.o: $(DEV_PATH)/%.c
	$(CC) $(CFLAGS) -c $< -I $(INC)

firmware.elf: $(OBJS) 
	$(LD) $(LDFLAGS) $(OBJ) -o $@ $(OBJS)

firmware.hex: firmware.elf
	$(OC) -Oihex firmware.elf firmware.hex

all: firmware.hex
	@echo "   Size:"	
	$(OS) ./firmware.elf

clean :
	-rm *.o *.elf *.hex *.map

# Print variable values
print-%:
	@echo $* = $($*)
