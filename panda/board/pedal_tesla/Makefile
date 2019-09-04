# :set noet
PROJ_NAME = comma
OUT = obj



CFLAGS  = -O2 -Wall -std=gnu11 -DPEDAL
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3
CFLAGS += -msoft-float -DSTM32F2 -DSTM32F205xx
CFLAGS += -I ../inc -I ../ -I ../../ -nostdlib
CFLAGS += -T../stm32_flash.ld

STARTUP_FILE = startup_stm32f205xx

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
DFU_UTIL = "dfu-util"

# pedal only uses the debug cert
CERT = ../../certs/debug
CFLAGS += "-DALLOW_DEBUG"
$(shell   mkdir -p $(OUT))
$(shell   tmux kill-session -t comma)

canflash: $(OUT)/$(PROJ_NAME).bin
	../../tests/pedal/enter_canloader.py $<

usbflash: $(OUT)/$(PROJ_NAME).bin
	../../tests/pedal/enter_canloader_tesla.py; sleep 0.5
	PYTHONPATH=../../ python -c "from python import Panda; p = [x for x in [Panda(x) for x in Panda.list()] if x.bootstub]; assert(len(p)==1); p[0].flash('$(OUT)/$(PROJ_NAME).bin', reconnect=False)"

recover: $(OUT)/bootstub.bin $(OUT)/$(PROJ_NAME).bin
	../../tests/pedal/enter_canloader_tesla.py --recover; sleep 0.5
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000 -D $(OUT)/$(PROJ_NAME).bin
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08000000:leave -D $(OUT)/bootstub.bin

tesla: $(OUT)/$(PROJ_NAME).bin
	../../tests/pedal/enter_canloader_tesla.py $<

can2: $(OUT)/$(PROJ_NAME).bin
	../../tests/pedal/enter_canloader_can2.py $<

tesla2: $(OUT)/$(PROJ_NAME).bin
	../../tests/pedal/enter_canloader_tesla2.py $<


include ../../common/version.mk


$(OUT)/cert.h: ../../crypto/getcertheader.py
	../../crypto/getcertheader.py ../../certs/debug.pub ../../certs/release.pub > $@

$(OUT)/main.o: main.c ../*.h 
	mkdir -p $(OUT)
	$(CC) $(CFLAGS) -o $@ -c $<

$(OUT)/bootstub.o: ../bootstub.c ../*.h $(OUT)/gitversion.h $(OUT)/cert.h 
	mkdir -p $(OUT)
	mkdir -p ../$(OUT)
	cp $(OUT)/gitversion.h ../$(OUT)/gitversion.h
	cp $(OUT)/cert.h  ../$(OUT)/cert.h
	$(CC) $(CFLAGS) -o $@ -c $<

$(OUT)/$(STARTUP_FILE).o: ../$(STARTUP_FILE).s
	$(CC) $(CFLAGS) -o $@ -c $<

$(OUT)/%.o: ../../crypto/%.c
	$(CC) $(CFLAGS) -o $@ -c $<

$(OUT)/$(PROJ_NAME).bin: $(OUT)/$(STARTUP_FILE).o $(OUT)/main.o 
  # hack
	$(CC) -Wl,--section-start,.isr_vector=0x8004000 $(CFLAGS) -o $(OUT)/$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary $(OUT)/$(PROJ_NAME).elf $(OUT)/code.bin
	SETLEN=1 ../../crypto/sign.py $(OUT)/code.bin $@ $(CERT)

$(OUT)/bootstub.bin: $(OUT)/$(STARTUP_FILE).o $(OUT)/bootstub.o $(OUT)/sha.o $(OUT)/rsa.o 
	$(CC) $(CFLAGS) -o $(OUT)/bootstub.$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary $(OUT)/bootstub.$(PROJ_NAME).elf $@


	
clean:
	rm -f $(OUT)/*

