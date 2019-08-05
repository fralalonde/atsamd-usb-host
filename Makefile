# How to compile the examples.
APP = simple
RELEASE = debug
TARGET = thumbv6m-none-eabi

# VID and PID of device to flash firmware to.
USBVID = 239a
USBPID = '001e|801e'

BOSSAC = bossac
SERIAL = $(shell ./find-serial-port $(USBVID) $(USBPID) || echo 'cant-find-serial-port')
OFFSET = 0x2000

OBJCOPY = arm-none-eabi-objcopy

TARGETDIR = target/$(TARGET)
EXAMPLEDIR = $(TARGETDIR)/$(RELEASE)/examples

CARGOFLAGS = --target thumbv6m-none-eabi

.PHONY: all clean cargo-build target/thumbv6m-none-eabi/debug/examples/$(APP)

all: $(EXAMPLEDIR)/$(APP)

clean:
	rm -f $(APP).uf2
	rm -f $(APP).bin
	cargo clean

test:
	cargo test

cargo-build:
	cargo rustc --example $(APP) $(CARGOFLAGS) -- -C link-arg=-Tlink.x

$(EXAMPLEDIR)/$(APP): cargo-build

flash: $(APP).bin $(SERIAL)
	$(BOSSAC) -R -e -w -v -o$(OFFSET) -p$(SERIAL) $<

qemu: $(EXAMPLEDIR)/$(APP)
	qemu-system-arm -d in_asm,int,exec,cpu,guest_errors,unimp -pidfile qemu.pid -cpu cortex-m0 -machine lm3s6965evb -nographic -semihosting-config enable=on,target=native -s -S -kernel $<

gdb: $(EXAMPLEDIR)/$(APP)
	gdb-multiarch -ex "target remote localhost:1234" -ex "break main" -ex "continue" $<

%.bin: $(EXAMPLEDIR)/%
	$(OBJCOPY) -O binary $< $@

# Requires https://github.com/sajattack/uf2conv-rs.git
%.uf2: %.bin
	uf2conv-rs $< --base $(OFFSET) --output $@
