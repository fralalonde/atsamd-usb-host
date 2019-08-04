APP = simple

USBVID = 239a
USBPID = '001e|801e'

BOSSAC = bossac
SERIAL = $(shell ./find-serial-port $(USBVID) $(USBPID) || echo 'cant-find-serial-port')
OFFSET = 0x2000

.PHONY: all clean cargo-build target/thumbv6m-none-eabi/debug/examples/$(APP)

all: $(APP).uf2

clean:
	rm -f $(APP).uf2
	rm -f $(APP).bin
	cargo clean

cargo-build:
	cargo build --example $(APP)

target/thumbv6m-none-eabi/debug/examples/$(APP): cargo-build

$(APP).bin: target/thumbv6m-none-eabi/debug/examples/$(APP)
	cargo objcopy --example $(APP) -- -O binary $(APP).bin

# Requires https://github.com/sajattack/uf2conv-rs.git
%.uf2: %.bin
	uf2conv-rs $< --base $(OFFSET) --output $@

flash: $(APP).bin $(SERIAL)
	$(BOSSAC) -R -e -w -v -o$(OFFSET) -p$(SERIAL) $<

qemu: target/thumbv6m-none-eabi/debug/$(APP)
	qemu-system-arm -d in_asm,int,exec,cpu,guest_errors,unimp -pidfile qemu.pid -cpu cortex-m0 -machine lm3s6965evb -nographic -semihosting-config enable=on,target=native -s -S -kernel $<

gdb: target/thumbv6m-none-eabi/debug/$(APP)
	gdb-multiarch -ex "target remote localhost:1234" -ex "break main" -ex "continue" $<
