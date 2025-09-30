# Enable these flags when compiling against a GCOV-enabled kernel
# loadable objects are not compatible with different configured kernels (i.e. gcov) 
# GCOV_PROFILE := y
# CFLAGS +=-ftest-coverage -fprofile-arcs
# export CFLAGS

LINUX_SRC=/usr/src/linux

ifneq ($(KERNELRELEASE),)
#kbuild part of Makefile
include Kbuild
else
# Normal Makefile - The device overlay is compiled outside of the makefile -
# Usually by a bash script running the makefile
all:
	$(MAKE) -C $(LINUX_SRC) M=$$PWD modules
	dtc -@ -I dts -O dtb -o ./arch/arm/boot/dts/tmf8829-overlay-fpc.dtbo ./arch/arm/boot/dts/tmf8829-overlay-fpc.dts
	dtc -@ -I dts -O dtb -o ./arch/arm/boot/dts/tmf8829-overlay-fpc-polled.dtbo ./arch/arm/boot/dts/tmf8829-overlay-fpc-polled.dts
	dtc -@ -I dts -O dtb -o ./arch/arm/boot/dts/tmf8829-overlay-fpc-spi.dtbo ./arch/arm/boot/dts/tmf8829-overlay-fpc-spi.dts
	dtc -@ -I dts -O dtb -o ./arch/arm/boot/dts/tmf8829-overlay-fpc-spi-polled.dtbo ./arch/arm/boot/dts/tmf8829-overlay-fpc-spi-polled.dts

modules:
	$(MAKE) -C $(LINUX_SRC) M=$$PWD $@

clean:
	$(MAKE) -C $(LINUX_SRC) M=$$PWD clean
	$(RM) ./arch/arm/boot/dts/*.dtbo
	$(RM) .tmp*.gcno *.gcov *.gz coverage.info
	$(RM) -rf html sys 
endif
