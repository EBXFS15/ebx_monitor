ARCH:=arm
CROSS_COMPILE:=arm-linux-
KDIR:=/opt/embedded/bbb/kernel/linux-dev-am33x-v3.18


ifneq ($(KERNELRELEASE),)
obj-m := ebx_monitor.o

else
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

endif

