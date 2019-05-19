KERNELDIR := ../lis3dh_tools/linux
ARM_COMPILER := ../lis3dh_tools/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-
ARCH := arm
PWD := ${shell pwd}

CC=$(ARM_COMPILER)gcc
LD=$(ARM_COMPILER)ld

obj-m := lxdriver.o
lxdriver-objs := lis3dh.o

.PHONY: clean

all:
	make -C $(KERNELDIR) ARCH=$(ARCH) CROSS_COMPILE=$(ARM_COMPILER) M=$(PWD) modules 

clean:
	make -C $(KERNELDIR) M=$(PWD) clean
