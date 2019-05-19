KERNELDIR := ../lis3dh_tools/linux
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
