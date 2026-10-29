# 本机执行 make：编译 my_pciedriver.c，生成 my_pciedriver.ko。
# Kbuild 进入本目录时，只声明要生成的模块。
ifneq ($(KERNELRELEASE),)
obj-m := my_pciedriver.o
else

# 默认使用板子正在运行的内核；也可 make KERNEL_DIR=/path/to/kernel。
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
# 本工程面向 ARM64 板子，本机编译不需要交叉编译前缀。
# 如在其他主机交叉编译，显式传入 CROSS_COMPILE=aarch64-linux-gnu-。
ARCH ?= arm64
CROSS_COMPILE ?=

.PHONY: all modules clean
all: modules

modules:
	$(MAKE) -C "$(KERNEL_DIR)" M="$(CURDIR)" ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

# 仅清理模块产物，不清理 Qt 的 build 目录。
clean:
	$(MAKE) -C "$(KERNEL_DIR)" M="$(CURDIR)" ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean

endif
