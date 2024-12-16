#!/bin/bash

export PATH=/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.2/bin:$PATH

make vmlinuz ARCH=loongarch CROSS_COMPILE=loongarch64-linux-gnu- -j 16
#make modules ARCH=loongarch CROSS_COMPILE=loongarch64-linux-gnu- M=drivers/kernel_test -j4
#make modules ARCH=loongarch CROSS_COMPILE=loongarch64-linux-gnu- M=drivers/irqchip -j4
