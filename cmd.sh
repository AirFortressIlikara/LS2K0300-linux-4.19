export PATH=~/loongson/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.3-1/bin:$PATH 
make vmlinuz ARCH=loongarch CROSS_COMPILE=loongarch64-linux-gnu- -j 42
cksum vmlinuz

