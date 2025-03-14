#!/bin/bash
###
 # @Author: ilikara 3435193369@qq.com
 # @Date: 2025-02-27 08:27:57
 # @LastEditors: ilikara 3435193369@qq.com
 # @LastEditTime: 2025-03-14 14:06:52
 # @FilePath: /LS2K0300-linux-4.19/cmd.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 
export PATH=/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6/bin:$PATH 
make vmlinuz ARCH=loongarch CROSS_COMPILE=loongarch64-linux-gnu- -j$(nproc)
cksum vmlinuz

