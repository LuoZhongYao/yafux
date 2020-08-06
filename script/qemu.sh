#!/bin/bash
qemu-system-gnuarmeclipse --verbose --verbose	\
	--board generic				\
	--mcu STM32F405RG			\
	--gdb tcp::1234	-S				\
	-d unimp,guest_errors --semihosting-config enable=on,target=native $*

