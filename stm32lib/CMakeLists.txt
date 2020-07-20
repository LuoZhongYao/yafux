enable_language(ASM)

add_library(stm32)
target_compile_definitions(stm32 PUBLIC USE_STDPERIPH_DRIVER=1 -DSTM32F10X_HD)
target_include_directories(stm32 PUBLIC
	STM32F10x_StdPeriph_Driver/inc
	CMSIS/CM3)
target_sources(stm32 PRIVATE 
	CMSIS/CM3/core_cm3.c
	CMSIS/CM3/system_stm32f10x.c
	CMSIS/CM3/startup_stm32f10x_hd.s

	STM32F10x_StdPeriph_Driver/src/misc.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
	STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c
	)
