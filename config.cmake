option(CONFIG_LOGGING "Enable logging" ON)
if (CONFIG_LOGGING)
	set(CONFIG_LOGGING_LEVEL 3 CACHE STRING "Set log output level")
	option(CONFIG_LOGGING_LED "Use LED logging backend" OFF)
	option(CONFIG_LOGGING_USART "Use USART logging backed" OFF)
	option(CONFIG_LOGGING_SEMIHOSTING "Use semihosting logging backed" ON)
endif(CONFIG_LOGGING)