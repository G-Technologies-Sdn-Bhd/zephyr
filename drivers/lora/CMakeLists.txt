# SPDX-License-Identifier: Apache-2.0

# LoRa drivers depend on the include directories exposed by the loramac-node
# library. Hence, if it exists then the source files are added to that otherwise
# a library with same name is created.
if(TARGET loramac-node)
	set(ZEPHYR_CURRENT_LIBRARY loramac-node)
else()
	zephyr_library_named(loramac-node)
endif()

# zephyr_library_sources_ifdef(CONFIG_LORA_SHELL shell.c)
zephyr_library_sources_ifdef(CONFIG_HAS_SEMTECH_RADIO_DRIVERS hal_common.c)
zephyr_library_sources_ifdef(CONFIG_HAS_SEMTECH_RADIO_DRIVERS sx12xx_common.c)
zephyr_library_sources_ifdef(CONFIG_LORA_SX127X sx127x.c)
zephyr_library_sources_ifdef(CONFIG_LORA_AT lora_at.c)
if (CONFIG_LORA_SX126X OR CONFIG_LORA_STM32WL_SUBGHZ_RADIO)
	zephyr_library_sources(sx126x.c)
	zephyr_library_sources_ifdef(CONFIG_LORA_SHELL shell.c)
# else()
elseif (CONFIG_LORA_AT)

  zephyr_library_include_directories(
    ${ZEPHYR_BASE}/drivers/modem
	# option(CONFIG_LORA_SHELL "Enable LoRa shell support" OFF)
  )

	# zephyr_library_sources(lora_at.c)
endif()

zephyr_library_sources_ifdef(CONFIG_LORA_SX126X sx126x_standalone.c)
zephyr_library_sources_ifdef(CONFIG_LORA_STM32WL_SUBGHZ_RADIO sx126x_stm32wl.c)
