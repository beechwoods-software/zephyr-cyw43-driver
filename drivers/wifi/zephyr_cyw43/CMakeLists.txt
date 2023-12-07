# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_ZEPHYR_CYW43)
  if(CONFIG_USE_PICOWI_PIOSPI)
    zephyr_library_include_directories(src/ src/cyw43-driver/src/ src/cyw43-driver/firmware/)
    
    zephyr_library_sources(
      src/zephyr_cyw43_drv.c
      src/zephyr_cyw43_led_drv.c
      src/zephyr_cyw43_gpio_drv.c
      src/zephyr_cyw43_shell.c
      src/cyw43-driver/src/cyw43_ll.c
      src/cyw43-driver/src/cyw43_ctrl.c
      src/cyw43_bus_picowi_pio_spi.c
      src/picowi_pio/picowi_pio.c
      src/picowi_pio/picowi_pico.c
    )

  else()
    zephyr_library_include_directories(src/ src/cyw43-driver/src/ src/cyw43-driver/firmware/)
    
    zephyr_library_sources(
      src/zephyr_cyw43_drv.c
      src/zephyr_cyw43_led_drv.c
      src/zephyr_cyw43_gpio_drv.c
      src/zephyr_cyw43_shell.c
      src/cyw43-driver/src/cyw43_ll.c
      src/cyw43-driver/src/cyw43_ctrl.c
      src/cyw43_bus_pio_spi.c
    )
  endif()
endif()  
