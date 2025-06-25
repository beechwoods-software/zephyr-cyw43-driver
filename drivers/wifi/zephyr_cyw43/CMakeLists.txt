# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_ZEPHYR_CYW43)  
  zephyr_library_include_directories(src/ src/cyw43-driver/src/ src/cyw43-driver/firmware/)

  zephyr_library_sources(
      src/zephyr_cyw43_drv.c
      src/zephyr_cyw43_led_drv.c
      src/zephyr_cyw43_gpio_drv.c
      src/zephyr_cyw43_shell.c
      src/cyw43-driver/src/cyw43_ll.c
      src/cyw43-driver/src/cyw43_ctrl.c
    )    

  # The default now is to use Zephyr's PIO-SPI (the 2nd one)  
  if(CONFIG_USE_PICOWI_PIOSPI)
    zephyr_library_sources(
      src/cyw43_bus_picowi_pio_spi.c
      src/picowi_pio/picowi_pio.c
      src/picowi_pio/picowi_pico.c
    )

  else()
    zephyr_library_sources(
      src/cyw43_bus_pio_spi.c
    )
  endif()

  #Only required if bluetooth is enabled
  set(rp2_shared_bus_dir ${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/src/rp2_common/pico_cyw43_driver/cybt_shared_bus)
  zephyr_library_include_directories_ifdef(CONFIG_BT ${rp2_shared_bus_dir}/)    
  zephyr_library_sources_ifdef(CONFIG_BT src/zephyr_cyw43_bt_hci_drv.c
    ${rp2_shared_bus_dir}/cybt_shared_bus.c
    ${rp2_shared_bus_dir}/cybt_shared_bus_driver.c
  )
  
endif()  
