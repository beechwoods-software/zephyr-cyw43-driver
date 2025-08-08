#!/bin/sh

cp webblinky_conf/webblinky_boards_rpi_pico_rp2040_w.conf webblinky/boards/rpi_pico_rp2040_w.conf
cp webblinky_conf/webblinky_boards_rpi_pico_rp2040_w.overlay webblinky/boards/rpi_pico_rp2040_w.overlay
cp webblinky_conf/webblinky_prj.conf webblinky/prj.conf

# Build with this west command:
# west build -p always -b rpi_pico/rp2040/w webblinky -Dwebblinky_SNIPPET=mbedtls -DOPENOCD=/usr/local/bin/openocd -DOPENOCD_DEFAULT_PATH=/usr/local/share/openocd/scripts/ -DRPI_PICO_DEBUG_ADAPTER=cmsis-dap  -DEXTRA_DTC_OVERLAY_FILE=onboarding/boards/rpi_pico_rp2040_w.overlay 2>&1 | tee /tmp/build_rpi_pico_rp2040_w.log
