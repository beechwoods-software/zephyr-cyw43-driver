# cyw43-driver module for the Zephyr OS

This is an integration of the georgerobotics cyw43-driver code for use with the Zephyr OS and its native IP stack. It currently only contains board support for Raspberry Pi Pico hardware, but can easily be extended to others. 

## Building a sample Zephyr image
### Set up Zephyr development environment
In order to build a Zephyr image that contains cyw43-driver WiFi support, it is necessary to have a Zephyr development environment. If you have not done this, follow the Zephyr project's Getting Started Guide: https://docs.zephyrproject.org/latest/develop/getting_started/index.html

### Build the example application
Once you've set up the Zephyr development environment, you can use the **west** tool and code in this repository to build a Raspberry Pi Pico image with cyw43-driver WiFi support by following these steps:
```
  cd <directory where you want your zephyr build workspace>
  west init -m https://github.com/beechwoods-software/zephyr-cyw43-driver --mr main my-workspace
  cd my-workspace
  (cd zephyr-cyw43-driver; git submodule init; git submodule update)
  west update
  cd zephyr-cyw43-driver
  west build -b rpi_pico_w app
```
Once the build is finished, you should find a uf2 image **cyw43-driver/zephyr_buildbuild/zephyr/zephyr.uf2** that can be flashed onto the device using the instructions here: https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/3 The build process will also produce images in other formats (the elf format image is useful if you want to run your code in a debugger).

The image will run the Zephyr network shell on the serial console, which can be used to set up a wifi connection with the command:
```
  uart:~$ wifi connect <your ssid> <your passphrase>
```

You can see the full set of wifi subcommands, which can be used to set up an access point or perform other wifi management and control functions, by issuing the **wifi** command with no arguments. 

### Autoconnecting to your wifi network
If you wish for your image to automatically connect to your own wifi network, uncomment the two lines in **app/local.conf** and fill them in with your wifi credentials.

### Providing support for other boards
In order to provide support for other boards, a device tree source overlay file will need to be provided to tell the build how the cyw43 module is connected to the system (SDIO or SPI, and which particular pins to be used for that connection).

The overlay file for the Pico W can be used as an example:
```
cyw43-driver/zephyr_build/app/boards/rpi_pico_w.overlay
```

