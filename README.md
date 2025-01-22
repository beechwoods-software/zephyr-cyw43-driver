# cyw43-driver module for the Zephyr OS
## Important notice: WiFi support for the CYW43 chip on the Raspberry Pi Pico W is now officially provided via a driver that was provided to the Zephyr project by Infineon and integrated by Beechwoods. When you build Zephyr for the Pico W on the latest main branch, you will get WiFi support via that driver by default. While most people will probably choose to use the officially supported Infineon driver, I will continue to try to keep this alternate implementation from Georgerobotics up to date for those who still need Pico W WiFi on Zephyr 3.6/3.7 and also for those who want an alternate implmentation. I will also try to add a Zephyr binding for the Bluetooth functionality in this driver soon.

# Introduction
This is an integration of the [Georgerobotics cyw43-driver](https://github.com/georgerobotics) (the same WiFi driver that's included in the RPi Pico SDK for use with the Zephyr OS and its native IP stack. It currently only contains board support for Raspberry Pi Pico hardware, but can easily be extended to others.

The earlier versions of this project (for Zephyr 3.x used the [picowi](https://github.com/jbentham/picowi) implementation of the SPI-PIO interface. [Beechwoods Software](https://github.com/beechwoods-software) has now contibuted a native implementation of SPI-PIO to the Zephyr project, so the Zephyr SPI-PIO driver will be used for Zephyr 4.0 and beyond.

## Building a sample Zephyr image
### Set up Zephyr development environment
In order to build a Zephyr image that contains cyw43-driver WiFi support, it is necessary to have a Zephyr development environment. If you have not done this, follow the Zephyr project's Getting Started Guide: https://docs.zephyrproject.org/latest/develop/getting_started/index.html

### Build the example application
Once you've set up the Zephyr development environment, you can use the **west** tool and code in this repository to build a Raspberry Pi Pico image with cyw43-driver WiFi support by following the step below. These instructions will do an out-of-tree build of the tip of the Zephyr main branch. To build for Zephyr 3.6 or Zephyr 3.7, replace '--mr main' with '--mr v3.6.0' or '--mr v3.7.1' (you may also want to look at the README associated with those tags for subtly different build instructions).
```
  cd <directory where you want your zephyr build workspace>
  west init -m https://github.com/beechwoods-software/zephyr-cyw43-driver --mr main my-workspace
  cd my-workspace
  (cd zephyr-cyw43-driver; git submodule init; git submodule update)
  west update
  cd zephyr-cyw43-driver
  west build -b rpi_pico/rp2040/w app
```
Once the build is finished, you should find a uf2 image **cyw43-driver/zephyr_buildbuild/zephyr/zephyr.uf2** that can be flashed onto the device using the instructions here: https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/3 The build process will also produce images in other formats (the elf format image is useful if you want to run your code in a debugger).

The image will run the Zephyr network shell on the serial console, which can be used to set up a wifi connection with the command:
```
  uart:~$ wifi connect -s <your ssid> -p <your passphrase> -k 1
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

