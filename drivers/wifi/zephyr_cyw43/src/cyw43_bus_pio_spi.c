/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2023 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define CYW43_USE_DMA 0

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "cyw43.h"
#include "cyw43_internal.h"
#include "cyw43_spi.h"
#include "cyw43_debug_pins.h"

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "hardware/gpio.h"

#define DT_DRV_COMPAT infineon_cyw43

#if CYW43_SPI_PIO

#if !CYW43_USE_SPI
#error CYW43_USE_SPI should be true
#endif

/* Set to 1 to enable */
#if ENABLE_SPI_DUMPING
#if 0
#define DUMP_SPI_TRANSACTIONS(A) A
#else
static bool enable_spi_packet_dumping=true; /* set to true to dump */
#define DUMP_SPI_TRANSACTIONS(A) if (enable_spi_packet_dumping) {A}
#endif

static uint32_t counter = 0;
#else
#define DUMP_SPI_TRANSACTIONS(A)
#endif

#undef SWAP32
__force_inline static uint32_t __swap16x2(uint32_t a) {
        __asm ("rev16 %0, %0" : "+l" (a) : : );
        return a;
}
#define SWAP32(a) __swap16x2(a)

#if ENABLE_SPI_DUMPING
static void dump_bytes(const uint8_t *bptr, uint32_t len) {
        unsigned int i = 0;

        for (i = 0; i < len;) {
                if ((i & 0x07) == 0) {
                        printf(" ");
                }
                printf("%02x ", bptr[i++]);
        }
        printf("\n");
}
#endif

#define WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE        (4)


struct cyw43_pio_spi_data {
        const struct cyw43_wifi_config *cfg;
};

#define PINCTRL_STATE_HOST_WAKE PINCTRL_STATE_PRIV_START
PINCTRL_DT_INST_DEFINE(0);

#define WLAN_CBUCK_DISCHARGE_MS 10
#define WLAN_POWER_UP_DELAY_MS 250

#define CYW43_WIFI_SPI_OPERATION (SPI_WORD_SET(DT_PROP_OR(DT_DRV_INST(0), spi_word_size, 8)) \
                                  | SPI_HALF_DUPLEX \
                                  | SPI_TRANSFER_MSB)

struct cyw43_wifi_config cyw43_wifi_config = {
        .bus_spi = SPI_DT_SPEC_GET(DT_DRV_INST(0), CYW43_WIFI_SPI_OPERATION, 0),

        .wl_on_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), wl_on_gpios, {0}),
        .host_wake_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(infineon_cyw43_module), host_wake_gpios),
        .bus_select_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), bus_select_gpios, {0}),

        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

static struct cyw43_pio_spi_data cyw43_pio_spi0;

int cyw43_spi_init(cyw43_int_t *self) {

        struct cyw43_pio_spi_data *spi = &cyw43_pio_spi0; /* Static instance */
        struct cyw43_wifi_config *config = &cyw43_wifi_config; /* Static instance */
        
        printf("\n\nRunning with the Zephyr PIO-SPI low level interface\n\n\n");

        int ret;

	/* Pull bus select line low before enabling WiFi chip */
	gpio_pin_configure_dt(&config->bus_select_gpio, GPIO_OUTPUT_INACTIVE);

        /* power wifi on */
        if (!device_is_ready(config->wl_on_gpio.port)) {
            printk("Error: failed to configure wifi_reg_on %s pin %d\n",
                   config->wl_on_gpio.port->name, config->wl_on_gpio.pin);
            return -1;
        }

        ret = gpio_pin_configure_dt(&config->wl_on_gpio, GPIO_OUTPUT);
        if (ret) {
            printk("Error %d: failed to configure wifi_reg_on %s pin %d\n", ret,
                   config->wl_on_gpio.port->name, config->wl_on_gpio.pin);
            return ret;
        }

        ret = gpio_pin_set_dt(&config->wl_on_gpio, 0);
	if (ret) {
            printk("gpio_pin_set_dt(&config->wl_on_gpio, 0) returned %d\n", ret);
            return ret;
	}

	/* Allow CBUCK regulator to discharge */
	k_msleep(WLAN_CBUCK_DISCHARGE_MS);

	/* WIFI power on */
	ret = gpio_pin_set_dt(&config->wl_on_gpio, 1);
	if (ret) {
            printk("gpio_pin_set_dt(&config->wl_on_gpio, 1) returned %d\n", ret);
            return ret;
	}
	k_msleep(WLAN_POWER_UP_DELAY_MS);
        

        if (!spi_is_ready_dt(&config->bus_spi)) {
		printk("SPI device is not ready\n");
		return -1;
	}
        
        cyw43_spi_irq_enable(config, true);


        /* Only does something if CYW43_LOGIC_DEBUG=1 */
        logic_debug_init();

        spi->cfg = config;
        assert(!self->bus_data);
        self->bus_data = spi;

        return 0;
}

void cyw43_spi_deinit(cyw43_int_t *self) {
        return;
}

int cyw43_spi_transfer(cyw43_int_t *self, const uint8_t *tx, size_t tx_length, uint8_t *rx,
                       size_t rx_length) {
        int rv = -CYW43_EINVAL;
        struct cyw43_pio_spi_data *bus_data = (struct cyw43_pio_spi_data *)self->bus_data;

        struct cyw43_wifi_config *cfg = &cyw43_wifi_config;

        /*
        CYW43_VDEBUG("Calling cyw43_spi_transfer(self=0x%lx, tx=0x%lx, tx_length=%d, rx=0x%lx, rx_length=%d)\n",
                     (unsigned long)self, (unsigned long)tx, (unsigned int)tx_length,
                     (unsigned long)rx, (unsigned int)rx_length);
        */
        
        cyw43_spi_irq_enable(cfg, false);

        if ((tx == NULL) && (rx == NULL)) {
                return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
        }

	if (tx == NULL && tx_length > 0 && rx_length >= tx_length) {
		tx = rx;
	}
    

	const struct spi_buf tx_buf = {.buf = (uint8_t *)tx, .len = tx_length};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf[2];
        struct spi_buf_set rx_set = {.buffers = rx_buf, .count = 2};

	if (rx != NULL) {
		rx += tx_length;
	}
        
	if (rx_length >= tx_length) {
		rx_length -= tx_length;
	} else {
		rx_length = 0;
	}
        
        rx_buf[0].buf = rx;
        rx_buf[0].len = rx_length;
        rx_set.count = 1;
        
        /*
        CYW43_VDEBUG("Calling spi_transceive_dt(0x%lx, rx_buf.len=%d, tx_buf.len=%d)\n",
                     (long unsigned) &bus_data->cfg->bus_spi, tx_set.buffers[0].len, rx_set.buffers[0].len);
        */
        DUMP_SPI_TRANSACTIONS(
                printf("TXed:");
                dump_bytes(tx, tx_length);
                printf("\n");
        )

	rv = spi_transceive_dt(&bus_data->cfg->bus_spi, &tx_set, &rx_set);
        if (rv != 0) {
            printk("rv = %d\n", rv);
            CYW43_VDEBUG("rv = %d\n", rv);
        }

        DUMP_SPI_TRANSACTIONS(
                printf("RXed:");
                dump_bytes(rx, rx_length);
                printf("\n");
        )

        cyw43_spi_irq_enable(cfg, true);
        
        //CYW43_VDEBUG("spi_transceive_dt() returned %d\n", rv);
        
        return rv;
}


void cyw43_spi_irq_enable(struct cyw43_wifi_config *cfg, bool enable) {
    int ret;
    //printk("cyw43_spi_irq_enable(%s)\n", enable ? "true" : "false");

    if (enable) {
        ret = pinctrl_apply_state(cyw43_wifi_config.pcfg, PINCTRL_STATE_HOST_WAKE);
        if (ret) {
            printk("could not apply pinconfig HOST_WAKE, return code %d\n", ret);
        }

        ret = gpio_pin_interrupt_configure_dt(&cyw43_wifi_config.host_wake_gpio, GPIO_INT_EDGE_FALLING);
        if (ret) {
                printk("Unable to configure GPIO pin %u\n", cyw43_wifi_config.host_wake_gpio.pin);
        }
        gpio_pin_interrupt_configure_dt(&cfg->host_wake_gpio, (GPIO_INT_ENABLE|GPIO_INT_LEVEL_HIGH));

    } else {
        gpio_pin_interrupt_configure_dt(&cfg->host_wake_gpio, GPIO_INT_DISABLE);
        ret = pinctrl_apply_state(cyw43_wifi_config.pcfg, PINCTRL_STATE_DEFAULT);
        if (ret) {
            printk("could not apply pinconfig DEFAULT, return code %d\n", ret);
        }        
    }
 
    return;
}

/* Initialise our gpios */
void cyw43_spi_gpio_setup(void) {
        return;
}

/* Reset wifi chip */
void cyw43_spi_reset(void) {
        return;
}

static inline uint32_t make_cmd(bool write, bool inc, uint32_t fn, uint32_t addr, uint32_t sz) {
        return write << 31 | inc << 30 | fn << 28 | (addr & 0x1ffff) << 11 | sz;
}

#if CYW43_VERBOSE_DEBUG
static const char *func_name(int fn) {
        switch (fn)
        {
        case BUS_FUNCTION:
                return "BUS_FUNCTION";
        case BACKPLANE_FUNCTION:
                return "BACKPLANE_FUNCTION";
        case WLAN_FUNCTION:
                return "WLAN_FUNCTION";
        default:
                return "UNKNOWN";
        }
}
#endif

uint32_t read_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
        uint32_t buf[2] = {0};
        //CYW43_VDEBUG("Calling read_reg_u32_swap()");        
        assert(fn != BACKPLANE_FUNCTION);
        buf[0] = make_cmd(false, true, fn, reg, 4);
        CYW43_VDEBUG("command=0x%08X\n", buf[0]);
        buf[0] = SWAP32(buf[0]);
        CYW43_VDEBUG("swapped=0x%08X\n", buf[0]);
        int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 4, (uint8_t *)buf, 8);
        if (ret != 0) {
                return ret;
        }
        CYW43_VDEBUG("buf[0]: 0x%08X, swapped: 0x%08X\n",
                     buf[0], SWAP32(buf[0]));
        CYW43_VDEBUG("buf[1]: 0x%08X, swapped: 0x%08X\n",
                     buf[1], SWAP32(buf[1]));
        return SWAP32(buf[1]);
}

static inline uint32_t _cyw43_read_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint size) {
        /* Padding plus max read size of 32 bits + another 4? */
        static_assert(CYW43_BACKPLANE_READ_PAD_LEN_BYTES % 4 == 0, "");
        int index = (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4) + 1 + 1;
        uint32_t buf32[index];
        uint8_t *buf = (uint8_t *)buf32;
        const uint32_t padding = (fn == BACKPLANE_FUNCTION) ? CYW43_BACKPLANE_READ_PAD_LEN_BYTES : 0; // Add response delay
        buf32[0] = make_cmd(false, true, fn, reg, size + padding);

        if (fn == BACKPLANE_FUNCTION) {
                logic_debug_set(pin_BACKPLANE_READ, 1);
        }
        int ret = cyw43_spi_transfer(self, NULL, 4, buf, 8 + padding);
        if (fn == BACKPLANE_FUNCTION) {
                logic_debug_set(pin_BACKPLANE_READ, 0);
        }

        if (ret != 0) {
                return ret;
        }
        uint32_t result = buf32[padding > 0 ? index - 1 : 1];
        CYW43_VDEBUG("cyw43_read_reg_u%d %s 0x%lx=0x%lx\n", size * 8,
                     func_name(fn), (unsigned long)reg, (unsigned long)result);
        return result;
}

uint32_t cyw43_read_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
        return _cyw43_read_reg(self, fn, reg, 4);
}

int cyw43_read_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
        return _cyw43_read_reg(self, fn, reg, 2);
}

int cyw43_read_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
        return _cyw43_read_reg(self, fn, reg, 1);
}

/* This is only used to switch the word order on boot */
int write_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
    //CYW43_VDEBUG("Calling write_reg_u32_swap()");
        uint32_t buf[2];
        /* Boots up in little endian so command needs swapping too */
        buf[0] = SWAP32(make_cmd(true, true, fn, reg, 4));
        buf[1] = SWAP32(val);
        int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 8, NULL, 0);
        CYW43_VDEBUG("write_reg_u32_swap %s 0x%lx=0x%lx\n",
                     func_name(fn), (unsigned long)reg, (unsigned long)val);
        return ret;
}

static inline int _cyw43_write_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val, uint size) {
    //CYW43_VDEBUG("Calling  _cyw43_write_reg()");    
        uint32_t buf[2];
        buf[0] = make_cmd(true, true, fn, reg, size);
        buf[1] = val;
        if (fn == BACKPLANE_FUNCTION) {
                /* In case of f1 overflow */
                self->last_size = 8;
                self->last_header[0] = buf[0];
                self->last_header[1] = buf[1];
                self->last_backplane_window = self->cur_backplane_window;
        }

        if (fn == BACKPLANE_FUNCTION) {
                logic_debug_set(pin_BACKPLANE_WRITE, 1);
        }

        int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 8, NULL, 0);

        if (fn == BACKPLANE_FUNCTION) {
                logic_debug_set(pin_BACKPLANE_WRITE, 0);
        }

        CYW43_VDEBUG("cyw43_write_reg_u%d %s 0x%lx=0x%lx\n",size * 8,
                     func_name(fn), (unsigned long)reg, (unsigned long)val);
        return ret;
}

int cyw43_write_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
        return _cyw43_write_reg(self, fn, reg, val, 4);
}

int cyw43_write_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint16_t val) {
        return _cyw43_write_reg(self, fn, reg, val, 2);
}

int cyw43_write_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
        return _cyw43_write_reg(self, fn, reg, val, 1);
}

#if CYW43_BUS_MAX_BLOCK_SIZE > 0x7f8
#error Block size is wrong for SPI
#endif

int cyw43_read_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, uint8_t *buf) {

    CYW43_VDEBUG("Calling cyw43_read_bytes()"); 
        /* this flag is initialized to true, but should remain false starting with the first call 
           to this function. */

        assert(fn != BACKPLANE_FUNCTION || (len <= CYW43_BUS_MAX_BLOCK_SIZE));
        const uint32_t padding = (fn == BACKPLANE_FUNCTION) ? CYW43_BACKPLANE_READ_PAD_LEN_BYTES : 0; // Add response delay
        size_t aligned_len = (len + 3) & ~3;
        assert(aligned_len > 0 && aligned_len <= 0x7f8);
        assert(buf == self->spid_buf || buf < self->spid_buf || buf >= (self->spid_buf + sizeof(self->spid_buf)));
        self->spi_header[padding > 0 ? 0 : (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(false, true, fn, addr, len);
        if (fn == WLAN_FUNCTION) {
                logic_debug_set(pin_WIFI_RX, 1);
        }
        int ret = cyw43_spi_transfer(self, NULL, 4, (uint8_t *)&self->spi_header[padding > 0 ? 0 : (CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4 + padding);
        if (fn == WLAN_FUNCTION) {
                logic_debug_set(pin_WIFI_RX, 0);
        }
        if (ret != 0) {
                printf("cyw43_read_bytes error %d", ret);
                return ret;
        }
        if (buf != self->spid_buf) { // avoid a copy in the usual case just to add the header
            memcpy(buf, self->spid_buf, len);
        }
    return 0;
}

/* See whd_bus_spi_transfer_bytes
   Note, uses spid_buf if src isn't using it already
   Apart from firmware download this appears to only be used for wlan functions?
*/
int cyw43_write_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, const uint8_t *src) {
        assert(fn != BACKPLANE_FUNCTION || (len <= CYW43_BUS_MAX_BLOCK_SIZE));
        const size_t aligned_len = (len + 3) & ~3u;
        assert(aligned_len > 0 && aligned_len <= 0x7f8);
        if (fn == WLAN_FUNCTION) {
                /* Wait for FIFO to be ready to accept data */
                int f2_ready_attempts = 1000;
                while (f2_ready_attempts-- > 0) {
                        uint32_t bus_status = cyw43_read_reg_u32(self, BUS_FUNCTION, SPI_STATUS_REGISTER);
                        if (bus_status & STATUS_F2_RX_READY) {
                                logic_debug_set(pin_F2_RX_READY_WAIT, 0);
                                break;
                        } else {
                                logic_debug_set(pin_F2_RX_READY_WAIT, 1);
                        }
                }
                if (f2_ready_attempts <= 0) {
                        printf("F2 not ready\n");
                        return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
                }
        }
        if (src == self->spid_buf) { // avoid a copy in the usual case just to add the header
                self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(true, true, fn, addr, len);
                logic_debug_set(pin_WIFI_TX, 1);
                int res = cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4, NULL, 0);	
                logic_debug_set(pin_WIFI_TX, 0);
                return res;
        } else {
                /* todo: would be nice to get rid of this. Only used for firmware download? */
                assert(src < self->spid_buf || src >= (self->spid_buf + sizeof(self->spid_buf)));
                self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)] = make_cmd(true, true, fn, addr, len);	
                memcpy(self->spid_buf, src, len);
                return cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[(CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4)], aligned_len + 4, NULL, 0);
        }
}
#endif
