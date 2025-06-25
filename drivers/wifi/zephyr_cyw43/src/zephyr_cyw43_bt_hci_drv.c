/**
 * Copyright (c) 2025 Beechwoods Software, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_cyw43_bt_hci

#include <zephyr/device.h>
#include <zephyr/drivers/bluetooth.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "cyw43.h"
#include "cybt_shared_bus_driver.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_driver);

/* Offset of special item */
#define PACKET_TYPE             0
#define PACKET_TYPE_SIZE        1
#define EVT_HEADER_EVENT	1
#define EVT_HEADER_SIZE		2
#define EVT_LE_META_SUBEVENT	3
#define EVT_VENDOR_CODE_LSB	3
#define EVT_VENDOR_CODE_MSB	4

#define MAX_BT_MSG_SIZE 2048

// cyw43_bluetooth_hci_write and cyw43_bluetooth_hci_read require a custom 4-byte packet header in front of the actual HCI packet
// the HCI packet type is stored in the fourth byte of the packet header
#define CYW43_PACKET_HEADER_SIZE 4
static uint8_t __noinit cyw43_rxbuf[MAX_BT_MSG_SIZE + CYW43_PACKET_HEADER_SIZE];
static uint8_t __noinit cyw43_txbuf[CONFIG_NET_BUF_DATA_SIZE + CYW43_PACKET_HEADER_SIZE];

struct zephyr_cyw43_bt_hci_data {
	bt_hci_recv_t recv;
};


static int zephyr_cyw43_bt_hci_init(const struct device *dev)
{
	int rv;
	LOG_DBG("zephyr_cyw43_bt_hci_init() calling cyw43_bluetooth_hci_init()");
	rv = cyw43_bluetooth_hci_init();
	LOG_DBG("cyw43_bluetooth_hci_init() rv = %d", rv);
        return rv;
}

static int zephyr_cyw43_bt_hci_open(const struct device *dev, bt_hci_recv_t recv)
{
	int rv = 0;	
	struct zephyr_cyw43_bt_hci_data *hci_data = dev->data;
	
	hci_data->recv = recv;
	
	return rv;
}

static int zephyr_cyw43_bt_hci_close(const struct device *dev)
{
	int rv = 0;
	struct zephyr_cyw43_bt_hci_data *hci_data = dev->data;
	
	hci_data->recv = NULL;
	
	return rv;
}

void cyw43_bluetooth_hci_process(void) {
	
	struct net_buf *buf=NULL;
	bool discardable = false;
	k_timeout_t timeout = K_FOREVER;
	struct bt_hci_acl_hdr acl_hdr = { .len = 0 };
	struct bt_hci_iso_hdr iso_hdr = { .len = 0 };
	uint32_t cyw43_len;
	uint32_t len;
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct zephyr_cyw43_bt_hci_data *hci_data = dev->data;
	uint8_t packet_type;
	uint8_t *rxmsg;
	
        LOG_DBG("Entering cyw43_bluetooth_hci_process()");
	
	cyw43_bluetooth_hci_read(&cyw43_rxbuf[0], MAX_BT_MSG_SIZE, &cyw43_len);

	rxmsg = &cyw43_rxbuf[CYW43_PACKET_HEADER_SIZE - 1];
	packet_type = rxmsg[PACKET_TYPE];
	len = cyw43_len - (CYW43_PACKET_HEADER_SIZE - 1);
		
	LOG_HEXDUMP_DBG(rxmsg, len, "HCI RX data:");
	LOG_DBG("cyw43_bluetooth_hci_process(), len = %d", len);
	LOG_DBG("cyw43_bluetooth_hci_process(): packet_type = %d", packet_type);
	
	switch (packet_type) {
	case BT_HCI_H4_EVT:
		if (rxmsg[EVT_HEADER_EVENT] == BT_HCI_EVT_LE_META_EVENT &&
		    (rxmsg[EVT_LE_META_SUBEVENT] == BT_HCI_EVT_LE_ADVERTISING_REPORT)) {
			discardable = true;
			timeout = K_NO_WAIT;
		}
		buf = bt_buf_get_evt(rxmsg[EVT_HEADER_EVENT],
				     discardable, timeout);
		len = sizeof(struct bt_hci_evt_hdr) + rxmsg[EVT_HEADER_SIZE];
		LOG_DBG("EVT len = %d", len);
		break;
	case BT_HCI_H4_ACL:
		buf = bt_buf_get_rx(BT_BUF_ACL_IN, timeout);
		memcpy(&acl_hdr, &rxmsg[1], sizeof(acl_hdr));
		len = sizeof(struct bt_hci_acl_hdr) + sys_le16_to_cpu(acl_hdr.len);
		LOG_DBG("ACL len = %d", len);
		if (buf != NULL && len > net_buf_tailroom(buf)) {
			LOG_ERR("ACL too long: %d", len);
			net_buf_unref(buf);
			return;
		}

		break;
	case BT_HCI_H4_ISO:
	case BT_HCI_H4_SCO:
		buf = bt_buf_get_rx(BT_BUF_ISO_IN, timeout);
		memcpy(&iso_hdr, &rxmsg[1], sizeof(iso_hdr));
		len = sizeof(struct bt_hci_iso_hdr) + bt_iso_hdr_len(sys_le16_to_cpu(iso_hdr.len));
		LOG_DBG("ISO len = %d", len);
		break;
	default:
		buf = NULL;
		len = 0;
		break;
	}

	if (len == 0) {
		LOG_WRN("Unknown BT buf type %d", rxmsg[PACKET_TYPE]);
	}
	else {
		net_buf_add_mem(buf, &rxmsg[1], len);
		hci_data->recv(dev, buf);
	}
	LOG_DBG("Leaving cyw43_bluetooth_hci_process()\n");
	
	return;
}

static int zephyr_cyw43_bt_hci_send(const struct device *dev, struct net_buf *buf)
{
	int rv=0;
	
	uint8_t packet_type = BT_HCI_H4_NONE;
	uint32_t cyw43_len = 0;
	
	LOG_DBG("zephyr_cyw43_bt_hci_send() bt_buf_get_type() = %s",
		(bt_buf_get_type(buf) == BT_BUF_CMD ? "BT_BUF_CMD" :
		 (bt_buf_get_type(buf) == BT_BUF_EVT ? "BT_BUF_EVT" :
		  (bt_buf_get_type(buf) == BT_BUF_ACL_OUT ? "BT_BUF_ACL_OUT" :
		   (bt_buf_get_type(buf) == BT_BUF_ISO_OUT ? "BT_BUF_ISO_OUT" :
		    "Unknown")))));
	
	switch (bt_buf_get_type(buf)) {
	case BT_BUF_CMD:
		packet_type = BT_HCI_H4_CMD;
		break;		
	case BT_BUF_EVT:
		packet_type = BT_HCI_H4_EVT;
		break;
	case BT_BUF_ACL_OUT:
		packet_type = BT_HCI_H4_ACL;
		break;
	case BT_BUF_ISO_OUT:
		packet_type = BT_HCI_H4_ISO;
		break;
	default:
		rv = -EINVAL;
		break;
	}
	
	if ((packet_type != BT_HCI_H4_NONE)) {
		net_buf_push_u8(buf, packet_type);

		memcpy(&cyw43_txbuf[CYW43_PACKET_HEADER_SIZE-1], buf->data, buf->len);
		cyw43_len = buf->len + CYW43_PACKET_HEADER_SIZE;
				
		LOG_DBG("Calling cyw43_bluetooth_hci_write()");
		rv = cyw43_bluetooth_hci_write(cyw43_txbuf, cyw43_len);
		LOG_DBG("cyw43_bluetooth_hci_write() rv=%d", rv);
		LOG_HEXDUMP_DBG(buf->data, buf->len, "HCI TX data:");
		LOG_DBG("zephyr_cyw43_bt_hci_send(), len = %d\n", buf->len);		
	}	
	net_buf_unref(buf);	
	return rv;
}

#if defined(CONFIG_BT_HCI_SETUP)
static int zephyr_cyw43_bt_hci_setup(const struct device *dev,
				  const struct bt_hci_setup_params *param)
{
	LOG_DBG("zephyr_cyw43_bt_hci_setup() not implemented");
	return 0;
}

#endif /* defined(CONFIG_BT_HCI_SETUP) */


static DEVICE_API(bt_hci, zephyr_cyw43_bt_hci_api) = {
        .open = zephyr_cyw43_bt_hci_open,
        .close = zephyr_cyw43_bt_hci_close,
	.send = zephyr_cyw43_bt_hci_send,
#if defined(CONFIG_BT_HCI_SETUP)	
	.setup = zephyr_cyw43_bt_hci_setup,
#endif /* defined(CONFIG_BT_HCI_SETUP) */	
};

#define HCI_DEVICE_INIT(inst) \
	static struct zephyr_cyw43_bt_hci_data zephyr_cyw43_bt_hci_data_##inst = { \
	}; \
	DEVICE_DT_INST_DEFINE(inst, zephyr_cyw43_bt_hci_init, NULL, &zephyr_cyw43_bt_hci_data_##inst, NULL, \
                              POST_KERNEL, CONFIG_WIFI_INIT_PRIORITY, &zephyr_cyw43_bt_hci_api)

HCI_DEVICE_INIT(0)
