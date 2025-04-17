/*!
 *****************************************************************************
  @file:  ad7124_console_app.c

  @brief: Implementation for the menu functions that handle the AD7124

  @details:
 -----------------------------------------------------------------------------
Copyright (c) 2019 Analog Devices, Inc.  All rights reserved.
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>                                                                                                                                                     

#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/regulator.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "nrfx.h"
#include "hal/nrf_gpio.h"


LOG_MODULE_REGISTER(AD7124_BLE, LOG_LEVEL_INF);


const struct device *const gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  bufr      pointer to data to read in register reg
 * @param  bufw      pointer to data to write in register reg
 * @param  len       number of consecutive bytes to write
 *
 */

 //primary custom service 
static struct bt_uuid_128 uuid_triggerbox_prim = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xa6c47c93, 0x1112, 0x4ca6, 0x176e, 0xdffb871f11f0));

static struct bt_uuid_128 uuid_data = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x84b45e35, 0x140a, 0x4d32, 0x92c6, 0x386d8bff160d));


#define ble_buff_size (5*sizeof(uint32_t))
static uint8_t triggers_ble_buff[ble_buff_size]; //size of packed protobuf


static ssize_t read_ad_buffer(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	void *buf, uint16_t len, uint16_t offset)
{	

	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(triggers_ble_buff));
}

void ble_notify_adbuffer_proc(struct k_work *ptrWorker);
void configure_gpio_interrupt(struct k_work *work);

K_WORK_DEFINE(ble_notify_task, ble_notify_adbuffer_proc);
K_THREAD_STACK_DEFINE(my_stack_area, 1024);
K_WORK_DEFINE(enableNotifyTask, configure_gpio_interrupt);

static const struct device *gpio_dev;
static struct gpio_callback rdy_cb_data;

void rdy_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Only trigger this when SPI is idle!
    uint32_t port_val = nrf_gpio_port_in_read(NRF_P0);
	//triggers_ble_buff = port_val :: think something smart:

	k_work_submit(&ble_notify_task);
    // Schedule SPI read from ADC here
}

void configure_gpio_interrupt(struct k_work *work)
{
    gpio_pin_configure(gpio0_dev, 2,GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpio_dev, 2, GPIO_INT_EDGE_TO_ACTIVE);

    gpio_init_callback(&rdy_cb_data, rdy_handler, BIT(2));
    gpio_add_callback(gpio_dev, &rdy_cb_data);
	//NVIC_EnableIRQ(GPIOTE_IRQn);
}

void disable_gpio_interrupt()
{    
	//NVIC_DisableIRQ(GPIOTE_IRQn);
	
    gpio_pin_interrupt_configure(gpio0_dev, 2, GPIO_INT_DISABLE);    
}


struct k_work_q my_work_q;



static uint8_t notify_ad_buffer_on = 0;

static void ble_ad_buffer_notify_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	ARG_UNUSED(attr);
	LOG_INF("notify_changed: %d", value);
	notify_ad_buffer_on = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
	if(value == BT_GATT_CCC_NOTIFY)	{
		LOG_INF("start listening to triggers");
		k_work_submit_to_queue(&my_work_q, &enableNotifyTask);		
		
		
	}
}


void ble_notify_adbuffer_proc(struct k_work *ptrWorker);



/* AD7124 readout primary Service Declaration */
BT_GATT_SERVICE_DEFINE(triggerbox_svc,
	BT_GATT_PRIMARY_SERVICE(&uuid_triggerbox_prim),
	
	// BT_GATT_CHARACTERISTIC(&uuid_identifier.uuid,
	// 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	// 		       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
	// 		       read_identifier, write_identifier, uniqueIdentifier_value),	

	BT_GATT_CHARACTERISTIC(&uuid_data.uuid, 
	 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
	 		       BT_GATT_PERM_READ,
	 		       read_ad_buffer, NULL, triggers_ble_buff),
	BT_GATT_CCC(ble_ad_buffer_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/// @brief Notify BLE when data ready
/// @param ptrWorker 
void ble_notify_adbuffer_proc(struct k_work *ptrWorker) {					
	if(!notify_ad_buffer_on) return;
	
	struct bt_gatt_attr *notify_attr = bt_gatt_find_by_uuid(triggerbox_svc.attrs, triggerbox_svc.attr_count, &uuid_data.uuid);	
	struct bt_gatt_notify_params params = {
        .attr = notify_attr,
        .data = triggers_ble_buff,
        .len  = sizeof(triggers_ble_buff),
      //  .func = notify_complete_cb, 
    };

	int ret = bt_gatt_notify_cb(NULL, &params);
	if(ret < 0) {
		LOG_ERR("error notify data %d", ret);
	}
}

//advertising data packet
const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL), BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
};


static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err 0x%02x)\n", err);
	} else {		
		LOG_INF("Connected\n");		
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", reason);	
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,	
};

#define BT_LE_ADV_CONN_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE, \
	BT_GAP_ADV_FAST_INT_MIN_2*5, \
	BT_GAP_ADV_FAST_INT_MAX_2*5, NULL)
	
static void bt_ready(void)
{
	int err;

	LOG_INF("Bluetooth initialized\n");	

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_CUSTOM, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return;
	}

	LOG_INF("Advertising successfully started\n");
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = NULL,// auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = NULL,
	.passkey_confirm = NULL,	
};


static struct bt_gatt_cb gatt_callbacks = {
	
};

int ble_load(void)
{		
	k_work_queue_init(&my_work_q);

	k_work_queue_start(&my_work_q, my_stack_area,
					   K_THREAD_STACK_SIZEOF(my_stack_area), 95,
					   NULL);	

	LOG_INF("loading ble");
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)\n", err);
		return -EADV;
	}

	bt_ready();

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	//clean up bonds
	//bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);	
	
	return 0;
}

SYS_INIT(ble_load, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
