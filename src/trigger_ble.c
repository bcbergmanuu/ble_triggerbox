#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>                                                                                                                                                     

#include <zephyr/sys/ring_buffer.h>
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


LOG_MODULE_REGISTER(TRIGGERBOX_BLE, LOG_LEVEL_INF);


const struct device *const gpio_dev[2] = {DEVICE_DT_GET(DT_NODELABEL(gpio0)),DEVICE_DT_GET(DT_NODELABEL(gpio1))};

static struct bt_uuid_128 uuid_triggerbox_prim = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xa6c47c93, 0x1112, 0x4ca6, 0x176e, 0xdffb871f11f0));

static struct bt_uuid_128 uuid_data = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x84b45e35, 0x140a, 0x4d32, 0x92c6, 0x386d8bff160d));


static bool isConnected = 0;
static uint8_t triggers_ble_buff[50]; //size of packed protobuf


static ssize_t read_ad_buffer(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	void *buf, uint16_t len, uint16_t offset)
{	

	const uint8_t *value = attr->user_data;	

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(triggers_ble_buff));
}

void ble_notify_adbuffer_proc(struct k_work *ptrWorker);
void port_triggered_proc(struct k_work *worker);

void enable_gpio_interrupt(struct k_work *work);
void disable_gpio_interrupt(struct k_work *work);

K_WORK_DEFINE(work_ble_notify, ble_notify_adbuffer_proc);
K_WORK_DEFINE(work_porttrigger, port_triggered_proc);

K_WORK_DEFINE(work_enable_notify, enable_gpio_interrupt);
K_WORK_DEFINE(work_disable_notify, disable_gpio_interrupt); //todo:implement?


K_THREAD_STACK_DEFINE(threatstack_notifyen, 1024);
K_THREAD_STACK_DEFINE(threatstack_port_triggered, 1024);
K_THREAD_STACK_DEFINE(threatstack_ble_notify, 1024);


RING_BUF_DECLARE(ringbuf, 75);

struct k_work_q work_q_notify_enabler;
struct k_work_q work_q_port_trigger;
struct k_work_q work_q_ble_notify;

void rdy_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{    		
	//SPARSE("triggerd by: %d", pins);
	k_work_submit_to_queue(&work_q_port_trigger, &work_porttrigger);		
}

static const uint8_t gpiopins0[6] = {2,3,4,5,28,29};
static const uint8_t gpiopins1[2] = {11,12};

static const uint8_t *gpiopinarrs[2] = {gpiopins0, gpiopins1};
static const uint8_t gpiosizes[2] = {6,2};

void configure_gpio()
{	
	static struct gpio_callback rdy_cb_data_port0[ARRAY_SIZE(gpiopins0)];
	static struct gpio_callback rdy_cb_data_port1[ARRAY_SIZE(gpiopins1)];

	static struct gpio_callback *rdy_cb_data_ports[2] = {rdy_cb_data_port0, rdy_cb_data_port1};

	for(int portnum = 0; portnum < 2; portnum++) {	
		for(int pin = 0; pin < gpiosizes[portnum]; pin++) {
			gpio_pin_configure(gpio_dev[portnum], gpiopinarrs[portnum][pin], GPIO_INPUT | GPIO_PULL_UP);    
			gpio_init_callback(&rdy_cb_data_ports[portnum][pin], rdy_handler, BIT(gpiopinarrs[portnum][pin]));    	
			gpio_add_callback(gpio_dev[portnum], &rdy_cb_data_ports[portnum][pin]);	
		}
	}
	
}

void enable_gpio_interrupt(struct k_work *work) {
	for(int portnum = 0; portnum < 2; portnum++) {	
		for(int pin = 0; pin < gpiosizes[portnum]; pin++) {	
			gpio_pin_interrupt_configure(gpio_dev[portnum], gpiopinarrs[portnum][pin], GPIO_INT_EDGE_BOTH);
		}
	}
}

void disable_gpio_interrupt(struct k_work *work)
{    
	for(int portnum = 0; portnum < 2; portnum++) {	
		for(int pin = 0; pin < gpiosizes[portnum]; pin++) {	
			gpio_pin_configure(gpio_dev[portnum], gpiopinarrs[portnum][pin], GPIO_INPUT);  
			gpio_pin_interrupt_configure(gpio_dev[portnum], gpiopinarrs[portnum][pin], GPIO_INT_DISABLE);
		}
	}
}


static uint8_t notify_ad_buffer_on = 0;

static void ble_ad_buffer_notify_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	ARG_UNUSED(attr);
	LOG_INF("notify_changed: %d", value);
	notify_ad_buffer_on = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
	if(value == BT_GATT_CCC_NOTIFY || value == BT_GATT_CCC_INDICATE)	{
		LOG_INF("start listening to triggers");
		k_work_submit_to_queue(&work_q_notify_enabler, &work_enable_notify);						
	} else {
		LOG_INF("stop listening to triggers");
		k_work_submit_to_queue(&work_q_notify_enabler, &work_disable_notify);						
	}
}


void ble_notify_adbuffer_proc(struct k_work *ptrWorker);

BT_GATT_SERVICE_DEFINE(triggerbox_svc,
	BT_GATT_PRIMARY_SERVICE(&uuid_triggerbox_prim),
	
	BT_GATT_CHARACTERISTIC(&uuid_data.uuid, 
	 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
	 		       BT_GATT_PERM_READ,
	 		       read_ad_buffer, NULL, triggers_ble_buff),
	BT_GATT_CCC(ble_ad_buffer_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
K_MUTEX_DEFINE(MUT_porttrigger);
K_MUTEX_DEFINE(MUT_ble_notify);

void port_triggered_proc(struct k_work *worker) {
	
	if (k_mutex_lock(&MUT_porttrigger, K_NO_WAIT) == 0) {

		uint32_t port_val0 = nrf_gpio_port_in_read(NRF_P0);
		uint32_t port_val1 = nrf_gpio_port_in_read(NRF_P1);
		
		uint8_t portdata =\
			(port_val0 >> 2 & 0b11) |\
			(port_val0 >> 28 & 0b11) << 2 |\
			(port_val0 >> 4 & 0b11) << 4 | \
			(port_val1 >> 11 & 0b11) << 6;
				
		LOG_INF("value to send: %d, port0: %d, port1: %d", portdata, port_val0, port_val1);
		ring_buf_put(&ringbuf, &portdata, 1);
		k_sleep(K_USEC(30));
		k_mutex_unlock(&MUT_porttrigger);		
	}
	k_work_submit_to_queue(&work_q_ble_notify, &work_ble_notify);
}

/// @brief Notify BLE when data ready
/// @param ptrWorker 
void ble_notify_adbuffer_proc(struct k_work *ptrWorker) {					
	
	if(!isConnected) return;
	if(!notify_ad_buffer_on) return;
	if (k_mutex_lock(&MUT_ble_notify, K_NO_WAIT) == 0) {		
		uint8_t enqueued_ringbuffer = ring_buf_get(&ringbuf, triggers_ble_buff, sizeof(triggers_ble_buff));

		struct bt_gatt_attr *notify_attr = bt_gatt_find_by_uuid(triggerbox_svc.attrs, triggerbox_svc.attr_count, &uuid_data.uuid);	
		struct bt_gatt_notify_params params = {
			.attr = notify_attr,
			.data = triggers_ble_buff,
			.len  = enqueued_ringbuffer,
		    //.func = notify_complete_cb, 
		};

		int ret = bt_gatt_notify_cb(NULL, &params);
		if(ret < 0) {
			LOG_ERR("error notify data %d", ret);
		}
		
		LOG_INF("notifications send, amount: %d", enqueued_ringbuffer);
		k_sleep(K_MSEC(50));
		
		k_mutex_unlock(&MUT_ble_notify);
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
		isConnected = true;
		LOG_INF("Connected\n");		
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)\n", reason);	
	isConnected = false;
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

int initialize(void)
{		
	k_work_queue_init(&work_q_ble_notify);
	k_work_queue_init(&work_q_notify_enabler);
	k_work_queue_init(&work_q_port_trigger);

	k_work_queue_start(&work_q_notify_enabler, threatstack_notifyen,
					   K_THREAD_STACK_SIZEOF(threatstack_notifyen), 95,
					   NULL);	

	k_work_queue_start(&work_q_ble_notify, threatstack_ble_notify,
						K_THREAD_STACK_SIZEOF(threatstack_ble_notify), 96,
						NULL);	

	k_work_queue_start(&work_q_port_trigger, threatstack_port_triggered,
						K_THREAD_STACK_SIZEOF(threatstack_port_triggered), 97,
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
	configure_gpio();
	
	return 0;
}

SYS_INIT(initialize, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
