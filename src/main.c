/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;

static const struct device *imu_dev = DEVICE_DT_GET_ONE(st_lsm6dsv16x);
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct gpio_dt_spec maxm86161_ldo = GPIO_DT_SPEC_GET_OR(DT_ALIAS(maxm86161_ldo_en), gpios, {0});
static const struct gpio_dt_spec bq25120a_cd = GPIO_DT_SPEC_GET_OR(DT_ALIAS(bq25120a_cd), gpios, {0});
static bool imu_ready = false;
static bool uart_ready = false;

/* I2C device addresses (7-bit) */
#define MAXM86161_I2C_ADDR  0x62  /* Pulse oximeter / heart rate */
#define MAX30208_I2C_ADDR   0x50  /* Temperature sensor (GPIO0/1 grounded) */
#define BQ25120A_I2C_ADDR   0x6A  /* Battery charger */

/* MAXM86161 Register Definitions */
#define MAXM86161_REG_IRQ_STATUS1       0x00
#define MAXM86161_REG_IRQ_STATUS2       0x01
#define MAXM86161_REG_IRQ_ENABLE1       0x02
#define MAXM86161_REG_IRQ_ENABLE2       0x03
#define MAXM86161_REG_FIFO_WRITE_PTR    0x04
#define MAXM86161_REG_FIFO_READ_PTR     0x05
#define MAXM86161_REG_OVF_COUNTER       0x06
#define MAXM86161_REG_FIFO_DATA_COUNTER 0x07
#define MAXM86161_REG_FIFO_DATA         0x08
#define MAXM86161_REG_FIFO_CONFIG1      0x09
#define MAXM86161_REG_FIFO_CONFIG2      0x0A
#define MAXM86161_REG_SYSTEM_CONTROL    0x0D
#define MAXM86161_REG_PPG_SYNC_CONTROL  0x10
#define MAXM86161_REG_PPG_CONFIG1       0x11
#define MAXM86161_REG_PPG_CONFIG2       0x12
#define MAXM86161_REG_PPG_CONFIG3       0x13
#define MAXM86161_REG_PD_BIAS           0x15
#define MAXM86161_REG_LED_SEQ1          0x20
#define MAXM86161_REG_LED_SEQ2          0x21
#define MAXM86161_REG_LED_SEQ3          0x22
#define MAXM86161_REG_LED1_PA           0x23
#define MAXM86161_REG_LED2_PA           0x24
#define MAXM86161_REG_LED3_PA           0x25
#define MAXM86161_REG_LED_RANGE1        0x2A
#define MAXM86161_REG_DIE_TEMP_CONFIG   0x40
#define MAXM86161_REG_DIE_TEMP_INT      0x41
#define MAXM86161_REG_DIE_TEMP_FRAC     0x42
#define MAXM86161_REG_PART_ID           0xFF

/* MAXM86161 System Control Values */
#define MAXM86161_SYS_CTRL_SW_RESET     0x01
#define MAXM86161_SYS_CTRL_SHUT_DOWN    0x02
#define MAXM86161_SYS_CTRL_POWER_ON     0x00

/* BQ25120A Register Definitions */
#define BQ25120A_REG_STATUS             0x00
#define BQ25120A_REG_FAULTS             0x01
#define BQ25120A_REG_TS_CONTROL         0x02
#define BQ25120A_REG_FAST_CHG           0x03
#define BQ25120A_REG_TERMINATION_CURR   0x04
#define BQ25120A_REG_BATTERY_CTRL       0x05
#define BQ25120A_REG_SYS_VOUT_CTRL      0x06
#define BQ25120A_REG_LDO_CTRL           0x07
#define BQ25120A_REG_PUSH_BUTT_CTRL     0x08
#define BQ25120A_REG_ILIM_UVLO_CTRL     0x09
#define BQ25120A_REG_BATT_MON           0x0A
#define BQ25120A_REG_VIN_DPM            0x0B

/* Sensor state tracking */
static bool maxm86161_ready = false;
static bool bq25120a_ready = false;
static bool ble_streaming_enabled = false;
static bool system_sleeping = false;

/* Sensor mode flags */
static bool hr_mode_enabled = true;   /* Heart rate (Green LED) - default ON */
static bool spo2_mode_enabled = false; /* SpO2 (Red + IR LEDs) - default OFF for cleaner HR */

/* Last temperature reading (die temp, approximates skin temp when in contact) */
static int16_t last_die_temp_c10 = 0; /* Temperature in 0.1C units (e.g., 365 = 36.5C) */

/* SpO2 calculation buffers - forward declaration for use in BLE commands */
#define SPO2_BUFFER_SIZE 100
static uint32_t red_buffer[SPO2_BUFFER_SIZE];
static uint32_t ir_buffer[SPO2_BUFFER_SIZE];
static uint32_t green_buffer[SPO2_BUFFER_SIZE];
static int spo2_buffer_idx = 0;
static int spo2_buffer_count = 0;

/* Forward declarations */
static int maxm86161_init(void);
static const char *bq25120a_get_charge_status(uint8_t status);

/* Sleep button GPIOs - P0.02 and P0.17 with internal pull-ups */
static const struct gpio_dt_spec sleep_btn1 = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(gpio0), gpios, {0});
static const struct device *gpio0_dev;
#define SLEEP_BTN1_PIN 2   /* P0.02 */
#define SLEEP_BTN2_PIN 17  /* P0.17 */

/* SpO2 calculation data */
static uint32_t spo2_red_ac = 0;
static uint32_t spo2_red_dc = 0;
static uint32_t spo2_ir_ac = 0;
static uint32_t spo2_ir_dc = 0;
static uint8_t spo2_value = 0;

/* BLE Sensor Data Packet Structure (packed binary for efficiency)
 * Total: 32 bytes
 *
 * Header (2 bytes):
 *   [0]    = 0xAA (sync byte 1)
 *   [1]    = 0x55 (sync byte 2)
 *
 * IMU Accelerometer (6 bytes) - int16_t x1000 (milli-g -> 0.001 m/s2 precision):
 *   [2-3]  = accel_x
 *   [4-5]  = accel_y
 *   [6-7]  = accel_z
 *
 * IMU Gyroscope (6 bytes) - int16_t x1000 (milli-rad/s):
 *   [8-9]  = gyro_x
 *   [10-11] = gyro_y
 *   [12-13] = gyro_z
 *
 * PPG (4 bytes):
 *   [14-17] = ppg_raw (uint32_t)
 *
 * Battery (2 bytes):
 *   [18]    = battery_percent (uint8_t)
 *   [19]    = charge_status (uint8_t)
 *
 * Timestamp (4 bytes):
 *   [20-23] = timestamp_ms (uint32_t)
 *
 * SpO2 data (3 bytes):
 *   [24]    = spo2_percent (uint8_t, 0-100)
 *   [25]    = heart_rate_ppg (uint8_t, from PPG peaks)
 *   [26]    = signal_quality (uint8_t, 0-100)
 *
 * Temperature and mode (3 bytes):
 *   [27]    = skin_temp_c (int8_t, signed celsius, offset by +40 for transmission)
 *   [28]    = mode_flags (bit0=HR, bit1=SpO2)
 *   [29]    = reserved
 *
 * Checksum (2 bytes):
 *   [30-31] = simple sum checksum
 */

#define SENSOR_PACKET_SIZE 32
#define SENSOR_PACKET_SYNC1 0xAA
#define SENSOR_PACKET_SYNC2 0x55

struct __attribute__((packed)) sensor_packet {
	uint8_t sync1;
	uint8_t sync2;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint32_t ppg_raw;
	uint8_t battery_percent;
	uint8_t charge_status;
	uint32_t timestamp_ms;
	uint8_t spo2_percent;
	uint8_t heart_rate_ppg;
	uint8_t signal_quality;
	uint8_t skin_temp_c40;  /* Temp in C + 40 offset (e.g., 76 = 36C) */
	uint8_t mode_flags;     /* bit0=HR, bit1=SpO2 */
	uint8_t reserved;
	uint16_t checksum;
};

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

/* I2C device addresses */
struct i2c_device {
	const char *name;
	uint8_t possible_addrs[4];
	uint8_t num_addrs;
	uint8_t found_addr;
	bool found;
};

static struct i2c_device i2c_devices[] = {
	{
		.name = "MAXM86161 (PPG/Heart Rate)",
		.possible_addrs = {MAXM86161_I2C_ADDR},
		.num_addrs = 1,
		.found = false
	},
	{
		.name = "MAX30208 (Temperature)",
		.possible_addrs = {MAX30208_I2C_ADDR},
		.num_addrs = 1,
		.found = false
	},
	{
		.name = "BQ25120A (Battery Charger)",
		.possible_addrs = {BQ25120A_I2C_ADDR},
		.num_addrs = 1,
		.found = false
	}
};

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data[0]);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service sample\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
		return err;
	}

	uart_ready = true;
	LOG_INF("UART initialized successfully");
	return 0;
}

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);

	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX)) {
		LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
	} else {
		LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
	}
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	/* Check for streaming commands */
	if (len >= 5 && memcmp(data, "START", 5) == 0) {
		ble_streaming_enabled = true;
		printk("BLE streaming ENABLED\n");
		LOG_INF("BLE streaming enabled");
		bt_nus_send(NULL, "OK:START\n", 9);
		return;
	}

	if (len >= 4 && memcmp(data, "STOP", 4) == 0) {
		ble_streaming_enabled = false;
		printk("BLE streaming DISABLED\n");
		LOG_INF("BLE streaming disabled");
		bt_nus_send(NULL, "OK:STOP\n", 8);
		return;
	}

	if (len >= 6 && memcmp(data, "STATUS", 6) == 0) {
		char status_msg[128];
		int slen = snprintf(status_msg, sizeof(status_msg),
			"IMU:%s PPG:%s BAT:%s STREAM:%s HR:%s SPO2:%s TEMP:%d.%dC\n",
			imu_ready ? "OK" : "ERR",
			maxm86161_ready ? "OK" : "ERR",
			bq25120a_ready ? "OK" : "ERR",
			ble_streaming_enabled ? "ON" : "OFF",
			hr_mode_enabled ? "ON" : "OFF",
			spo2_mode_enabled ? "ON" : "OFF",
			last_die_temp_c10 / 10, last_die_temp_c10 % 10);
		if (slen > 0 && slen < sizeof(status_msg)) {
			bt_nus_send(NULL, status_msg, slen);
		}
		return;
	}

	/* Heart rate mode commands */
	if (len >= 5 && memcmp(data, "HR_ON", 5) == 0) {
		hr_mode_enabled = true;
		/* Reinit sensor with new mode */
		maxm86161_init();
		printk("Heart rate mode ENABLED\n");
		bt_nus_send(NULL, "OK:HR_ON\n", 9);
		return;
	}

	if (len >= 6 && memcmp(data, "HR_OFF", 6) == 0) {
		hr_mode_enabled = false;
		/* Reinit sensor with new mode */
		maxm86161_init();
		printk("Heart rate mode DISABLED\n");
		bt_nus_send(NULL, "OK:HR_OFF\n", 10);
		return;
	}

	/* SpO2 mode commands */
	if (len >= 7 && memcmp(data, "SPO2_ON", 7) == 0) {
		spo2_mode_enabled = true;
		/* Reinit sensor with new mode */
		maxm86161_init();
		printk("SpO2 mode ENABLED\n");
		bt_nus_send(NULL, "OK:SPO2_ON\n", 11);
		return;
	}

	if (len >= 8 && memcmp(data, "SPO2_OFF", 8) == 0) {
		spo2_mode_enabled = false;
		/* Clear SpO2 buffers */
		spo2_buffer_idx = 0;
		spo2_buffer_count = 0;
		/* Reinit sensor with new mode */
		maxm86161_init();
		printk("SpO2 mode DISABLED\n");
		bt_nus_send(NULL, "OK:SPO2_OFF\n", 12);
		return;
	}

	/* Temperature query */
	if (len >= 4 && memcmp(data, "TEMP", 4) == 0) {
		char temp_msg[32];
		int slen = snprintf(temp_msg, sizeof(temp_msg),
			"TEMP:%d.%dC\n",
			last_die_temp_c10 / 10,
			(last_die_temp_c10 < 0 ? -last_die_temp_c10 : last_die_temp_c10) % 10);
		if (slen > 0 && slen < sizeof(temp_msg)) {
			bt_nus_send(NULL, temp_msg, slen);
		}
		return;
	}

	/* Help command */
	if (len >= 4 && memcmp(data, "HELP", 4) == 0) {
		const char *help = "Commands: START STOP STATUS HR_ON HR_OFF SPO2_ON SPO2_OFF TEMP\n";
		bt_nus_send(NULL, help, strlen(help));
		return;
	}

	/* Echo received data for debugging */
	printk("BLE RX: %.*s\n", len, data);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void send_ble_message(const char *msg)
{
	if (current_conn && bt_nus_send(NULL, msg, strlen(msg))) {
		LOG_WRN("Failed to send message over BLE");
	}
}

static int enable_maxm86161_ldo(void)
{
	int err;

	if (!gpio_is_ready_dt(&maxm86161_ldo)) {
		LOG_WRN("MAXM86161 LDO GPIO not ready");
		printk("MAXM86161 LDO GPIO not ready\n");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&maxm86161_ldo, GPIO_OUTPUT_ACTIVE);
	if (err) {
		LOG_ERR("Failed to configure MAXM86161 LDO GPIO (err %d)", err);
		return err;
	}

	/* Set LDO enable high */
	gpio_pin_set_dt(&maxm86161_ldo, 1);
	LOG_INF("MAXM86161 LDO enabled (P1.03 HIGH)");
	printk("MAXM86161 LDO enabled (P1.03 HIGH)\n");

	/* Wait for power to stabilize */
	k_msleep(50);

	return 0;
}

static void i2c_bus_sweep(void)
{
	int err;
	uint8_t dummy_data;
	int found_count = 0;

	printk("\n--- Full I2C Bus Sweep (0x08-0x77) ---\n");
	printk("I2C bus: %s\n", i2c_dev->name);
	LOG_INF("Starting full I2C bus sweep...");

	/* Try to configure I2C speed */
	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	err = i2c_configure(i2c_dev, i2c_cfg);
	if (err) {
		printk("Warning: Failed to configure I2C speed (err %d)\n", err);
	} else {
		printk("I2C configured: 100kHz standard mode\n");
	}

	printk("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

	for (uint8_t row = 0; row < 8; row++) {
		printk("%02X: ", row << 4);

		for (uint8_t col = 0; col < 16; col++) {
			uint8_t addr = (row << 4) | col;

			/* Skip reserved addresses */
			if (addr < 0x08 || addr > 0x77) {
				printk("   ");
				continue;
			}

			/* Method 1: Try zero-length write (most universal detection) */
			struct i2c_msg msg = {
				.buf = NULL,
				.len = 0,
				.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
			};
			err = i2c_transfer(i2c_dev, &msg, 1, addr);

			if (err == 0) {
				printk("%02X ", addr);
				LOG_INF("Device found at 0x%02X", addr);
				found_count++;
			} else {
				/* Method 2: Try single-byte read as fallback */
				err = i2c_read(i2c_dev, &dummy_data, 1, addr);
				if (err == 0) {
					printk("%02X ", addr);
					LOG_INF("Device found at 0x%02X (read)", addr);
					found_count++;
				} else {
					printk("-- ");
				}
			}
		}
		printk("\n");
	}

	printk("--- Sweep complete: %d device(s) found ---\n\n", found_count);
	LOG_INF("I2C sweep complete: %d device(s) found", found_count);
}

static int i2c_scan_devices(void)
{
	int err;
	uint8_t dummy_data;
	bool any_not_found = false;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		printk("ERROR: I2C device not ready\n");
		return -ENODEV;
	}

	printk("\n========================================\n");
	printk("       I2C Sensor Detection\n");
	printk("========================================\n");
	LOG_INF("Starting I2C device scan...");

	/* Enable MAXM86161 LDO before scanning */
	printk("\nEnabling MAXM86161 LDO (P1.03)...\n");
	err = enable_maxm86161_ldo();
	if (err) {
		printk("Warning: Could not enable MAXM86161 LDO\n");
	}

	/* Small delay after LDO enable */
	k_msleep(100);

	printk("\nScanning for known devices:\n");
	printk("----------------------------------------\n");

	/* Scan for each device */
	for (int i = 0; i < ARRAY_SIZE(i2c_devices); i++) {
		struct i2c_device *dev = &i2c_devices[i];
		dev->found = false;

		/* Try each possible address for this device */
		for (int j = 0; j < dev->num_addrs; j++) {
			uint8_t addr = dev->possible_addrs[j];

			/* Try to read from the device */
			err = i2c_read(i2c_dev, &dummy_data, 1, addr);

			if (err == 0) {
				/* Device found! */
				dev->found = true;
				dev->found_addr = addr;

				printk("[OK]  %s @ 0x%02X\n", dev->name, addr);
				LOG_INF("Found %s at address 0x%02X", dev->name, addr);
				break;
			}
		}

		if (!dev->found) {
			printk("[--]  %s NOT FOUND (expected 0x%02X)\n",
				dev->name, dev->possible_addrs[0]);
			LOG_WRN("%s not found at 0x%02X", dev->name, dev->possible_addrs[0]);
			any_not_found = true;
		}
	}

	printk("----------------------------------------\n");

	/* If any device not found, do a full bus sweep */
	if (any_not_found) {
		printk("\nSome devices not found. Running full bus sweep...\n");
		i2c_bus_sweep();
	} else {
		printk("\nAll expected devices found!\n");
	}

	printk("========================================\n\n");
	LOG_INF("I2C device scan complete");
	return 0;
}

/* I2C helper functions */
static int i2c_reg_write(uint8_t addr, uint8_t reg, uint8_t value)
{
	uint8_t buf[2] = {reg, value};
	return i2c_write(i2c_dev, buf, 2, addr);
}

static int i2c_reg_read(uint8_t addr, uint8_t reg, uint8_t *value)
{
	return i2c_write_read(i2c_dev, addr, &reg, 1, value, 1);
}

static int i2c_reg_read_multi(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	return i2c_write_read(i2c_dev, addr, &reg, 1, buf, len);
}

/* MAXM86161 PPG Sensor Functions */
static int maxm86161_init(void)
{
	int err;
	uint8_t part_id;

	/* Read Part ID to verify communication */
	err = i2c_reg_read(MAXM86161_I2C_ADDR, MAXM86161_REG_PART_ID, &part_id);
	if (err) {
		LOG_ERR("MAXM86161: Failed to read Part ID (err %d)", err);
		return err;
	}

	printk("MAXM86161 Part ID: 0x%02X\n", part_id);
	LOG_INF("MAXM86161 Part ID: 0x%02X", part_id);

	/* Software reset */
	err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_SYSTEM_CONTROL,
			    MAXM86161_SYS_CTRL_SW_RESET);
	if (err) {
		LOG_ERR("MAXM86161: Software reset failed");
		return err;
	}
	k_msleep(10);

	/* Power on (exit shutdown) */
	err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_SYSTEM_CONTROL,
			    MAXM86161_SYS_CTRL_POWER_ON);
	if (err) {
		LOG_ERR("MAXM86161: Failed to power on");
		return err;
	}

	/* Clear FIFO pointers */
	i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_FIFO_WRITE_PTR, 0x00);
	i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_FIFO_READ_PTR, 0x00);
	i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_OVF_COUNTER, 0x00);

	/* Configure PPG based on mode
	 * HR-only mode: Green LED only at 100 sps for clean signal
	 * SpO2 mode: Green + Red + IR at 50 sps
	 */
	if (spo2_mode_enabled) {
		/* SpO2 mode: 50 sps, 3 LEDs
		 * PPG_CONFIG1: ADC_RNG=01, SMP_RATE=001 (50 sps), TINT=011
		 */
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_PPG_CONFIG1, 0x4B);
		if (err) {
			LOG_ERR("MAXM86161: Failed to configure PPG_CONFIG1");
			return err;
		}

		/* LED Sequence: Green + IR + Red */
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED_SEQ1, 0x21);
		if (err) return err;
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED_SEQ2, 0x03);
		if (err) return err;

		/* LED currents: Green 12mA, IR 25mA, Red 25mA */
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED1_PA, 0x1F);
		if (err) return err;
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED2_PA, 0x3F);
		if (err) return err;
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED3_PA, 0x3F);
		if (err) return err;

		printk("MAXM86161: SpO2 mode (Green+IR+Red @ 50sps)\n");
	} else {
		/* HR-only mode: 100 sps, Green LED only for clean signal
		 * PPG_CONFIG1: ADC_RNG=01, SMP_RATE=010 (100 sps), TINT=010 (58.7us)
		 */
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_PPG_CONFIG1, 0x52);
		if (err) {
			LOG_ERR("MAXM86161: Failed to configure PPG_CONFIG1");
			return err;
		}

		/* LED Sequence: Green only (LEDC1=Green, LEDC2=none) */
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED_SEQ1, 0x01);
		if (err) return err;
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED_SEQ2, 0x00);
		if (err) return err;

		/* LED currents: Green 15mA (slightly higher for better signal) */
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED1_PA, 0x28);
		if (err) return err;
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED2_PA, 0x00);
		if (err) return err;
		err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED3_PA, 0x00);
		if (err) return err;

		printk("MAXM86161: HR-only mode (Green @ 100sps)\n");
	}

	/* PPG_CONFIG2 (0x12): Sample averaging
	 *   [2:0] SMP_AVE = 010 (4 sample average for noise reduction)
	 */
	err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_PPG_CONFIG2, 0x02);
	if (err) {
		LOG_ERR("MAXM86161: Failed to configure PPG_CONFIG2");
		return err;
	}

	/* Set LED range for all LEDs */
	err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_LED_RANGE1, 0x00);
	if (err) {
		LOG_ERR("MAXM86161: Failed to set LED range");
		return err;
	}

	/* Enable die temperature measurement */
	err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_DIE_TEMP_CONFIG, 0x01);
	if (err) {
		LOG_WRN("MAXM86161: Failed to enable temperature");
	}

	maxm86161_ready = true;
	printk("MAXM86161 PPG+SpO2 sensor initialized\n");
	LOG_INF("MAXM86161 PPG+SpO2 sensor initialized");
	return 0;
}

/* Shutdown MAXM86161 for sleep mode */
static int maxm86161_shutdown(void)
{
	if (!maxm86161_ready) {
		return 0;
	}

	int err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_SYSTEM_CONTROL,
				MAXM86161_SYS_CTRL_SHUT_DOWN);
	if (err) {
		LOG_ERR("MAXM86161: Failed to shutdown");
		return err;
	}

	maxm86161_ready = false;
	printk("MAXM86161 shutdown\n");
	return 0;
}

static int maxm86161_read_ppg(uint32_t *ppg_value)
{
	int err;
	uint8_t fifo_count;

	if (!maxm86161_ready) {
		return -ENODEV;
	}

	/* Check how many samples are in FIFO */
	err = i2c_reg_read(MAXM86161_I2C_ADDR, MAXM86161_REG_FIFO_DATA_COUNTER, &fifo_count);
	if (err) {
		return err;
	}

	if (spo2_mode_enabled) {
		/* SpO2 mode: read 3 LEDs (9 bytes) */
		uint8_t fifo_data[9];

		if (fifo_count < 3) {
			return -EAGAIN; /* Need at least 3 samples (one per LED) */
		}

		err = i2c_reg_read_multi(MAXM86161_I2C_ADDR, MAXM86161_REG_FIFO_DATA, fifo_data, 9);
		if (err) {
			return err;
		}

		/* Parse the 3 LED values (each is 19-bit, tag in upper 5 bits) */
		uint32_t green_val = ((uint32_t)(fifo_data[0] & 0x07) << 16) |
				     ((uint32_t)fifo_data[1] << 8) |
				     (uint32_t)fifo_data[2];

		uint32_t ir_val = ((uint32_t)(fifo_data[3] & 0x07) << 16) |
				  ((uint32_t)fifo_data[4] << 8) |
				  (uint32_t)fifo_data[5];

		uint32_t red_val = ((uint32_t)(fifo_data[6] & 0x07) << 16) |
				   ((uint32_t)fifo_data[7] << 8) |
				   (uint32_t)fifo_data[8];

		/* Store values in circular buffers for SpO2 calculation */
		green_buffer[spo2_buffer_idx] = green_val;
		ir_buffer[spo2_buffer_idx] = ir_val;
		red_buffer[spo2_buffer_idx] = red_val;

		spo2_buffer_idx = (spo2_buffer_idx + 1) % SPO2_BUFFER_SIZE;
		if (spo2_buffer_count < SPO2_BUFFER_SIZE) {
			spo2_buffer_count++;
		}

		*ppg_value = green_val;
	} else {
		/* HR-only mode: read 1 LED (3 bytes) - much cleaner signal */
		uint8_t fifo_data[3];

		if (fifo_count < 1) {
			return -EAGAIN;
		}

		err = i2c_reg_read_multi(MAXM86161_I2C_ADDR, MAXM86161_REG_FIFO_DATA, fifo_data, 3);
		if (err) {
			return err;
		}

		/* Parse Green LED value (19-bit) */
		uint32_t green_val = ((uint32_t)(fifo_data[0] & 0x07) << 16) |
				     ((uint32_t)fifo_data[1] << 8) |
				     (uint32_t)fifo_data[2];

		*ppg_value = green_val;
	}

	return 0;
}

/* Calculate SpO2 from buffered Red and IR data
 * Uses the ratio-of-ratios method:
 * R = (AC_red/DC_red) / (AC_ir/DC_ir)
 * SpO2 = 110 - 25 * R (simplified linear approximation)
 */
static uint8_t calculate_spo2(void)
{
	if (spo2_buffer_count < 50) {
		return 0; /* Not enough data */
	}

	/* Calculate DC (mean) and AC (peak-to-peak) for Red and IR */
	uint32_t red_sum = 0, ir_sum = 0;
	uint32_t red_min = UINT32_MAX, red_max = 0;
	uint32_t ir_min = UINT32_MAX, ir_max = 0;

	for (int i = 0; i < spo2_buffer_count; i++) {
		red_sum += red_buffer[i];
		ir_sum += ir_buffer[i];

		if (red_buffer[i] < red_min) red_min = red_buffer[i];
		if (red_buffer[i] > red_max) red_max = red_buffer[i];
		if (ir_buffer[i] < ir_min) ir_min = ir_buffer[i];
		if (ir_buffer[i] > ir_max) ir_max = ir_buffer[i];
	}

	uint32_t red_dc = red_sum / spo2_buffer_count;
	uint32_t ir_dc = ir_sum / spo2_buffer_count;
	uint32_t red_ac = red_max - red_min;
	uint32_t ir_ac = ir_max - ir_min;

	/* Store for packet transmission */
	spo2_red_dc = red_dc;
	spo2_red_ac = red_ac;
	spo2_ir_dc = ir_dc;
	spo2_ir_ac = ir_ac;

	/* Avoid division by zero */
	if (red_dc == 0 || ir_dc == 0 || ir_ac == 0) {
		return 0;
	}

	/* Calculate R = (AC_red/DC_red) / (AC_ir/DC_ir)
	 * Rearranged: R = (AC_red * DC_ir) / (DC_red * AC_ir)
	 * Scale by 1000 for fixed-point math
	 */
	uint32_t numerator = (uint32_t)red_ac * ir_dc;
	uint32_t denominator = (uint32_t)red_dc * ir_ac;

	if (denominator == 0) {
		return 0;
	}

	/* R * 1000 */
	uint32_t R_scaled = (numerator * 1000) / denominator;

	/* SpO2 = 110 - 25 * R
	 * With R_scaled = R * 1000:
	 * SpO2 = 110 - (25 * R_scaled) / 1000
	 */
	int32_t spo2_calc = 110 - (25 * R_scaled) / 1000;

	/* Clamp to valid range */
	if (spo2_calc < 70) spo2_calc = 70;
	if (spo2_calc > 100) spo2_calc = 100;

	spo2_value = (uint8_t)spo2_calc;
	return spo2_value;
}

/* Calculate signal quality (0-100) based on AC amplitude */
static uint8_t calculate_signal_quality(void)
{
	if (spo2_buffer_count < 20) {
		return 0;
	}

	/* Quality based on IR AC amplitude - typical good signal is 1000-50000 */
	if (spo2_ir_ac < 100) {
		return 0; /* No signal */
	} else if (spo2_ir_ac < 500) {
		return 20; /* Very weak */
	} else if (spo2_ir_ac < 2000) {
		return 50; /* Weak */
	} else if (spo2_ir_ac < 10000) {
		return 80; /* Good */
	} else {
		return 100; /* Excellent */
	}
}

static int maxm86161_read_temperature(int8_t *temp_int, uint8_t *temp_frac)
{
	int err;

	if (!maxm86161_ready) {
		return -ENODEV;
	}

	/* Trigger temperature measurement */
	err = i2c_reg_write(MAXM86161_I2C_ADDR, MAXM86161_REG_DIE_TEMP_CONFIG, 0x01);
	if (err) {
		return err;
	}
	k_msleep(2); /* Wait for measurement */

	err = i2c_reg_read(MAXM86161_I2C_ADDR, MAXM86161_REG_DIE_TEMP_INT, (uint8_t *)temp_int);
	if (err) {
		return err;
	}

	err = i2c_reg_read(MAXM86161_I2C_ADDR, MAXM86161_REG_DIE_TEMP_FRAC, temp_frac);
	if (err) {
		return err;
	}

	return 0;
}

/* BQ25120A Battery Charger Functions */
static int bq25120a_init(void)
{
	int err;
	uint8_t status, faults;

	printk("\n");
	printk("********************************************\n");
	printk("*       BQ25120A CHARGER INIT START       *\n");
	printk("********************************************\n");

	/* Drive CD (Charge Disable) pin LOW to enable charging */
	if (gpio_is_ready_dt(&bq25120a_cd)) {
		err = gpio_pin_configure_dt(&bq25120a_cd, GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("BQ25120A: Failed to configure CD pin (err %d)", err);
		} else {
			gpio_pin_set_dt(&bq25120a_cd, 0);  /* Drive LOW to enable charging */
			printk("BQ25120A: CD pin driven LOW (charging enabled)\n");
		}
	} else {
		LOG_WRN("BQ25120A: CD GPIO not ready");
		printk("BQ25120A: WARNING - CD GPIO not ready, charging may be disabled!\n");
	}
	k_msleep(5);

	/* CRITICAL: Set BUVLO threshold FIRST before reading any status
	 * to prevent BAT_UVLO fault from latching when battery is below default 3.0V.
	 * Register 0x09: ILIM_UVLO_CTRL (reset state: 0x0A)
	 * B7: RESET (write 1 resets all regs - DO NOT SET!)
	 * B6: Reserved
	 * B5:B3 = INLIM[2:0]: I = 50mA + CODE*50mA (101 = 300mA)
	 * B2:B0 = BUVLO[2:0]: 010=3.0V, 011=2.8V, 100=2.6V, 101=2.4V, 110/111=2.2V
	 * Set to 0x2D: INLIM=300mA (101), BUVLO=2.4V (101)
	 */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_ILIM_UVLO_CTRL, 0x2D);
	if (err) {
		LOG_WRN("BQ25120A: Failed to set BUVLO threshold early");
	} else {
		printk("BQ25120A: BUVLO threshold set to 2.4V (before status read)\n");
	}
	k_msleep(5);

	/* Read status register to verify communication */
	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_STATUS, &status);
	if (err) {
		LOG_ERR("BQ25120A: Failed to read status (err %d)", err);
		return err;
	}

	printk("BQ25120A Status: 0x%02X\n", status);
	printk("  - Charge state: %s\n", bq25120a_get_charge_status(status));
	printk("  - VINDPM active: %s\n", (status & 0x04) ? "YES" : "NO");
	printk("  - CD pin (charge disable): %s\n", (status & 0x02) ? "HIGH (DISABLED!)" : "LOW (enabled)");
	printk("  - SYS output: %s\n", (status & 0x01) ? "ENABLED" : "DISABLED");
	LOG_INF("BQ25120A Status: 0x%02X", status);

	/* Read faults register */
	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_FAULTS, &faults);
	printk("BQ25120A Faults: 0x%02X\n", faults);
	if (faults & 0xF0) {  /* Only upper 4 bits are fault flags */
		if (faults & 0x80) printk("  - VIN_OV (input overvoltage)\n");
		if (faults & 0x40) printk("  - VIN_UV (input undervoltage)\n");
		if (faults & 0x20) printk("  - BAT_UVLO (battery undervoltage)\n");
		if (faults & 0x10) printk("  - BAT_OCP (battery overcurrent)\n");
	}

	/* Disable TS (temperature sensor) function if no thermistor connected
	 * Register 0x02: TS_CONTROL
	 * Bit [7] = TS_EN: 0 = TS function disabled (no thermistor required)
	 * Bits [6:4] = TS_FAULT thresholds
	 * Bits [3:0] = TS current
	 * Set to 0x00 to disable TS completely
	 */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_TS_CONTROL, 0x00);
	if (err) {
		LOG_WRN("BQ25120A: Failed to disable TS");
	} else {
		printk("BQ25120A: TS function disabled (no thermistor)\n");
	}

	/* Configure battery regulation voltage (VBATREG)
	 * Register 0x05: BATTERY_CTRL
	 * Bits [7:1] = VBATREG: 3.6V + (code * 10mV)
	 * For 4.2V: (4200 - 3600) / 10 = 60 = 0x3C -> 0x78 (shifted left 1)
	 * Bit [0] = 0 (reserved)
	 */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_BATTERY_CTRL, 0x78);
	if (err) {
		LOG_WRN("BQ25120A: Failed to set VBATREG");
	} else {
		printk("BQ25120A: VBATREG set to 4.2V\n");
	}

	/* Configure fast charge current
	 * Register 0x03: FAST_CHG
	 * Bit [7] = CE_DIS: 0 = charging enabled
	 * Bit [6] = 0 (reserved)
	 * Bits [5:2] = ICHG: charge current code
	 *   0000 = 5mA, each step adds 5mA up to 35mA (code 6)
	 *   From code 7+: 40mA + (code-7)*10mA
	 *   For 50mA: code = 8 -> 0x20
	 *   For 100mA: code = 13 -> 0x34
	 * Bits [1:0] = IPRETERM range
	 */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_FAST_CHG, 0x34);
	if (err) {
		LOG_WRN("BQ25120A: Failed to set charge current");
	} else {
		printk("BQ25120A: Fast charge current set to ~100mA\n");
	}

	/* Configure termination/pre-charge current
	 * Register 0x04: TERMINATION_CURR
	 * Bits [7:5] = IPRETERM code (pre-charge and termination current)
	 *   Based on range from FAST_CHG[1:0]
	 * Bits [4:2] = reserved
	 * Bit [1] = TE: 1 = termination enabled
	 * Bit [0] = 0 (2X disable)
	 * Set termination enabled with default pre-charge
	 */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_TERMINATION_CURR, 0x02);
	if (err) {
		LOG_WRN("BQ25120A: Failed to set termination");
	}

	/* Configure SYS output voltage - CRITICAL for battery operation
	 * Register 0x06: SYS_VOUT_CTRL
	 * Bit [7] = EN_SYS_OUT: 1 = System output enabled
	 * Bits [6:1] = SYS_VOUT: output voltage code
	 *   1.1V + (code * 100mV)
	 *   For 1.8V: (1800 - 1100) / 100 = 7 = 0x07 -> 0x0E (shifted)
	 * Bit [0] = SYS_SEL: 1 = SYS always regulated (helps in fault state)
	 *
	 * Set EN_SYS_OUT=1, SYS_VOUT for 1.8V, SYS_SEL=1
	 * 0x80 | 0x0E | 0x01 = 0x8F
	 */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_SYS_VOUT_CTRL, 0x8F);
	if (err) {
		LOG_WRN("BQ25120A: Failed to set SYS output");
	} else {
		printk("BQ25120A: SYS output enabled at 1.8V (always regulated)\n");
	}

	/* Read back to verify */
	uint8_t sys_vout_readback;
	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_SYS_VOUT_CTRL, &sys_vout_readback);
	if (err == 0) {
		printk("BQ25120A SYS_VOUT_CTRL readback: 0x%02X (expected 0x8F)\n", sys_vout_readback);
	}

	/* Configure VIN_DPM - input voltage dynamic power management
	 * Register 0x0B: VIN_DPM
	 * Bits [7:5] = VINDPM threshold: 4.2V + code*0.1V
	 *   000 = 4.2V (lowest), 111 = 4.9V
	 * Set to 000 (4.2V) for lowest threshold
	 */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_VIN_DPM, 0x00);
	if (err == 0) {
		printk("BQ25120A: VIN_DPM set to 4.2V threshold\n");
	}

	/* Re-apply ILIM/UVLO settings (already set at init start, but confirm here)
	 * Register 0x09: ILIM_UVLO_CTRL
	 * B7: RESET (DO NOT SET - resets all registers!)
	 * B5:B3 = INLIM: 50mA + CODE*50mA (101 = 300mA)
	 * B2:B0 = BUVLO: 101 = 2.4V
	 * Value 0x2D = 0b00101101
	 */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_ILIM_UVLO_CTRL, 0x2D);
	if (err) {
		LOG_WRN("BQ25120A: Failed to set ILIM/UVLO");
	} else {
		printk("BQ25120A: ILIM=300mA, BUVLO=2.4V\n");
	}

	/* Read back status to confirm */
	k_msleep(10);
	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_STATUS, &status);
	if (err == 0) {
		printk("BQ25120A Status after config: 0x%02X (%s)\n",
		       status, bq25120a_get_charge_status(status));
	}

	/* Verify ILIM/UVLO register was written correctly */
	uint8_t ilim_uvlo;
	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_ILIM_UVLO_CTRL, &ilim_uvlo);
	if (err == 0) {
		printk("BQ25120A ILIM_UVLO_CTRL: 0x%02X (expected 0x2D)\n", ilim_uvlo);
	}

	/* Toggle charge enable to clear fault state
	 * CE_DIS bit[7]: 1 = disable charging, 0 = enable charging
	 */
	printk("BQ25120A: Toggling charge enable to clear faults...\n");
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_FAST_CHG, 0xB4); /* CE_DIS=1 */
	k_msleep(100);
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_FAST_CHG, 0x34); /* CE_DIS=0 */
	k_msleep(100);

	/* Read final status */
	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_STATUS, &status);
	if (err == 0) {
		printk("BQ25120A Final Status: 0x%02X (%s)\n", status, bq25120a_get_charge_status(status));
		printk("  - VIN_PG (USB power): %s\n", (status & 0x20) ? "GOOD" : "NOT PRESENT");
	}
	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_FAULTS, &faults);
	if (err == 0) {
		printk("BQ25120A Final Faults: 0x%02X\n", faults);
	}

	bq25120a_ready = true;
	printk("********************************************\n");
	printk("*       BQ25120A CHARGER INIT DONE        *\n");
	printk("********************************************\n\n");
	LOG_INF("BQ25120A battery charger initialized");
	return 0;
}

static int bq25120a_read_status(uint8_t *status, uint8_t *faults)
{
	int err;

	if (!bq25120a_ready) {
		return -ENODEV;
	}

	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_STATUS, status);
	if (err) {
		return err;
	}

	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_FAULTS, faults);
	if (err) {
		return err;
	}

	return 0;
}

static int bq25120a_read_battery_voltage(uint8_t *vbat_percent)
{
	int err;
	uint8_t batt_mon;

	if (!bq25120a_ready) {
		return -ENODEV;
	}

	/* Initiate battery voltage reading by writing 0x80 to BATT_MON */
	err = i2c_reg_write(BQ25120A_I2C_ADDR, BQ25120A_REG_BATT_MON, 0x80);
	if (err) {
		return err;
	}

	/* Wait for measurement (2ms per datasheet) */
	k_msleep(3);

	/* Read battery monitor register */
	err = i2c_reg_read(BQ25120A_I2C_ADDR, BQ25120A_REG_BATT_MON, &batt_mon);
	if (err) {
		return err;
	}

	/* Extract voltage threshold (bits 6:2) - represents % of VBATREG in 2% increments */
	*vbat_percent = ((batt_mon >> 2) & 0x1F) * 2 + 60; /* 60% to 100% range */

	return 0;
}

static const char *bq25120a_get_charge_status(uint8_t status)
{
	switch ((status >> 6) & 0x03) {
	case 0: return "Ready/Standby";
	case 1: return "Charging";
	case 2: return "Charge Done";
	case 3: return "Fault";
	default: return "Unknown";
	}
}

/* IMU Reading Function */
static int read_imu_data(struct sensor_value *accel, struct sensor_value *gyro)
{
	int err;

	if (!imu_ready) {
		return -ENODEV;
	}

	err = sensor_sample_fetch(imu_dev);
	if (err) {
		return err;
	}

	err = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (err) {
		return err;
	}

	err = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (err) {
		return err;
	}

	return 0;
}

/* Periodic sensor reading function */
static void read_all_sensors(void)
{
	int err;

	printk("\n========== SENSOR READINGS ==========\n");

	/* Read IMU */
	if (imu_ready) {
		struct sensor_value accel[3], gyro[3];
		err = read_imu_data(accel, gyro);
		if (err == 0) {
			/* Use integer format: val1 is whole part, val2 is fractional (millionths) */
			printk("IMU Accel: X=%d.%02d Y=%d.%02d Z=%d.%02d m/s2\n",
			       accel[0].val1, abs(accel[0].val2) / 10000,
			       accel[1].val1, abs(accel[1].val2) / 10000,
			       accel[2].val1, abs(accel[2].val2) / 10000);
			printk("IMU Gyro:  X=%d.%03d Y=%d.%03d Z=%d.%03d rad/s\n",
			       gyro[0].val1, abs(gyro[0].val2) / 1000,
			       gyro[1].val1, abs(gyro[1].val2) / 1000,
			       gyro[2].val1, abs(gyro[2].val2) / 1000);
		} else {
			printk("IMU: Read failed (err %d)\n", err);
		}
	} else {
		printk("IMU: Not ready\n");
	}

	/* Read PPG */
	if (maxm86161_ready) {
		uint32_t ppg_value;
		int8_t temp_int;
		uint8_t temp_frac;

		err = maxm86161_read_ppg(&ppg_value);
		if (err == 0) {
			printk("PPG: Raw=%u (0x%05X)\n", ppg_value, ppg_value);
		} else if (err == -EAGAIN) {
			printk("PPG: No data in FIFO\n");
		} else {
			printk("PPG: Read failed (err %d)\n", err);
		}

		err = maxm86161_read_temperature(&temp_int, &temp_frac);
		if (err == 0) {
			printk("PPG Die Temp: %d.%d C\n", temp_int, (temp_frac * 100) / 256);
		}
	} else {
		printk("PPG: Not ready\n");
	}

	/* Read Battery */
	if (bq25120a_ready) {
		uint8_t status, faults, vbat_pct;

		err = bq25120a_read_status(&status, &faults);
		if (err == 0) {
			printk("Battery Status: 0x%02X (%s) VIN_PG=%s\n", status,
			       bq25120a_get_charge_status(status),
			       (status & 0x20) ? "USB_OK" : "NO_USB");
			if (faults) {
				printk("Battery Faults: 0x%02X", faults);
				if (faults & 0x20) printk(" [BAT_UVLO]");
				if (faults & 0x04) printk(" [TS_FAULT]");
				if (faults & 0x01) printk(" [PG_STAT]");
				printk("\n");
			}
		}

		err = bq25120a_read_battery_voltage(&vbat_pct);
		if (err == 0) {
			printk("Battery Voltage: ~%d%% of VBATREG\n", vbat_pct);
		}
	} else {
		printk("Battery: Not ready\n");
	}

	printk("=====================================\n\n");
}

/* Build and send sensor data packet over BLE */
static int send_sensor_packet(void)
{
	struct sensor_packet pkt = {0};
	uint16_t checksum = 0;
	uint8_t *pkt_bytes = (uint8_t *)&pkt;
	int err;

	/* Sync bytes */
	pkt.sync1 = SENSOR_PACKET_SYNC1;
	pkt.sync2 = SENSOR_PACKET_SYNC2;

	/* Timestamp */
	pkt.timestamp_ms = k_uptime_get_32();

	/* Read IMU data */
	if (imu_ready) {
		struct sensor_value accel[3], gyro[3];
		err = read_imu_data(accel, gyro);
		if (err == 0) {
			/* Convert to milli-units (val1 * 1000 + val2 / 1000) */
			pkt.accel_x = (int16_t)(accel[0].val1 * 1000 + accel[0].val2 / 1000);
			pkt.accel_y = (int16_t)(accel[1].val1 * 1000 + accel[1].val2 / 1000);
			pkt.accel_z = (int16_t)(accel[2].val1 * 1000 + accel[2].val2 / 1000);
			pkt.gyro_x = (int16_t)(gyro[0].val1 * 1000 + gyro[0].val2 / 1000);
			pkt.gyro_y = (int16_t)(gyro[1].val1 * 1000 + gyro[1].val2 / 1000);
			pkt.gyro_z = (int16_t)(gyro[2].val1 * 1000 + gyro[2].val2 / 1000);
		}
	}

	/* Read PPG data and calculate SpO2 */
	if (maxm86161_ready) {
		uint32_t ppg_value = 0;
		err = maxm86161_read_ppg(&ppg_value);
		if (err == 0) {
			pkt.ppg_raw = ppg_value;

			/* Calculate SpO2 (updates internal state) */
			pkt.spo2_percent = calculate_spo2();
			pkt.signal_quality = calculate_signal_quality();
		}
	}

	/* Read battery data (less frequently, cache it) */
	static uint8_t cached_batt_pct = 0;
	static uint8_t cached_charge_status = 0;
	static uint32_t last_batt_read = 0;

	if (bq25120a_ready && (k_uptime_get_32() - last_batt_read > 1000)) {
		uint8_t status, faults, vbat_pct;
		err = bq25120a_read_status(&status, &faults);
		if (err == 0) {
			cached_charge_status = (status >> 6) & 0x03;
		}
		err = bq25120a_read_battery_voltage(&vbat_pct);
		if (err == 0) {
			cached_batt_pct = vbat_pct;
		}
		last_batt_read = k_uptime_get_32();
	}
	pkt.battery_percent = cached_batt_pct;
	pkt.charge_status = cached_charge_status;

	/* Read die temperature periodically (approximates skin temp when sensor contacts skin) */
	static uint32_t last_temp_read = 0;
	if (maxm86161_ready && (k_uptime_get_32() - last_temp_read > 500)) {
		int8_t temp_int;
		uint8_t temp_frac;
		err = maxm86161_read_temperature(&temp_int, &temp_frac);
		if (err == 0) {
			/* Store as 0.1C units: temp_int is integer, temp_frac/256 is decimal */
			last_die_temp_c10 = temp_int * 10 + (temp_frac * 10) / 256;
		}
		last_temp_read = k_uptime_get_32();
	}

	/* Skin temperature: offset by +40 to fit in uint8_t (range -40C to +215C) */
	int16_t temp_celsius = last_die_temp_c10 / 10;
	pkt.skin_temp_c40 = (uint8_t)(temp_celsius + 40);

	/* Mode flags: bit0=HR enabled, bit1=SpO2 enabled */
	pkt.mode_flags = (hr_mode_enabled ? 0x01 : 0x00) | (spo2_mode_enabled ? 0x02 : 0x00);

	/* Calculate checksum (sum of all bytes except checksum field) */
	for (int i = 0; i < sizeof(pkt) - 2; i++) {
		checksum += pkt_bytes[i];
	}
	pkt.checksum = checksum;

	/* Send over BLE NUS */
	if (current_conn) {
		err = bt_nus_send(NULL, pkt_bytes, sizeof(pkt));
		if (err) {
			return err;
		}
	}

	return 0;
}

static int imu_init(void)
{
	int err;
	struct sensor_value odr_attr;

	if (!device_is_ready(imu_dev)) {
		LOG_ERR("IMU device %s not ready", imu_dev->name);
		return -ENODEV;
	}

	LOG_INF("IMU device %s initialized", imu_dev->name);

	/* Configure accelerometer ODR to 104 Hz */
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;
	err = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (err) {
		LOG_WRN("Failed to set accel ODR (err: %d)", err);
	}

	/* Configure gyroscope ODR to 104 Hz */
	err = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (err) {
		LOG_WRN("Failed to set gyro ODR (err: %d)", err);
	}

	/* Test read to verify sensor is working */
	err = sensor_sample_fetch(imu_dev);
	if (err) {
		LOG_ERR("Failed to fetch initial sensor sample (err: %d)", err);
		return err;
	}

	LOG_INF("IMU sensor configured and responding");
	imu_ready = true;
	return 0;
}

/* Initialize sleep buttons (P0.02 and P0.17 with pull-ups) */
static int init_sleep_buttons(void)
{
	int err;

	gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(gpio0_dev)) {
		LOG_ERR("GPIO0 device not ready");
		return -ENODEV;
	}

	/* Configure P0.02 as input with pull-up */
	err = gpio_pin_configure(gpio0_dev, SLEEP_BTN1_PIN,
				 GPIO_INPUT | GPIO_PULL_UP);
	if (err) {
		LOG_ERR("Failed to configure P0.02 (err %d)", err);
		return err;
	}

	/* Configure P0.17 as input with pull-up */
	err = gpio_pin_configure(gpio0_dev, SLEEP_BTN2_PIN,
				 GPIO_INPUT | GPIO_PULL_UP);
	if (err) {
		LOG_ERR("Failed to configure P0.17 (err %d)", err);
		return err;
	}

	printk("Sleep buttons configured (P0.02 + P0.17)\n");
	LOG_INF("Sleep buttons configured on P0.02 and P0.17");
	return 0;
}

/* Check if both sleep buttons are pressed (active low with pull-ups) */
static bool are_sleep_buttons_pressed(void)
{
	if (!gpio0_dev) {
		return false;
	}

	int btn1 = gpio_pin_get(gpio0_dev, SLEEP_BTN1_PIN);
	int btn2 = gpio_pin_get(gpio0_dev, SLEEP_BTN2_PIN);

	/* Both pressed when both are LOW (0) */
	return (btn1 == 0 && btn2 == 0);
}

/* Enter sleep mode - turn off sensors and BLE */
static void enter_sleep_mode(void)
{
	if (system_sleeping) {
		return;
	}

	printk("\n*** ENTERING SLEEP MODE ***\n");
	LOG_INF("Entering sleep mode");

	system_sleeping = true;
	ble_streaming_enabled = false;

	/* Turn off MAXM86161 */
	maxm86161_shutdown();

	/* Turn off MAXM86161 LDO */
	if (gpio_is_ready_dt(&maxm86161_ldo)) {
		gpio_pin_set_dt(&maxm86161_ldo, 0);
		printk("MAXM86161 LDO disabled\n");
	}

	/* Stop BLE advertising */
	bt_le_adv_stop();
	printk("BLE advertising stopped\n");

	/* Turn off LEDs */
	dk_set_led_off(RUN_STATUS_LED);
	dk_set_led_off(CON_STATUS_LED);

	printk("*** SYSTEM SLEEPING - Press both buttons to wake ***\n\n");
}

/* Exit sleep mode - turn on sensors and BLE */
static void exit_sleep_mode(void)
{
	if (!system_sleeping) {
		return;
	}

	printk("\n*** EXITING SLEEP MODE ***\n");
	LOG_INF("Exiting sleep mode");

	/* Turn on MAXM86161 LDO */
	if (gpio_is_ready_dt(&maxm86161_ldo)) {
		gpio_pin_set_dt(&maxm86161_ldo, 1);
		printk("MAXM86161 LDO enabled\n");
		k_msleep(50); /* Wait for power to stabilize */
	}

	/* Re-initialize MAXM86161 */
	int err = maxm86161_init();
	if (err) {
		printk("Failed to re-init MAXM86161 (err %d)\n", err);
	}

	/* Clear SpO2 buffers */
	spo2_buffer_idx = 0;
	spo2_buffer_count = 0;

	/* Restart BLE advertising */
	advertising_start();
	printk("BLE advertising restarted\n");

	system_sleeping = false;
	ble_streaming_enabled = true;

	printk("*** SYSTEM AWAKE ***\n\n");
}

/* Check for sleep button combo - called from main loop */
static void check_sleep_buttons(void)
{
	static bool buttons_were_pressed = false;
	static uint32_t press_start_time = 0;
	const uint32_t HOLD_TIME_MS = 1000; /* Hold for 1 second */

	bool buttons_pressed = are_sleep_buttons_pressed();

	if (buttons_pressed && !buttons_were_pressed) {
		/* Buttons just pressed - start timing */
		press_start_time = k_uptime_get_32();
		buttons_were_pressed = true;
	} else if (buttons_pressed && buttons_were_pressed) {
		/* Buttons held - check if held long enough */
		uint32_t hold_time = k_uptime_get_32() - press_start_time;
		if (hold_time >= HOLD_TIME_MS) {
			if (system_sleeping) {
				exit_sleep_mode();
			} else {
				enter_sleep_mode();
			}
			/* Wait for button release */
			while (are_sleep_buttons_pressed()) {
				k_msleep(50);
			}
			buttons_were_pressed = false;
		}
	} else if (!buttons_pressed && buttons_were_pressed) {
		/* Buttons released */
		buttons_were_pressed = false;
	}
}

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	/* Initialize sleep buttons */
	err = init_sleep_buttons();
	if (err) {
		LOG_WRN("Failed to init sleep buttons (err: %d)", err);
	}
}

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	err = uart_init();
	if (err) {
		LOG_ERR("UART initialization failed (err: %d)", err);
		/* Don't halt - continue with BLE even if UART fails */
	}

	/* Scan for I2C sensors */
	printk("\n*** Starting I2C Sensor Scan ***\n");
	i2c_scan_devices();

	/* Initialize IMU */
	printk("\n*** Initializing Sensors ***\n");
	err = imu_init();
	if (err) {
		printk("IMU init failed (err %d)\n", err);
	}

	/* Initialize MAXM86161 PPG sensor */
	err = maxm86161_init();
	if (err) {
		printk("MAXM86161 init failed (err %d)\n", err);
	}

	/* Initialize BQ25120A battery charger */
	err = bq25120a_init();
	if (err) {
		printk("BQ25120A init failed (err %d)\n", err);
	}

	/* Do initial sensor reading */
	read_all_sensors();

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	for (;;) {
		/* Check sleep button combo */
		check_sleep_buttons();

		if (!system_sleeping) {
			/* Normal operation */
			dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
			k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));

			/* Read sensors every 5 seconds (5 blinks) */
			if ((blink_status % 5) == 0) {
				read_all_sensors();
			}
		} else {
			/* Sleeping - just check buttons periodically */
			k_sleep(K_MSEC(100));
		}
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	printk("BLE streaming thread started\n");
	LOG_INF("BLE streaming thread started");

	/* Auto-enable streaming when connected */
	ble_streaming_enabled = true;

	for (;;) {
		if (system_sleeping) {
			/* System is sleeping, don't try to send */
			k_msleep(100);
			continue;
		}

		if (current_conn && ble_streaming_enabled) {
			/* Send sensor packet at ~25Hz (40ms interval) */
			int err = send_sensor_packet();
			if (err == -ENOMEM) {
				/* BLE buffer full, slow down */
				k_msleep(100);
			} else {
				k_msleep(40);
			}
		} else {
			/* Not connected or streaming disabled, wait */
			k_msleep(100);
		}
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
