
#include "ibeacon.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#define MSG_SIZE 32

#define STACKSIZE 1028
#define PRIORITY 2

#ifndef IBEACON_RSSI
#define IBEACON_RSSI 0xc8
#endif

LOG_MODULE_DECLARE(AHU);

/*
 * Set iBeacon demo advertisement data. These values are for
 * demonstration only and must be changed for production environments!
 *
 * UUID:  18ee1516-016b-4bec-ad96-bcb96d166e99
 * Major: 0
 * Minor: 0
 * RSSI:  -56 dBm
 */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
		      0x4c, 0x00, /* Apple */
		      0x02, 0x15, /* iBeacon */
		      0x18, 0xee, 0x15, 0x16, /* UUID[15..12] */
		      0x01, 0x6b, /* UUID[11..10] */
		      0x4b, 0xec, /* UUID[9..8] */
		      0xad, 0x96, /* UUID[7..6] */
		      0xbc, 0xb9, 0x6d, 0x16, 0x6e, 0x99, /* UUID[5..0] */
		      0x00, 0x00, /* Major */
		      0x00, 0x00, /* Minor */
		      IBEACON_RSSI) /* Calibrated RSSI @ 1m */
};

K_FIFO_DEFINE(sensor_fifo);

typedef struct {
    void *fifo_reserved;
    char message[4];
} ibeacon_queue_item;

/* function add_to_ibeacon_queue()
 * ---------------------------------
 * Adds a sensor reading to the iBeacon queue for advertisement
 *
 * movementType: The movement type integer (e.g., '0', '1', '2', '3', '4')
 *
 * Returns: None */
void add_to_ibeacon_queue(int movementType) {

	ibeacon_queue_item *queueItem = k_calloc(1, sizeof(ibeacon_queue_item));

	queueItem->message[0] = '0' + movementType;  // Convert int to corresponding char digit
	queueItem->message[1] = 0;
	queueItem->message[2] = 0;
	queueItem->message[3] = 0;

  	k_fifo_put(&sensor_fifo, queueItem);
}

/* function bt_init()
 * ---------------------------------
 * Bluetooth initialization callback
 *
 * err: Error code from Bluetooth initialization
 *
 * Returns: None
 *
 * Errors: Prints error messages for initialization failures */
static void bt_init(int err) {

  if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	struct bt_le_oob oob;
	err = bt_le_oob_get_local(BT_ID_DEFAULT, &oob);
	if (err) {
		printk("Failed to get local address (err %d)\n", err);
	} else {
		printk("Advertising with address: %02X:%02X:%02X:%02X:%02X:%02X\n",
		       oob.addr.a.val[5], oob.addr.a.val[4], oob.addr.a.val[3],
		       oob.addr.a.val[2], oob.addr.a.val[1], oob.addr.a.val[0]);
	}

}

/* function bt_send()
 * ---------------------------------
 * Sends the updated iBeacon advertisement data
 *
 * Returns: None
 *
 * Errors: Logs an error if advertisement update fails */
static void bt_send() {

	int err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
	printk("broadcasting\n");
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	
	k_msleep(100);
}

/* function thread_read_queue()
 * ---------------------------------
 * Thread function to read sensor data from the queue and update iBeacon advertisement
 *
 * Returns: None
 *
 * Errors: Logs initialization and advertisement errors */
static void thread_read_queue() {

  	int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_init);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	}

  	for (;;) {
    	ibeacon_queue_item *queueItem = k_fifo_get(&sensor_fifo, K_MSEC(100));

    	if (!queueItem) {
    		k_msleep(100);
      	continue;
      	}

    	memcpy(&ad[1].data[20], queueItem->message, sizeof(queueItem->message));

		printk("message: %c\n", queueItem->message[0]);

    	bt_send();
      
    	k_free(queueItem);
    	k_msleep(100);
	}
}

K_THREAD_DEFINE(ibeaconThread, STACKSIZE, thread_read_queue, NULL, NULL, NULL,
                PRIORITY, 0, 0);