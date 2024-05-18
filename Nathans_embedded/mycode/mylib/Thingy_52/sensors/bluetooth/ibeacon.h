#ifndef IBEACON_H_
#define IBEACON_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

void read_from_queue();
void add_to_ibeacon_queue(int movement_id);

#endif /* IBEACON_H_ */
