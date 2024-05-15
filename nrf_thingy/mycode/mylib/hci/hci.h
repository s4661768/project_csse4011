#ifndef HCI_H_
#define HCI_H_


#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_core.h>

void init_hci();
void add_to_hci_queue(char movement_id);

#endif /* HCI_H_ */