
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>

#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "../../../mylib/hci/hci.h"
#include "../../../mylib/gcu/commands.h"
#include "../../../mylib/bluetooth/bluetooth.h"

#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(AHU);

// Main Function
int main(void) {

  init_hci();

  bt_ready();


  while (1) {
    k_msleep(100);
  }

  return 1;
}
