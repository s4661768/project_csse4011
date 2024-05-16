#include "bluetooth.h"
#include "../hci/hci.h"

#include <zephyr/sys/slist.h>
#include <stdlib.h>
#include <string.h>

#include <ctype.h> // Include ctype.h for tolower or toupper

LOG_MODULE_DECLARE(AHU);

// Constants
#define APPLE_COMPANY_ID 0x004C
#define IBEACON_TYPE 0x02
#define IBEACON_DATA_LEN 0x15
#define TARGET_UUID {0x18, 0xee, 0x15, 0x16, 0x01, 0x6b, 0x4b, 0xec, 0xad, 0x96, 0xbc, 0xb9, 0x6d, 0x16, 0x6e, 0x99}

#define MOVE_STOP 0
#define MOVE_FORWARD 1
#define MOVE_BACKWARD 2 
#define MOVE_LEFT 3
#define MOVE_RIGHT 4

// Static Global Varialbes
static bool should_print_devices = false;
static bt_addr_le_t target_addr;

// Utility function to duplicate a string
char* my_strdup(const char* str) {
    if (str == NULL) {
        return NULL;
    }
    
    // Allocate memory for the new string
    char* copy = (char*)k_malloc(strlen(str) + 1);  // Plus null terminator
    if (copy == NULL) {
        return NULL;  // Memory allocation failed
    }
    
    // Copy the string into the newly allocated memory and return the copy
    strcpy(copy, str);
    return copy;
}

/* function process_ibeacon_data()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
 * Processes iBeacon data received from BLE advertisements
 *
 * data: Pointer to the advertisement data
 * len: Length of the advertisement data
 *
 * Returns: None
 *
 * Errors: None */
static void process_ibeacon_data(const uint8_t *data, uint8_t len) {
    if (len < IBEACON_DATA_LEN) {
        return;
    }

    // Major at offset 20, Minor at offset 22
    uint16_t major = sys_get_be16(&data[20]);
    uint16_t minor = sys_get_be16(&data[22]);

    // Extract the movement type
    char movement_id = major >> 8;

    printk("movement_id: %c\n", movement_id);

    add_to_hci_queue(movement_id);
}

/* function is_ibeacon()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−
 * Checks if the BLE advertisement data corresponds to an iBeacon
 *
 * data: The advertisement data to check
 *
 * Returns: true if the data is from an iBeacon, false otherwise
 *
 * Errors: None */
static bool is_ibeacon(struct bt_data *data) {
    return data->type == BT_DATA_MANUFACTURER_DATA &&
           data->data_len >= IBEACON_DATA_LEN &&
           sys_get_le16(data->data) == APPLE_COMPANY_ID &&
           data->data[2] == IBEACON_TYPE;
}

/* function data_cb()
 * −−−−−−−−−−−−−−−−−−−−−−−−−
 * Callback function for processing BLE advertisement data
 *
 * data: The advertisement data
 * user_data: User-defined data, unused in this function
 *
 * Returns: true to continue parsing data, false to stop
 *
 * Errors: None */
static bool data_cb(struct bt_data *data, void *user_data) {
    if (is_ibeacon(data))
    {
        process_ibeacon_data(data->data, data->data_len);
        return false;
    }
    return true;
}

/* function device_found()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−−−−
 * Callback function for handling found BLE devices during scanning
 *
 * addr: Address of the found device
 * rssi: RSSI of the BLE advertisement
 * type: Advertisement type
 * ad: Advertisement data
 *
 * Returns: None
 *
 * Errors: None */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad) {
    if (!bt_addr_le_cmp(addr, &target_addr))
    {
        // The address matches the target address
        bt_data_parse(ad, data_cb, NULL);
    }
}

/* function observer_start()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−−
 * Starts the BLE observer to scan for iBeacons
 *
 * Returns: 0 on success, error code otherwise
 *
 * Errors: Prints error message if scanning fails to start */
int observer_start(void) {
    should_print_devices = false;

    struct bt_le_scan_param scan_param = {
        .type       = BT_LE_SCAN_TYPE_PASSIVE,
        .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval   = BT_GAP_SCAN_FAST_INTERVAL,
        .window     = BT_GAP_SCAN_FAST_WINDOW,
    };
    int err;

    err = bt_le_scan_start(&scan_param, device_found);
    if (err) {
        LOG_ERR("Start scanning failed (err %d)\n", err);
        return err;
    }
    LOG_INF("Started scanning for iBeacons...\n");

    return 0;
}

/* function observer_stop()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−−
 * Stops the BLE observer
 *
 * Returns: 0 on success, error code otherwise
 *
 * Errors: Prints error message if scanning fails to stop */
int observer_stop(void) {
    int err = bt_le_scan_stop();
    if (err) {
        LOG_ERR("Stop scanning failed (err %d)\n", err);
        return err;
    }
    LOG_ERR("Stopped scanning for iBeacons.\n");
    return 0;
}

/* function bt_ready()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−
 * Callback function called when Bluetooth is ready
 *
 * Returns: None
 *
 * Errors: Prints error message if Bluetooth initialization fails */
void bt_ready() {
    int err;
    err = bt_enable(NULL);

    LOG_INF("Starting Bluetooth Observer\n");

    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)\n", err);
        return;
    }

    LOG_INF("Bluetooth initialized\n");
}

/* function cmd_handle_bluetooth()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
 * Command line interface handler for Bluetooth operations
 *
 * sh: The shell instance
 * argc: Number of arguments
 * argv: Argument values
 *
 * Returns: 0 on success, error code otherwise
 *
 * Errors: Prints error message if command is invalid or fails */
int cmd_handle_bluetooth(const struct shell *sh, size_t argc, char **argv) {
    if ((argc > 3) || (argc == 1)) {
        LOG_ERR("bluetooth_module: invalid number of arguments");
        return -1;
    }

    if (strcmp(argv[1], "-s") == 0) {
        int err = bt_addr_le_from_str(argv[2], "random", &target_addr);
        if (err) {
            LOG_ERR("Invalid MAC address format");
            return -1;
        }
        observer_start();
    } else if (strcmp(argv[1], "-p") == 0) {
        observer_stop();
    } else {
        LOG_ERR("bluetooth_module: invalid command line argument");
        return -1;
    }

    return 0;
}

SHELL_CMD_ARG_REGISTER(blecon, NULL, "Receive Sensor Data from Thingy:52", cmd_handle_bluetooth,
                       2, 1);

/* function ble_scan_start_with_filter()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
 * Starts BLE scanning with an optional address filter
 *
 * filter_addr: BLE address to filter devices, NULL for no filtering
 *
 * Returns: 0 on success, error code otherwise
 *
 * Errors: Logs error on starting scan or invalid BLE address format */
int ble_scan_start_with_filter(const char *filter_addr) {
    should_print_devices = true;

    struct bt_le_scan_param scan_param = {
        .type       = BT_LE_SCAN_TYPE_ACTIVE,
        .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval   = BT_GAP_SCAN_FAST_INTERVAL,
        .window     = BT_GAP_SCAN_FAST_WINDOW,
    };
    int err;

    err = bt_le_scan_start(&scan_param, device_found);
    if (err) {
        LOG_ERR("Start scanning failed (err %d)", err);
        return err;
    }

    if (filter_addr != NULL) {
        int err = bt_addr_le_from_str(filter_addr, "random", &target_addr);
        if (err) {
            LOG_ERR("Invalid BLE address format");
            return err;
        }
    }

    LOG_INF("Started scanning for BLE devices...");
    return 0;
}

/* function ble_scan_stop()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−
 * Stops the ongoing BLE scan
 *
 * Returns: 0 on success, error code otherwise
 *
 * Errors: Logs error on failing to stop scan */
int ble_scan_stop(void) {
    int err = bt_le_scan_stop();
    if (err) {
        LOG_ERR("Stop scanning failed (err %d)", err);
        return err;
    }
    LOG_INF("Stopped scanning for BLE devices.");
    return 0;
}

/* function cmd_handle_blescan()
 * −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
 * Command line handler for starting and stopping BLE scanning with optional filtering
 *
 * sh: Shell instance
 * argc: Number of arguments
 * argv: Arguments array
 *
 * Returns: 0 on success, -1 on error
 *
 * Errors: Logs error on invalid arguments or command execution */
int cmd_handle_blescan(const struct shell *sh, size_t argc, char **argv) {
    if (argc < 2 || argc > 4) {
        LOG_ERR("Usage: blescan -<s/p> [-f <BLE ADDR>]");
        return -1;
    }

    if (strcmp(argv[1], "-s") == 0) {
        if (argc == 4 && strcmp(argv[2], "-f") == 0) {
            return ble_scan_start_with_filter(argv[3]);
        } else if (argc == 2) {
            return ble_scan_start_with_filter(NULL);
        } else {
            LOG_ERR("Invalid arguments for 'blescan -s'");
            return -1;
        }
    } else if (strcmp(argv[1], "-p") == 0) {
        return ble_scan_stop();
    } else {
        LOG_ERR("Invalid argument: Use 's' to start scanning or 'p' to stop");
        return -1;
    }
}

SHELL_CMD_ARG_REGISTER(blescan, NULL, "Scan for BLE devices (Usage: blescan -<s/p> [-f <BLE ADDR>])",
                       cmd_handle_blescan, 2, 2);