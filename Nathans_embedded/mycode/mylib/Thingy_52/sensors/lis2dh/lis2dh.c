
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
// #include <zephyr/drivers/sensor/lis2dh.h>
#include <stdio.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "../bluetooth/ibeacon.h"

#define STACKSIZE 512
#define PRIORITY 2

#define MOVE_STOP 0
#define MOVE_FORWARD 1
#define MOVE_BACKWARD 2 
#define MOVE_LEFT 3
#define MOVE_RIGHT 4

typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

#define MOVE_THRESHOLD 5 // Threshold for movement
#define TIME_WINDOW 1000   // Time window in milliseconds
#define COOLDOWN_PERIOD 500  // Cooldown period in milliseconds after a gesture

typedef struct {
    float x;
    float y;
    float z;
    uint32_t timestamp;
} movement_data_t;

static movement_data_t reference_data = {0, 0, 0, 0};
static bool is_reference_set = false;
static uint32_t last_gesture_time = 0;  // Timestamp of the last detected gesture

/* function lis2dh_ready()
 * ---------------------------------
 * Checks if the lis2dh sensor device is ready
 * 
 * Returns: Device structure pointer if the sensor is ready, otherwise NULL
 * 
 * Errors: Prints a message if the device is not found or not ready */
static const struct device* lis2dh_ready() {
    const struct device *const dev = DEVICE_DT_GET_ANY(st_lis2dh);

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 0;
    }

    return dev;
}

/* function lis2dh_read_accelerometer()
 * ---------------------------------
 * Reads accelerometer data from the lis2dh sensor
 * 
 * dev: The lis2dh device from which to read
 *
 * Returns: Acceleromter data
 *
 * Errors: Prints errors to console if data fetch or read operations fail */
static accel_data_t lis2dh_read_accelerometer(const struct device *dev)
{
	struct sensor_value accel[3];
    accel_data_t data = {0, 0, 0}; // Initialize to zero

	int rc = sensor_sample_fetch(dev);
	
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        if (rc == 0) {
            // Successfully retrieved data, convert to double and store in result
            data.x = sensor_value_to_double(&accel[0]);
            data.y = sensor_value_to_double(&accel[1]);
            data.z = sensor_value_to_double(&accel[2]);
            // printk("#%u @ %u ms: x %f, y %f, z %f\n",
            //        count, k_uptime_get_32(),
            //        data.x, data.y, data.z);
        } else {
            printk("ERROR: Unable to read accelerometer data: %d\n", rc);
        }
    } else {
        printk("ERROR: Sample fetch failed: %d\n", rc);
    }

    return data;
}

const int check_gesture(movement_data_t current_data) {
    uint32_t current_time = k_uptime_get_32();

    // Check if we are still within the cooldown period
    if (current_time - last_gesture_time < COOLDOWN_PERIOD) {
        return 0;  // Ignore any movements during the cooldown period
    }

    // Set the initial reference data if not already set
    if (!is_reference_set) {
        reference_data = current_data;
        is_reference_set = true;
        return 0; // Exit the function to wait for the next data point
    }
    
    int movement = 0;

    // Check gestures based on x-axis movement
    if (current_data.x - reference_data.x >= MOVE_THRESHOLD) {
        reference_data = current_data;
        movement = 1;
    } else if (reference_data.x - current_data.x >= MOVE_THRESHOLD) {
        reference_data = current_data;
        movement = 2;
    }

    // Check gestures based on y-axis movement
    if (current_data.y - reference_data.y >= MOVE_THRESHOLD) {
        reference_data = current_data;
        movement = 4;
    } else if (reference_data.y - current_data.y >= MOVE_THRESHOLD) {
        reference_data = current_data;
        movement = 3;
    }

    // Check gestures based on z-axis movement
    if (current_data.z - reference_data.z >= MOVE_THRESHOLD) {
        reference_data = current_data;
        movement = 6;
    } else if (reference_data.z - current_data.z >= MOVE_THRESHOLD) {
        reference_data = current_data;
        movement = 5;
    }

    if (movement != 0) {
        last_gesture_time = current_time;  // Update the last gesture time
        is_reference_set = false;  // Reset the reference to allow for new gesture detection after cooldown
    }

    return movement;
}

static int lis2dh_thread(void *p1, void *p2, void *p3) {
    const struct device* dev = lis2dh_ready();

    if (!dev) {
        return 0;
    }

    while (true) {
        accel_data_t accel_data = lis2dh_read_accelerometer(dev);
        uint32_t current_time = k_uptime_get_32();
        movement_data_t current_movement = {accel_data.x, accel_data.y, accel_data.z, current_time};

        // printk("Received Accelerometer Data: x=%f, y=%f, z=%f at %u ms\n", 
        //        current_movement.x, current_movement.y, current_movement.z, current_movement.timestamp);

        int movement = check_gesture(current_movement);

        if (movement != 0) {
            printk("Movement Received: %d\n", movement);
            if (movement == 5) {
                add_to_ibeacon_queue(0);
            } else if (movement == 6) {
                add_to_ibeacon_queue(5);
            } else {
                add_to_ibeacon_queue(movement);
            }
        }

        k_sleep(K_MSEC(50));
    }

    return 0;
}

K_THREAD_DEFINE(lis2dhThread, STACKSIZE, lis2dh_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);