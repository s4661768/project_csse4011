#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

#include <zephyr/sys/printk.h>
// #include <zephyr/drivers/sensor/mpu9250.h>

#include "/home/s4588783/CSSE4011_Project/project_csse4011/nrf_thingy/zephyr/drivers/sensor/tdk/mpu9250/mpu9250.h"

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "../bluetooth/ibeacon.h"

#define STACKSIZE 512
#define PRIORITY 2

// static void test_function() {
//     const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

//     if (!i2c_dev) {
//         printk("device initalisation failed\n");
//     }

//     uint8_t result = 0;
//     // i2c_write(i2c_dev, 0x3e, )
//     // int err = i2c_reg_write_byte(i2c_dev, 0x3e, 0x06, &result);
//     // int err = i2c_reg_write_byte(i2c_dev, 0x3e, 0x06, 0xff);
//     int err = 0;
//     // printk("got a err: %d status: %d\n", err, result);
//     err = i2c_reg_read_byte(i2c_dev, 0x68, 0x71, &result);
//     printk("got a err: %d status: %d\n", err, result);
// }

static int process_gyroscope(const struct device *dev) {
    struct sensor_value gyro[3];
    int rc = sensor_sample_fetch(dev);

    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    }
    if (rc == 0) {
        printk("Gyro: %f %f %f rad/s\n",
               sensor_value_to_double(&gyro[0]),
               sensor_value_to_double(&gyro[1]),
               sensor_value_to_double(&gyro[2]));
    } else {
        printk("Sample fetch/get failed: %d\n", rc);
    }
    return rc;
}

static int mpu9250_thread(void) {
    // const struct device *const mpu9250 = DEVICE_DT_GET_ONE(invensense_mpu9250);  // Adjusted for MPU-9250
    // const struct device *mpu9250 = DEVICE_DT_GET_ONE(invensense_mpu9250);
    const struct device *mpu9250 = DEVICE_DT_GET_ONE(invensense_mpu6050);


    if (!device_is_ready(mpu9250)) {
        printk("Device %s is not ready\n", mpu9250->name);
        return 0;
    }

    while (1) {
        // test_function();
        // int rc = process_gyroscope(mpu9250);

        printk("Testing\n");

        // if (rc != 0) {
        //     break;
        // }
        k_msleep(10);
    }

    return 0;
}

K_THREAD_DEFINE(mpu9250Thread, STACKSIZE, mpu9250_thread, NULL, NULL, NULL,
                PRIORITY, 0, 0);
