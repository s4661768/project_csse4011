#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

/* ======= I2C MACROS ======= */
/* For this address to work the AD0 pin on the MPU6050 board must be pulled down */
#define MPU_DEV_ADDR 0b1101000

#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define SMPLRT_DIV 0x19

#define LP_FILTER 0x05
#define FULL_SCALE_GYRO 0xF8

#define PWR_MGMT_1 0x6B
#define SLEEP_MODE_MASK 0b01000000

#define FIFO_EN 0x23

#define FIFO_EN_READ_REG_X 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44

#define FIFO_EN_READ_REG_Y 0x20
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46

#define FIFO_EN_READ_REG_Z 0x10
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

/* Offsets */
#define GYRO_X_OFFSET -849
#define GYRO_Y_OFFSET 875
#define GYRO_Z_OFFSET -1008

LOG_MODULE_REGISTER(MPU6050_TEST);

const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(arduino_i2c));

void init_mpu6050(void) {
    printk("\r\n\r\n");
    if (i2c_dev == NULL || !device_is_ready(i2c_dev)) {
        printk("Couldn't get I2C device\r\n");
        return;
    } else {
        printk("Got I2C device\r\n");
    }

    /* Turn on LP filter */
    int ret = i2c_reg_write_byte(i2c_dev, MPU_DEV_ADDR, CONFIG, LP_FILTER);

    /* Conf Gyro to use full scale */
    ret = i2c_reg_write_byte(i2c_dev, MPU_DEV_ADDR, GYRO_CONFIG, FULL_SCALE_GYRO);

    // Take the device out of sleep mode
    ret = i2c_reg_update_byte(i2c_dev, MPU_DEV_ADDR, PWR_MGMT_1, SLEEP_MODE_MASK, 0);
    if (ret != 0) {
        printk("Couldn't take device out of sleep mode\r\n");
    }

    /*  Conf Sample Rate to 1 KHz i.e. SMPLRT_DIV = 7 */
    ret = i2c_reg_write_byte(i2c_dev, MPU_DEV_ADDR, SMPLRT_DIV, 0x07);
    if (ret != 0) {
        printk("Couldn't set gyro sample rate\r\n");
    } else {
        printk("Set gyro sample rate\r\n");
    }
}

int16_t mpu6050_get_x(void) {
    uint8_t lower = 0;
    uint8_t upper = 0;
    int16_t gyro_x = 0;

    int ret = i2c_reg_write_byte(i2c_dev, MPU_DEV_ADDR, FIFO_EN,
                                 FIFO_EN_READ_REG_X); // Read gyro x registers

    // Read gyro x upper
    ret = i2c_reg_read_byte(i2c_dev, MPU_DEV_ADDR, GYRO_XOUT_L, &lower);

    // Read gyro x lower
    ret = i2c_reg_read_byte(i2c_dev, MPU_DEV_ADDR, GYRO_XOUT_H, &upper);

    // bit shift
    gyro_x = (upper << 8) | lower;

    return gyro_x + GYRO_X_OFFSET;
}

int16_t mpu6050_get_y(void) {
    uint8_t lower = 0;
    uint8_t upper = 0;
    int16_t gyro_y = 0;

    int ret = i2c_reg_write_byte(i2c_dev, MPU_DEV_ADDR, FIFO_EN,
                                 FIFO_EN_READ_REG_Y); // Read gyro x registers

    // Read gyro x upper
    ret = i2c_reg_read_byte(i2c_dev, MPU_DEV_ADDR, GYRO_YOUT_L, &lower);

    // Read gyro x lower
    ret = i2c_reg_read_byte(i2c_dev, MPU_DEV_ADDR, GYRO_YOUT_H, &upper);

    // bit shift
    gyro_y = (upper << 8) | lower;

    return gyro_y + GYRO_Y_OFFSET;
}

int16_t mpu6050_get_z(void) {
    uint8_t lower = 0;
    uint8_t upper = 0;
    int16_t gyro_z = 0;

    int ret = i2c_reg_write_byte(i2c_dev, MPU_DEV_ADDR, FIFO_EN,
                                 FIFO_EN_READ_REG_Z); // Read gyro x registers

    // Read gyro x upper
    ret = i2c_reg_read_byte(i2c_dev, MPU_DEV_ADDR, GYRO_ZOUT_L, &lower);

    // Read gyro x lower
    ret = i2c_reg_read_byte(i2c_dev, MPU_DEV_ADDR, GYRO_ZOUT_H, &upper);

    // bit shift
    gyro_z = (upper << 8) | lower;

    return gyro_z GYRO_Z_OFFSET;
}

int main(void) {

    init_mpu6050();

    while (true) {

        // print gyro x
        printk("Gyro xyz: %d | %d | %d\r\n", mpu6050_get_x(), mpu6050_get_y(), mpu6050_get_z());
        k_sleep(K_MSEC(40));
    }

    return 0;
}