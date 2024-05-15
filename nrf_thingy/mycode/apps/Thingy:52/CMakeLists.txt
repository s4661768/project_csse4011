# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Thingy:52)

target_sources(app PRIVATE src/main.c)

target_sources(app PRIVATE ../../mylib/Thingy:52/sensors/bluetooth/ibeacon.c)
target_sources(app PRIVATE ../../mylib/Thingy:52/sensors/lis2dh/lis2dh.c)
# target_sources(app PRIVATE ../../mylib/Thingy:52/sensors/ccs811/ccs811.c) 
# target_sources(app PRIVATE ../../mylib/Thingy:52/sensors/hts221/hts221.c) 
# target_sources(app PRIVATE ../../mylib/Thingy:52/sensors/lps22hb/lps22hb.c)

# zephyr_library()

# zephyr_library_sources(../../mylib/Thingy:52/sensors/mpu9250/mpu9250.c)
# zephyr_library_sources_ifdef(CONFIG_MPU9250_TRIGGER mpu9250_trigger.c)
# zephyr_library_sources_ifdef(CONFIG_MPU9250_MAGN_EN ak8963.c)
