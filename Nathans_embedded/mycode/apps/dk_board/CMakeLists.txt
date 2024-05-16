# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(prac3)

include_directories(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../mylib/protocolBuffer/nanopb)
# include_directories(${NANOPB_DIR})  # Include Nanopb headers

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE ../../mylib/hci/hci.c)  # Include hci.c as a source file
target_sources(app PRIVATE ../../mylib/bluetooth/bluetooth.c)  # Include bluetooth.c as a source file
# target_sources(app PRIVATE ../../mylib/protocolBuffer/hci.pb.c)  # Include hci.pb.c as a source file

# target_sources(app PRIVATE 
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../mylib/protocolBuffer/nanopb/pb_common.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../mylib/protocolBuffer/nanopb/pb_encode.c
# )