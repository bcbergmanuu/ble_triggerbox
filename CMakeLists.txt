# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set_property(GLOBAL PROPERTY CSTD c11)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble-ad7124)

target_sources(app PRIVATE src/main.c src/ad7124_ble.c src/storage_nvs.c)
