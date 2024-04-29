# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(FP)

target_sources(app PRIVATE src/main.c)

set(MYLIB_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../mylib)
set(MYLIB_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

zephyr_include_directories(${MYLIB_SRC_DIR})
zephyr_include_directories(${MYLIB_INC_DIR})

FILE(GLOB mylib_sources ${MYLIB_SRC_DIR}/*.c )

zephyr_library_sources(${mylib_sources})
