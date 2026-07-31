# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(omi_mesh)

target_sources(app PRIVATE
    src/main.c
    src/esb_radio.c
    src/tdma.c
    src/uart_bridge.c
    src/mesh_protocol.c
    src/ws_sync.c
    ../shared/mesh_core.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared
)
