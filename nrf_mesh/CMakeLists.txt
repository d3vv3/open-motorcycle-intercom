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
    src/mesh_protocol_rx.c
    src/mesh_protocol_runtime.c
    src/mesh_protocol_membership.c
    src/mesh_protocol_audio.c
    src/mesh_protocol_tx.c
    src/ws_sync.c
    ../shared/audio_bundle.c
    ../shared/bridge_frame.c
    ../shared/mesh_core.c
    ../shared/mesh_membership.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared
)
