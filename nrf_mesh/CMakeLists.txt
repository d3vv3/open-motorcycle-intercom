# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(omi_mesh)

# Resolve image-scoped sysbuild properties as well as standalone -D arguments.
zephyr_get(OMI_FIRMWARE_VERSION SYSBUILD LOCAL)
zephyr_get(OMI_GIT_SHA SYSBUILD LOCAL)
if(NOT DEFINED OMI_FIRMWARE_VERSION)
    set(OMI_FIRMWARE_VERSION "0.0.0-dev")
endif()
if(NOT DEFINED OMI_GIT_SHA)
    execute_process(
        COMMAND git rev-parse HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE OMI_GIT_SHA
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE OMI_GIT_RESULT
    )
    if(NOT OMI_GIT_RESULT EQUAL 0 OR OMI_GIT_SHA STREQUAL "")
        set(OMI_GIT_SHA "unknown")
    endif()
endif()

target_compile_definitions(app PRIVATE
    OMI_FIRMWARE_VERSION="${OMI_FIRMWARE_VERSION}"
    OMI_GIT_SHA="${OMI_GIT_SHA}"
)

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
    src/mesh_protocol_adaptive.c
    src/mesh_protocol_tx.c
    src/ws_sync.c
    ../shared/audio_bundle.c
    ../shared/bridge_frame.c
    ../shared/mesh_core.c
    ../shared/mesh_membership.c
    ../shared/mesh_adaptive.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared
)
