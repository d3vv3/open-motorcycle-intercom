# Mesh component - ESP-NOW transport for when nRF52840 is not available
# Always build - runtime detection determines if it's used

option(S31_LC3_WIRE "Use LC3 frames for ESP32-S31 legacy mesh audio" OFF)
option(S31_MESH_COEX_PREFER_WIFI "Experiment with Wi-Fi coexistence preference during S31 mesh operation" OFF)
option(S31_MESH_PAIR_UNICAST "Experiment with two-node origin-audio unicast on ESP32-S31" OFF)

if(S31_MESH_COEX_PREFER_WIFI)
    if(NOT IDF_TARGET STREQUAL "esp32s31" OR NOT CONFIG_ESP_COEX_SW_COEXIST_ENABLE)
        message(FATAL_ERROR "S31_MESH_COEX_PREFER_WIFI requires esp32s31 with software coexistence enabled")
    endif()
endif()

idf_component_register(
    SRCS
        "mesh.c"
        "mesh_transport.c"
        "mesh_tdma.c"
        "mesh_audio.c"
        "mesh_session.c"
        "../../shared/mesh_core.c"
    INCLUDE_DIRS "include" "../../shared"
    REQUIRES 
        esp_wifi
        esp_timer
        esp_event
        power
    PRIV_REQUIRES esp_coex
)

if(S31_LC3_WIRE AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE MESH_S31_LC3_WIRE=1)
endif()

if(S31_MESH_COEX_PREFER_WIFI)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE MESH_S31_COEX_PREFER_WIFI=1)
endif()

if(S31_MESH_PAIR_UNICAST AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE MESH_S31_PAIR_UNICAST=1)
endif()
