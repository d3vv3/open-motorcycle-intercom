# Mesh component - ESP-NOW transport for when nRF52840 is not available
# Always build - runtime detection determines if it's used

option(S31_LC3_WIRE "Use LC3 frames for ESP32-S31 legacy mesh audio" OFF)

idf_component_register(
    SRCS
        "mesh.c"
        "mesh_transport.c"
        "mesh_tdma.c"
        "mesh_audio.c"
        "mesh_session.c"
        "mesh_adaptive_esp.c"
        "../../shared/mesh_core.c"
        "../../shared/mesh_adaptive.c"
    INCLUDE_DIRS "include" "../../shared"
    REQUIRES 
        esp_wifi
        esp_timer
        esp_event
        power
)

if(S31_LC3_WIRE AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE MESH_S31_LC3_WIRE=1)
endif()
