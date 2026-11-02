# Mesh component - ESP-NOW transport for when nRF52840 is not available
# Always build - runtime detection determines if it's used
# Mesh audio frames are always LC3.

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
