# Mesh component - ESP-NOW transport for when nRF52840 is not available
# Always build - runtime detection determines if it's used

idf_component_register(
    SRCS "mesh.c" "../../shared/mesh_core.c"
    INCLUDE_DIRS "include" "../../shared"
    REQUIRES 
        esp_wifi
        esp_timer
        esp_event
        power
)
