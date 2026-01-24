idf_component_register(
    SRCS "mesh.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        esp_wifi
        esp_timer
        esp_event
        power
        audio
)
