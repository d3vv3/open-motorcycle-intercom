idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        audio
        mesh
        hwtest
        power
        esp_timer
        nvs_flash
)
