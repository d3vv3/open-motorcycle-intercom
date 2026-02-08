idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        audio
        mesh
        hwtest
        power
        button
        esp_timer
        nvs_flash
)
