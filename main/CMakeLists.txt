idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        audio
        mesh
        esp_timer
        esp_event
        nvs_flash
        driver
)
