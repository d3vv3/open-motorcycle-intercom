idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        audio
        hwtest
        esp_timer
        nvs_flash
)
