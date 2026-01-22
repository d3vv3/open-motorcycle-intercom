idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        hwtest
        esp_timer
        nvs_flash
)
