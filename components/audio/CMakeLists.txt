idf_component_register(
    SRCS "audio.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        esp_timer
        driver
        freertos
        log
)
