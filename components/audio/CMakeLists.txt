idf_component_register(
    SRCS "audio.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        esp_timer
        driver
        freertos
        log
        esp_adc
        78__esp-opus
    PRIV_REQUIRES
        soc
        hal
)
