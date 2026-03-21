idf_component_register(
    SRCS "audio.c" "vox.c" "voice_cleanup.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        esp_timer
        driver
        freertos
        log
        esp_adc
        78__esp-opus
        power
    PRIV_REQUIRES
        soc
        hal
)
