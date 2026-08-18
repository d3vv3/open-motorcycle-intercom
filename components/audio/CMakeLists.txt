idf_component_register(
    SRCS "audio.c" "audio_hw.c" "audio_capture.c" "audio_playout.c" "audio_rx.c"
         "audio_notify.c" "vox.c" "voice_cleanup.c" "audio_packet_store.c"
         "audio_pcm_resampler.c" "audio_rx_source_select.c"
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
