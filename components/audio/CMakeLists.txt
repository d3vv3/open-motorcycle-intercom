idf_component_register(
    SRCS "audio.c" "audio_hw.c" "audio_capture.c" "audio_playout.c" "audio_rx.c"
         "audio_notify.c" "vox.c" "voice_cleanup.c" "audio_packet_store.c"
             "audio_pcm_resampler.c" "audio_rate_converter.c" "audio_capture_fifo.c" "audio_sample_fifo.c" "audio_rx_source_select.c" "audio_reblock.c" "audio_aec.c"
         "audio_route.c" "audio_route_runtime.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        board
        esp_timer
        esp_driver_i2c
        esp_driver_i2s
        esp_codec_dev
        freertos
        log
        78__esp-opus
        esp-sr
    PRIV_REQUIRES
        esp_psram
        speex_resampler
)
