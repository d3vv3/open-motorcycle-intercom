# Main application
# Both mesh (ESP-NOW) and uart_bridge (nRF52840) are included
# Runtime detection determines which transport to use

idf_component_register(
    SRCS "main.c" "media_toggle_guard.c" "e2e_diag.c" "mesh_intent.c" "transport_espnow.c" "transport_nrf.c"
         "rtt_probe.c" "../shared/audio_bundle.c" "../shared/audio_tx_cache.c"
         "../shared/audio_rx_tracker.c"
    INCLUDE_DIRS "." "../shared"
    REQUIRES 
        audio
        cpu_profile
        phone_audio
        mesh
        uart_bridge
        hwtest
        power
        button
        esp_timer
        heap
        nvs_flash
)

if(S31_LC3_WIRE AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE APP_S31_LC3_WIRE=1)
endif()
