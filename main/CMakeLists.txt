# Main application
# Both mesh (ESP-NOW) and uart_bridge (nRF52840) are included
# Runtime detection determines which transport to use

idf_component_register(
    SRCS "main.c" "media_toggle_guard.c" "e2e_diag.c" "mesh_intent.c" "mesh_channel_pref.c" "transport_espnow.c" "transport_nrf.c"
         "../shared/audio_bundle.c" "../shared/audio_tx_cache.c"
         "../shared/audio_rx_tracker.c"
    INCLUDE_DIRS "." "../shared"
    REQUIRES 
        audio
        cpu_profile
        phone_audio
        mesh
        uart_bridge
        power
        button
        esp_timer
        heap
        nvs_flash
)
