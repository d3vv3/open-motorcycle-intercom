# Main application
# Both mesh (ESP-NOW) and uart_bridge (nRF52840) are included
# Runtime detection determines which transport to use

idf_component_register(
    SRCS "main.c" "rtt_probe.c" "../shared/audio_bundle.c"
    INCLUDE_DIRS "." "../shared"
    REQUIRES 
        audio
        mesh
        uart_bridge
        hwtest
        power
        button
        esp_timer
        nvs_flash
)
