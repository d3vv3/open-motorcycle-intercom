idf_component_register(
    SRCS "uart_bridge.c" "bridge_tx.c" "bridge_rx.c" "bridge_status.c" "bridge_commands.c"
         "../../shared/bridge_frame.c"
    INCLUDE_DIRS "include" "../../shared"
    REQUIRES board esp_driver_gpio esp_driver_spi esp_timer
)
