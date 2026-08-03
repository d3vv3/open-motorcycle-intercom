idf_component_register(
    SRCS "uart_bridge.c" "../../shared/bridge_frame.c"
    INCLUDE_DIRS "include" "../../shared"
    REQUIRES driver audio
)
