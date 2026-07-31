idf_component_register(
    SRCS "uart_bridge.c"
    INCLUDE_DIRS "include" "../../shared"
    REQUIRES driver audio
)
