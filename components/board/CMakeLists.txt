idf_component_register(
    SRCS "board.c" "board_indicator.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "../../shared"
    PRIV_REQUIRES esp_driver_gpio esp_hw_support soc
                  espressif__led_strip
)
