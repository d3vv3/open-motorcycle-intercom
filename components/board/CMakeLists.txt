idf_component_register(
    SRCS "board.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_gpio esp_hw_support soc
)
