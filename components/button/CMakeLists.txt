idf_component_register(
    SRCS "button.c" "button_gesture.c"
    INCLUDE_DIRS "include"
    REQUIRES board esp_driver_gpio esp_timer
)
