idf_component_register(
    SRCS "power.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer esp_wifi esp_pm driver
)
