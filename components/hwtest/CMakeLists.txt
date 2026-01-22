idf_component_register(
    SRCS "hwtest.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc log esp_timer
)
