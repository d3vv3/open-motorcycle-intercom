option(S31_LC3_SERIALIZE "Serialize experimental ESP32-S31 LC3 vendor calls" OFF)
option(S31_LC3_INTERNAL_BUFFERS "Use shared internal DRAM buffers for S31 LC3 decode" OFF)
option(S31_LC3_SINGLE_OWNER "Run S31 LC3 vendor calls on one pinned task" OFF)

if(S31_LC3_INTERNAL_BUFFERS AND NOT S31_LC3_SERIALIZE)
    message(FATAL_ERROR "S31_LC3_INTERNAL_BUFFERS requires S31_LC3_SERIALIZE=ON")
endif()

if(S31_LC3_SINGLE_OWNER AND NOT S31_LC3_SERIALIZE)
    message(FATAL_ERROR "S31_LC3_SINGLE_OWNER requires S31_LC3_SERIALIZE=ON")
endif()

if(S31_LC3_SINGLE_OWNER AND NOT S31_LC3_WIRE)
    message(FATAL_ERROR "S31_LC3_SINGLE_OWNER requires S31_LC3_WIRE=ON")
endif()

if(ESP_LC3_BENCH AND IDF_TARGET STREQUAL "esp32s31")
    idf_component_register(
        SRCS "esp_lc3_bench.c" "esp_lc3_codec.c"
        INCLUDE_DIRS "include"
        REQUIRES esp_audio_codec esp_timer esp_system log
    )
    if(S31_LC3_SERIALIZE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP_LC3_SERIALIZE=1)
    endif()
    if(S31_LC3_INTERNAL_BUFFERS)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP_LC3_INTERNAL_BUFFERS=1)
    endif()
    if(S31_LC3_SINGLE_OWNER)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP_LC3_SINGLE_OWNER=1)
    endif()
else()
    if("$ENV{OMI_ESP_LC3_BENCH}" STREQUAL "1")
        idf_component_register(INCLUDE_DIRS "include" REQUIRES esp_audio_codec esp_timer esp_system log)
    else()
        idf_component_register(INCLUDE_DIRS "include" REQUIRES esp_timer esp_system log)
    endif()
endif()
