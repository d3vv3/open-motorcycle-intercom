option(S31_LC3_SERIALIZE "Serialize experimental ESP32-S31 LC3 vendor calls" OFF)
option(S31_LC3_INTERNAL_BUFFERS "Use shared internal DRAM buffers for S31 LC3 decode" OFF)
option(S31_LC3_SINGLE_OWNER "Run S31 LC3 vendor calls on one pinned task" OFF)

if(S31_LC3_INTERNAL_BUFFERS AND NOT S31_LC3_SERIALIZE)
    message(FATAL_ERROR "S31_LC3_INTERNAL_BUFFERS requires S31_LC3_SERIALIZE=ON")
endif()

if(S31_LC3_SINGLE_OWNER AND NOT S31_LC3_SERIALIZE)
    message(FATAL_ERROR "S31_LC3_SINGLE_OWNER requires S31_LC3_SERIALIZE=ON")
endif()

# esp_lc3_codec.c is the production audio codec (mesh and loopback are LC3-only)
# and the vendor LC3 library is only pulled for esp32s31.
if(NOT IDF_TARGET STREQUAL "esp32s31")
    message(FATAL_ERROR "The LC3 audio codec requires IDF_TARGET=esp32s31")
endif()

idf_component_register(
    SRCS "esp_lc3_codec.c"
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
