option(ESP_LC3_BENCH "Enable the Espressif LC3 capture benchmark on ESP32-S31" OFF)
option(S31_LC3_WIRE "Use LC3 for ESP32-S31 mesh audio" OFF)
option(S31_LC3_SKIP_RX "Bypass ESP32-S31 mesh LC3 playout decoding for diagnostics" OFF)
option(S31_LC3_SELFTEST "Run the ESP32-S31 LC3 encode/decode startup self-test" OFF)
option(S31_LC3_SPLIT_CORES "Pin ESP32-S31 mesh LC3 playout to core 0" OFF)

if(S31_LC3_WIRE AND NOT ESP_LC3_BENCH)
    message(FATAL_ERROR "S31_LC3_WIRE requires ESP_LC3_BENCH=ON")
endif()

if(S31_LC3_SKIP_RX AND NOT S31_LC3_WIRE)
    message(FATAL_ERROR "S31_LC3_SKIP_RX requires S31_LC3_WIRE=ON")
endif()

if(S31_LC3_SKIP_RX AND NOT IDF_TARGET STREQUAL "esp32s31")
    message(FATAL_ERROR "S31_LC3_SKIP_RX is only valid for IDF_TARGET=esp32s31")
endif()

if(S31_LC3_SELFTEST AND NOT S31_LC3_WIRE)
    message(FATAL_ERROR "S31_LC3_SELFTEST requires S31_LC3_WIRE=ON")
endif()

if(S31_LC3_SELFTEST AND NOT IDF_TARGET STREQUAL "esp32s31")
    message(FATAL_ERROR "S31_LC3_SELFTEST is only valid for IDF_TARGET=esp32s31")
endif()

if(S31_LC3_SELFTEST AND NOT ESP_LC3_BENCH)
    message(FATAL_ERROR "S31_LC3_SELFTEST requires ESP_LC3_BENCH=ON")
endif()

if(S31_LC3_SPLIT_CORES AND NOT S31_LC3_WIRE)
    message(FATAL_ERROR "S31_LC3_SPLIT_CORES requires S31_LC3_WIRE=ON")
endif()

if(S31_LC3_SPLIT_CORES AND NOT IDF_TARGET STREQUAL "esp32s31")
    message(FATAL_ERROR "S31_LC3_SPLIT_CORES is only valid for IDF_TARGET=esp32s31")
endif()

idf_component_register(
    SRCS "audio.c" "audio_hw.c" "audio_capture.c" "audio_opus_stage_profile.c" "audio_playout.c" "audio_rx.c"
         "audio_notify.c" "vox.c" "voice_cleanup.c" "audio_packet_store.c"
             "audio_pcm_resampler.c" "audio_rate_converter.c" "audio_capture_fifo.c" "audio_sample_fifo.c" "audio_rx_source_select.c" "audio_reblock.c" "audio_aec.c"
         "audio_route.c" "audio_route_runtime.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        board
        esp_timer
        esp_driver_i2c
        esp_driver_i2s
        esp_codec_dev
        freertos
        log
        78__esp-opus
        esp-sr
    PRIV_REQUIRES
           cpu_profile
          esp_psram
          speex_resampler
          lc3_bench
          esp_lc3_bench
)

if(CONFIG_IDF_TARGET_ESP32S31)
    idf_component_get_property(OPUS_COMPONENT_DIR 78__esp-opus COMPONENT_DIR)
    target_include_directories(${COMPONENT_LIB} PRIVATE
        "${OPUS_COMPONENT_DIR}/silk"
        "${OPUS_COMPONENT_DIR}/celt"
    )
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_OPUS_STAGE_PROFILE=1)
    target_link_options(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=silk_Encode"
        "-Wl,--wrap=celt_encode_with_ec"
    )
endif()

if(ESP_LC3_BENCH AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_ESP_LC3_BENCH=1)
endif()

if(S31_LC3_WIRE AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_S31_LC3_WIRE=1)
endif()

if(S31_LC3_SINGLE_OWNER AND S31_LC3_WIRE AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_S31_LC3_SINGLE_OWNER=1)
endif()

if(S31_LC3_SELFTEST AND S31_LC3_WIRE AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_S31_LC3_SELFTEST=1)
endif()

if(S31_LC3_SKIP_RX AND S31_LC3_WIRE AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_S31_LC3_SKIP_RX=1)
endif()

if(S31_LC3_SPLIT_CORES AND S31_LC3_WIRE AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_S31_LC3_SPLIT_CORES=1)
endif()

if(LC3_SOURCE_DIR AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_LC3_BENCH=1)
endif()
