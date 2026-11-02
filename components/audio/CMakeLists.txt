option(S31_LC3_SELFTEST "Run the ESP32-S31 LC3 encode/decode startup self-test" OFF)

if(S31_LC3_SELFTEST AND NOT IDF_TARGET STREQUAL "esp32s31")
    message(FATAL_ERROR "S31_LC3_SELFTEST is only valid for IDF_TARGET=esp32s31")
endif()

# Audio is LC3-only (mesh and loopback); the codec lives in esp_lc3_bench.
idf_component_register(
    SRCS "audio.c" "audio_hw.c" "audio_capture.c" "audio_playout.c" "audio_rx.c"
           "audio_notify.c" "audio_prompt.c" "audio_mesh_call.c" "audio_urgent.c" "audio_volume_mix.c" "${CMAKE_CURRENT_BINARY_DIR}/audio_prompts_data.c" "vox.c" "voice_cleanup.c" "audio_packet_store.c"
             "audio_pcm_resampler.c" "audio_rate_converter.c" "audio_capture_fifo.c" "audio_sample_fifo.c" "audio_rx_source_select.c"
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
    PRIV_REQUIRES
           cpu_profile
          esp_psram
          speex_resampler
          esp_lc3_bench
)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(AUDIO_PROMPT_WAVS
    startup peer_join peer_leave mesh_on mesh_off phone_pairing channel_green channel_red
    channel_blue you_are_coordinator you_are_participant incoming_phone_call phone_paired)
list(TRANSFORM AUDIO_PROMPT_WAVS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/../../audios/")
list(TRANSFORM AUDIO_PROMPT_WAVS APPEND ".wav")
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/audio_prompts_data.c"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/encode_notifications.py"
            --audios "${CMAKE_CURRENT_SOURCE_DIR}/../../audios"
            --output "${CMAKE_CURRENT_BINARY_DIR}/audio_prompts_data.c"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/encode_notifications.py" ${AUDIO_PROMPT_WAVS}
    VERBATIM)

if(S31_LC3_SINGLE_OWNER AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_S31_LC3_SINGLE_OWNER=1)
endif()

if(S31_LC3_SELFTEST AND IDF_TARGET STREQUAL "esp32s31")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_S31_LC3_SELFTEST=1)
endif()
