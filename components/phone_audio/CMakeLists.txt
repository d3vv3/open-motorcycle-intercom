idf_component_register(
    SRCS "phone_audio.c" "phone_audio_call_state.c"
    INCLUDE_DIRS "include"
    REQUIRES audio bt
)
