idf_component_register(
    SRCS "resample.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES heap
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    OUTSIDE_SPEEX
    FIXED_POINT
    RANDOM_PREFIX=omi_speex
)
if(IDF_TARGET STREQUAL "esp32s31" AND CONFIG_SPIRAM)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE RESAMPLE_FULL_SINC_TABLE)
endif()
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-sign-compare)
