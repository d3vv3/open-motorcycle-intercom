idf_component_register(
    SRCS "resample.c"
    INCLUDE_DIRS "."
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    OUTSIDE_SPEEX
    FIXED_POINT
    RANDOM_PREFIX=omi_speex
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-sign-compare)
