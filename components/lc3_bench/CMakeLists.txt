if(LC3_SOURCE_DIR AND IDF_TARGET STREQUAL "esp32s31")
    get_filename_component(LC3_SOURCE_DIR "${LC3_SOURCE_DIR}" ABSOLUTE)
    set(LC3_PINNED_COMMIT "96a3af0beb5487aca3b98a4b992a539a1f6d80d1")
    set(LC3_REQUIRED_FILES
        "${LC3_SOURCE_DIR}/include/lc3.h"
        "${LC3_SOURCE_DIR}/src/attdet.c"
        "${LC3_SOURCE_DIR}/src/bits.c"
        "${LC3_SOURCE_DIR}/src/bwdet.c"
        "${LC3_SOURCE_DIR}/src/energy.c"
        "${LC3_SOURCE_DIR}/src/lc3.c"
        "${LC3_SOURCE_DIR}/src/ltpf.c"
        "${LC3_SOURCE_DIR}/src/mdct.c"
        "${LC3_SOURCE_DIR}/src/plc.c"
        "${LC3_SOURCE_DIR}/src/sns.c"
        "${LC3_SOURCE_DIR}/src/spec.c"
        "${LC3_SOURCE_DIR}/src/tables.c"
        "${LC3_SOURCE_DIR}/src/tns.c"
    )
    foreach(LC3_REQUIRED_FILE IN LISTS LC3_REQUIRED_FILES)
        if(NOT EXISTS "${LC3_REQUIRED_FILE}")
            message(FATAL_ERROR "Required liblc3 file is missing: ${LC3_REQUIRED_FILE}")
        endif()
    endforeach()

    execute_process(
        COMMAND git rev-parse HEAD
        WORKING_DIRECTORY "${LC3_SOURCE_DIR}"
        RESULT_VARIABLE LC3_GIT_RESULT
        OUTPUT_VARIABLE LC3_CHECKED_OUT_COMMIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT LC3_GIT_RESULT EQUAL 0 OR NOT LC3_CHECKED_OUT_COMMIT STREQUAL LC3_PINNED_COMMIT)
        message(FATAL_ERROR
            "LC3_SOURCE_DIR must be a git checkout at liblc3 commit ${LC3_PINNED_COMMIT} "
            "(found '${LC3_CHECKED_OUT_COMMIT}').")
    endif()

    execute_process(
        COMMAND git status --porcelain --untracked-files=no
        WORKING_DIRECTORY "${LC3_SOURCE_DIR}"
        RESULT_VARIABLE LC3_STATUS_RESULT
        OUTPUT_VARIABLE LC3_TRACKED_STATUS
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT LC3_STATUS_RESULT EQUAL 0 OR NOT LC3_TRACKED_STATUS STREQUAL "")
        message(FATAL_ERROR "LC3_SOURCE_DIR must have a clean tracked tree.")
    endif()

    set(LC3_UPSTREAM_SOURCES
        "${LC3_SOURCE_DIR}/src/attdet.c"
        "${LC3_SOURCE_DIR}/src/bits.c"
        "${LC3_SOURCE_DIR}/src/bwdet.c"
        "${LC3_SOURCE_DIR}/src/energy.c"
        "${LC3_SOURCE_DIR}/src/lc3.c"
        "${LC3_SOURCE_DIR}/src/ltpf.c"
        "${LC3_SOURCE_DIR}/src/mdct.c"
        "${LC3_SOURCE_DIR}/src/plc.c"
        "${LC3_SOURCE_DIR}/src/sns.c"
        "${LC3_SOURCE_DIR}/src/spec.c"
        "${LC3_SOURCE_DIR}/src/tables.c"
        "${LC3_SOURCE_DIR}/src/tns.c"
    )

    idf_component_register(
        SRCS "lc3_bench.c" ${LC3_UPSTREAM_SOURCES}
        INCLUDE_DIRS "include" "${LC3_SOURCE_DIR}/include"
        REQUIRES esp_timer log
        PRIV_REQUIRES heap esp_psram
    )

    target_compile_options(${COMPONENT_LIB} PRIVATE -ffast-math)
else()
    idf_component_register(
        INCLUDE_DIRS "include"
        REQUIRES esp_timer log
        PRIV_REQUIRES heap esp_psram
    )
endif()
