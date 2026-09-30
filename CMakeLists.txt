# Open Motorcycle Intercom (OMI)
# Root CMakeLists.txt for ESP-IDF

cmake_minimum_required(VERSION 3.16)

set(OMI_FIRMWARE_VERSION "0.0.0-dev" CACHE STRING "OMI firmware version")
# Keep ESP-IDF application metadata consistent with the boot version.
set(PROJECT_VER "${OMI_FIRMWARE_VERSION}")
if(NOT DEFINED OMI_GIT_SHA)
    execute_process(
        COMMAND git rev-parse HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE OMI_GIT_SHA
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE OMI_GIT_RESULT
    )
    if(NOT OMI_GIT_RESULT EQUAL 0 OR OMI_GIT_SHA STREQUAL "")
        set(OMI_GIT_SHA "unknown")
    endif()
endif()

# Include component directories
set(EXTRA_COMPONENT_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(omi)

idf_component_get_property(OMI_MAIN_TARGET main COMPONENT_LIB)
target_compile_definitions(${OMI_MAIN_TARGET} PRIVATE
    OMI_FIRMWARE_VERSION="${OMI_FIRMWARE_VERSION}"
    OMI_GIT_SHA="${OMI_GIT_SHA}"
)

# IDF v6.1 S31 Bluedroid triggers this warning under Werror. Keep the
# workaround private to the IDF bt component; application warnings stay errors.
idf_component_get_property(OMI_BT_TARGET bt COMPONENT_LIB)
if(OMI_BT_TARGET)
    target_compile_options(${OMI_BT_TARGET} PRIVATE -Wno-error=stringop-truncation)
endif()
