# Open Motorcycle Intercom (OMI)
# Root CMakeLists.txt for ESP-IDF

cmake_minimum_required(VERSION 3.16)

# Include component directories
set(EXTRA_COMPONENT_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(omi)

# IDF v6.1 S31 Bluedroid triggers this warning under Werror. Keep the
# workaround private to the IDF bt component; application warnings stay errors.
idf_component_get_property(OMI_BT_TARGET bt COMPONENT_LIB)
if(OMI_BT_TARGET)
    target_compile_options(${OMI_BT_TARGET} PRIVATE -Wno-error=stringop-truncation)
endif()
