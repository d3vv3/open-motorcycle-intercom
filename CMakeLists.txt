# Open Motorcycle Intercom (OMI)
# Root CMakeLists.txt for ESP-IDF

cmake_minimum_required(VERSION 3.16)

# Include component directories
set(EXTRA_COMPONENT_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(omi)
