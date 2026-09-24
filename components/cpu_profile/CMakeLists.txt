option(S31_TASK_CPU_PROFILE "Enable ESP32-S31 task CPU profiling" OFF)

if(S31_TASK_CPU_PROFILE)
    if(NOT IDF_TARGET STREQUAL "esp32s31")
        message(FATAL_ERROR "S31_TASK_CPU_PROFILE requires IDF_TARGET=esp32s31")
    endif()
    foreach(required GENERATE_RUN_TIME_STATS RUN_TIME_COUNTER_TYPE_U64
                     RUN_TIME_STATS_USING_ESP_TIMER VTASKLIST_INCLUDE_COREID)
        if(NOT CONFIG_FREERTOS_${required})
            message(FATAL_ERROR "S31_TASK_CPU_PROFILE requires CONFIG_FREERTOS_${required}=y; enable it in the build's sdkconfig (not the root sdkconfig)")
        endif()
    endforeach()
endif()

idf_component_register(SRCS "cpu_profile.c" INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "../../shared"
    PRIV_REQUIRES freertos heap esp_timer esp_hw_support log)

if(S31_TASK_CPU_PROFILE)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC CPU_PROFILE_ENABLED=1)
endif()
