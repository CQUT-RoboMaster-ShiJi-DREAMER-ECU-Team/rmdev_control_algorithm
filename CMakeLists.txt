cmake_minimum_required(VERSION 3.28...4.0.2)

project(rmdev_control_algorithm CXX)

if (EMDEVIF_USE_CPP_MODULES)
    add_library(${PROJECT_NAME} STATIC)

    target_link_libraries(${PROJECT_NAME}
        PUBLIC emdevif_core rmdev_math
        PRIVATE emdevif_compile_options
    )

    target_include_directories(${PROJECT_NAME} PUBLIC inc)
    target_sources(${PROJECT_NAME} PUBLIC
        FILE_SET rmdev_control_algorithm_modules TYPE CXX_MODULES FILES modules/pid.cppm
    )
else ()
    add_library(${PROJECT_NAME} INTERFACE)
    target_link_libraries(${PROJECT_NAME} INTERFACE emdevif_core rmdev_math emdevif_compile_options)
    target_include_directories(${PROJECT_NAME} INTERFACE inc)
endif ()
