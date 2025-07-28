cmake_minimum_required(VERSION 3.28)

project(rmdev_control_algorithm CXX)

add_library(${PROJECT_NAME} OBJECT)

target_link_libraries(${PROJECT_NAME} PUBLIC emdevif_core rmdev_math)

target_sources(${PROJECT_NAME} PUBLIC
    FILE_SET rmdev_control_algorithm_modules TYPE CXX_MODULES FILES
        Pid.cppm
)
