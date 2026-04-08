cmake_minimum_required(VERSION 3.13)
project(eigen)

set(Eigen3_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR})

add_library(${PROJECT_NAME} INTERFACE
        ../../../../app/targets/dart_rack/lcd_init.cpp
        ../../../../app/targets/dart_rack/lcd_init.cpp)

target_include_directories(${PROJECT_NAME} INTERFACE
    ${Eigen3_INCLUDE_DIR}
)