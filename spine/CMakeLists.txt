cmake_minimum_required(VERSION 3.21)
set(CMAKE_CXX_STANDARD 14)

project(spine)


add_library(dxl SHARED IMPORTED)
set_target_properties(dxl PROPERTIES
        IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/DynamixelSDK/libdxl_x64_cpp.so
        INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/DynamixelSDK/include/dynamixel_sdk"
        )

add_library(${PROJECT_NAME} minicheetah_motor.cpp robomodule_motor.cpp socketcan.cpp dynamixel_motor.cpp dynamixel_motor.h)
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} pthread dxl)