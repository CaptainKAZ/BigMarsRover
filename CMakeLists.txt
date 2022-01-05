cmake_minimum_required(VERSION 3.21)
project(BigMarsRover)

set(CMAKE_CXX_STANDARD 14)
#link_directories("./spine/DynamixelSDK")
add_subdirectory(spine)

add_executable(BigMarsRover main.cpp)

target_link_libraries(BigMarsRover spine)