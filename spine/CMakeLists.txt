include_directories("./")

add_library(socketcan STATIC socketcan.cpp)

add_executable(spine motor.cpp)

target_link_libraries(spine socketcan pthread lcm)