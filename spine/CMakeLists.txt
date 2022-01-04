include_directories("./")

add_library(socketcan STATIC socketcan.cpp)

add_executable(spine minicheetah_motor.cpp robomodule_motor.cpp robomodule_motor.h)

target_link_libraries(spine socketcan pthread lcm)