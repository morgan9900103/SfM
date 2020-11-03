cmake_minimum_required(VERSION 2.6)
project(sfm)

add_executable(sfm main.cpp)

install(TARGETS sfm RUNTIME DESTINATION bin)
