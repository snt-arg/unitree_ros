cmake_minimum_required(VERSION 2.14)
project(faceLightClient)

set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -march=native -Wall")

include_directories(include)
link_directories(lib)

add_executable(faceLightClient main.cpp)
target_link_libraries(faceLightClient libfaceLight_SDK_amd64.so)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
