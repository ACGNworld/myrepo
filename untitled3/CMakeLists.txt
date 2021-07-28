cmake_minimum_required(VERSION 3.19)
project(untitled)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
add_executable(untitled main.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}} ${OpenCV_LIBS})