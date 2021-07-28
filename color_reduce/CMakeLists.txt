cmake_minimum_required(VERSION 3.19)
project(untitled1)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
add_executable(untitled1 main.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})