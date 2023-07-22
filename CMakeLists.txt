cmake_minimum_required(VERSION 3.25)
project(Hough_transform)

set(CMAKE_CXX_STANDARD 14)

add_library(lib src/Hough.cpp)

#OpenCv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Hough_transform
        main.cpp)
target_link_libraries(${PROJECT_NAME} lib ${OpenCV_LIBS})
