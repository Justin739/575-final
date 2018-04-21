cmake_minimum_required(VERSION 3.9)
project(575_Final_Project)

set(CMAKE_CXX_STANDARD 11)
add_executable(575_Final_Project main.cpp Capturer.cpp Capturer.h coord.c Processor.cpp Processor.h)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenALPR REQUIRED openalpr)

find_package(OpenCV REQUIRED)
find_package (Threads)
include_directories(${OpenCV_INCLUDE_DIRS})
target_include_directories(575_Final_Project PUBLIC ${OpenALPR_INCLUDE_DIRS})
target_link_libraries(575_Final_Project ${OpenCV_LIBS} ${OpenALPR_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(575_Final_Project PUBLIC ${OpenALPR_CFLAGS_OTHER})

