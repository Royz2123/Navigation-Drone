cmake_minimum_required(VERSION 3.2)

project(fouriertrack)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(fouriertrack main.cpp)
target_link_libraries(fouriertrack ${OpenCV_LIBS})

if (CMAKE_COMPILER_IS_GNUCC)
	set_property( TARGET fouriertrack APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )
endif (CMAKE_COMPILER_IS_GNUCC)
