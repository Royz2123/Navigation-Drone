cmake_minimum_required(VERSION 3.2)

project(sport_drone)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(
	sport_drone
	main.cpp
	location.cpp
	channels.cpp
	gui.cpp
	detection.cpp
	pid.cpp
	quad_serial.cpp
)
target_link_libraries(sport_drone ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

if (CMAKE_COMPILER_IS_GNUCC)
	set_property( TARGET sport_drone APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )
endif (CMAKE_COMPILER_IS_GNUCC)
