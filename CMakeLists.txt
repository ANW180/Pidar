# CMake requires we pecify a minimum version, current min is 2.8
if(WIN32)
	# Require 2.8.2 for configuring Debug/Release configuration options in 
	# Visual Studio
	cmake_minimum_required (VERSION 2.8.2) 
else()
	cmake_minimum_required (VERSION 2.8)
endif(WIN32)

project(LidarSample)

add_executable(LidarSample src/LidarSample.cpp)

include_directories(/usr/local/include/urg_c)
target_link_libraries(LidarSample /usr/local/lib/liburg_c.a)
