cmake_minimum_required(VERSION 2.8.4)
project(vncpp-lib)

set(CMAKE_SUPPRESS_REGENERATION TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
	include/vn/exceptions.h
	include/vn/int.h
	include/vn/utilities.h
	include/vn/common/isimpleport.h
	include/vn/data/error_detection.h
	include/vn/math/consts.h
	include/vn/math/math.h
	include/vn/math/conversions.h
	include/vn/math/matrix.h
	include/vn/math/vector.h
	include/vn/math/kinematics.h
	include/vn/protocol/uart.h
	include/vn/sensors/sensors.h
	include/vn/sensors/searcher.h
	include/vn/util/memoryport.h
	include/vn/util/nocopy.h
	include/vn/xplat/criticalsection.h
	include/vn/xplat/event.h
	include/vn/xplat/serialport.h
	include/vn/xplat/signal.h
	include/vn/xplat/thread.h
	include/vn/xplat/time.h
	src/vn/math/conversions.cpp
	src/vn/protocol/uart.cpp
	src/vn/sensors/sensors.cpp
	src/vn/sensors/searcher.cpp
	src/vn/util/memoryport.cpp
	src/vn/xplat/criticalsection.cpp
	src/vn/xplat/event.cpp
	src/vn/xplat/serialport.cpp
	src/vn/xplat/thread.cpp
	src/vn/xplat/time.cpp
	src/vn/data/error_detection.cpp
	src/vn/utilities.cpp
	src/vn/common/isimpleport.cpp)

include_directories(include)

add_library(vncpp STATIC ${SOURCE_FILES})

if (WIN32)
	target_link_libraries(vncpp PRIVATE Setupapi)
endif()
