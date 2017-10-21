cmake_minimum_required (VERSION 2.6)
project (super_pix)

#For writing png files as output
find_package(PNG REQUIRED)

add_executable(${PROJECT_NAME} approximator.cpp curve.cpp my_point.cpp polygonalizer.cpp shifter.cpp sorter.cpp
	bound_box_shifter.cpp main.cpp path.cpp segment.cpp sorter2.cpp
	splitter.cpp superpix_helper_func.cpp write_png.cpp)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(SYSTEM ${PNG_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} ${PNG_LIBRARY})
