cmake_minimum_required (VERSION 2.6)
project (super_pix)

if(MSVC)
    add_compile_options("/W4" "$<$<CONFIG:RELEASE>:/O2>")
else()
    add_compile_options("-Wall" "-Wextra" "$<$<CONFIG:RELEASE>:-O3>")
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        add_compile_options("-stdlib=libc++")
    else()
        # nothing special for gcc at the moment
    endif()
endif()

#For writing png files as output
find_package(PNG REQUIRED)

add_executable(${PROJECT_NAME} approximator.cpp curve.cpp my_point.cpp polygonalizer.cpp shifter.cpp sorter.cpp
	bound_box_shifter.cpp main.cpp path.cpp segment.cpp sorter2.cpp
	splitter.cpp superpix_helper_func.cpp write_png.cpp)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(SYSTEM ${PNG_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} ${PNG_LIBRARY})
