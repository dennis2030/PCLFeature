cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(convert2Range)
find_package(PCL 1.3 REQUIRED COMPONENTS common io kdtree search surface features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(convert2Range convert2Range.cpp)
target_link_libraries(convert2Range ${PCL_LIBRARIES} )

