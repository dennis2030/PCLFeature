cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCLFeature)
find_package(PCL 1.6 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(PCLFeature PCLFeature.cpp)
target_link_libraries(PCLFeature ${PCL_COMMON_LIBRARIES}
${PCL_IO_LIBRARIES})
