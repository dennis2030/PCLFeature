# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bean/Projects/PCLFeature

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bean/Projects/PCLFeature

# Include any dependencies generated for this target.
include CMakeFiles/PCLFeature.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/PCLFeature.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/PCLFeature.dir/flags.make

CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o: CMakeFiles/PCLFeature.dir/flags.make
CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o: PCLFeature.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bean/Projects/PCLFeature/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o -c /home/bean/Projects/PCLFeature/PCLFeature.cpp

CMakeFiles/PCLFeature.dir/PCLFeature.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PCLFeature.dir/PCLFeature.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bean/Projects/PCLFeature/PCLFeature.cpp > CMakeFiles/PCLFeature.dir/PCLFeature.cpp.i

CMakeFiles/PCLFeature.dir/PCLFeature.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PCLFeature.dir/PCLFeature.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bean/Projects/PCLFeature/PCLFeature.cpp -o CMakeFiles/PCLFeature.dir/PCLFeature.cpp.s

CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o.requires:
.PHONY : CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o.requires

CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o.provides: CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o.requires
	$(MAKE) -f CMakeFiles/PCLFeature.dir/build.make CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o.provides.build
.PHONY : CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o.provides

CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o.provides.build: CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o

# Object files for target PCLFeature
PCLFeature_OBJECTS = \
"CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o"

# External object files for target PCLFeature
PCLFeature_EXTERNAL_OBJECTS =

PCLFeature: CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o
PCLFeature: /usr/lib/libboost_system-mt.so
PCLFeature: /usr/lib/libboost_filesystem-mt.so
PCLFeature: /usr/lib/libboost_thread-mt.so
PCLFeature: /usr/lib/libboost_date_time-mt.so
PCLFeature: /usr/lib/libboost_iostreams-mt.so
PCLFeature: /usr/lib/libboost_serialization-mt.so
PCLFeature: /usr/lib/libpcl_common.so
PCLFeature: /usr/lib/libpcl_octree.so
PCLFeature: /usr/lib/libOpenNI.so
PCLFeature: /usr/lib/libvtkCommon.so.5.8.0
PCLFeature: /usr/lib/libvtkRendering.so.5.8.0
PCLFeature: /usr/lib/libvtkHybrid.so.5.8.0
PCLFeature: /usr/lib/libvtkCharts.so.5.8.0
PCLFeature: /usr/lib/libpcl_io.so
PCLFeature: /usr/lib/libflann_cpp_s.a
PCLFeature: /usr/lib/libpcl_kdtree.so
PCLFeature: /usr/lib/libpcl_search.so
PCLFeature: /usr/lib/libqhull.so
PCLFeature: /usr/lib/libpcl_surface.so
PCLFeature: /usr/lib/libpcl_sample_consensus.so
PCLFeature: /usr/lib/libpcl_filters.so
PCLFeature: /usr/lib/libpcl_features.so
PCLFeature: /usr/lib/libboost_system-mt.so
PCLFeature: /usr/lib/libboost_filesystem-mt.so
PCLFeature: /usr/lib/libboost_thread-mt.so
PCLFeature: /usr/lib/libboost_date_time-mt.so
PCLFeature: /usr/lib/libboost_iostreams-mt.so
PCLFeature: /usr/lib/libboost_serialization-mt.so
PCLFeature: /usr/lib/libqhull.so
PCLFeature: /usr/lib/libOpenNI.so
PCLFeature: /usr/lib/libflann_cpp_s.a
PCLFeature: /usr/lib/libvtkCommon.so.5.8.0
PCLFeature: /usr/lib/libvtkRendering.so.5.8.0
PCLFeature: /usr/lib/libvtkHybrid.so.5.8.0
PCLFeature: /usr/lib/libvtkCharts.so.5.8.0
PCLFeature: /usr/lib/libboost_system-mt.so
PCLFeature: /usr/lib/libboost_filesystem-mt.so
PCLFeature: /usr/lib/libboost_thread-mt.so
PCLFeature: /usr/lib/libboost_date_time-mt.so
PCLFeature: /usr/lib/libboost_iostreams-mt.so
PCLFeature: /usr/lib/libboost_serialization-mt.so
PCLFeature: /usr/lib/libpcl_common.so
PCLFeature: /usr/lib/libOpenNI.so
PCLFeature: /usr/lib/libvtkCommon.so.5.8.0
PCLFeature: /usr/lib/libvtkRendering.so.5.8.0
PCLFeature: /usr/lib/libvtkHybrid.so.5.8.0
PCLFeature: /usr/lib/libvtkCharts.so.5.8.0
PCLFeature: /usr/lib/libpcl_io.so
PCLFeature: /usr/lib/libboost_system-mt.so
PCLFeature: /usr/lib/libboost_filesystem-mt.so
PCLFeature: /usr/lib/libboost_thread-mt.so
PCLFeature: /usr/lib/libboost_date_time-mt.so
PCLFeature: /usr/lib/libboost_iostreams-mt.so
PCLFeature: /usr/lib/libboost_serialization-mt.so
PCLFeature: /usr/lib/libpcl_common.so
PCLFeature: /usr/lib/libpcl_octree.so
PCLFeature: /usr/lib/libboost_system-mt.so
PCLFeature: /usr/lib/libboost_filesystem-mt.so
PCLFeature: /usr/lib/libboost_thread-mt.so
PCLFeature: /usr/lib/libboost_date_time-mt.so
PCLFeature: /usr/lib/libboost_iostreams-mt.so
PCLFeature: /usr/lib/libboost_serialization-mt.so
PCLFeature: /usr/lib/libpcl_common.so
PCLFeature: /usr/lib/libpcl_kdtree.so
PCLFeature: /usr/lib/libpcl_search.so
PCLFeature: /usr/lib/libpcl_surface.so
PCLFeature: /usr/lib/libpcl_features.so
PCLFeature: /usr/lib/libpcl_sample_consensus.so
PCLFeature: /usr/lib/libpcl_filters.so
PCLFeature: /usr/lib/libvtkViews.so.5.8.0
PCLFeature: /usr/lib/libvtkInfovis.so.5.8.0
PCLFeature: /usr/lib/libvtkWidgets.so.5.8.0
PCLFeature: /usr/lib/libvtkHybrid.so.5.8.0
PCLFeature: /usr/lib/libvtkParallel.so.5.8.0
PCLFeature: /usr/lib/libvtkVolumeRendering.so.5.8.0
PCLFeature: /usr/lib/libvtkRendering.so.5.8.0
PCLFeature: /usr/lib/libvtkGraphics.so.5.8.0
PCLFeature: /usr/lib/libvtkImaging.so.5.8.0
PCLFeature: /usr/lib/libvtkIO.so.5.8.0
PCLFeature: /usr/lib/libvtkFiltering.so.5.8.0
PCLFeature: /usr/lib/libvtkCommon.so.5.8.0
PCLFeature: /usr/lib/libvtksys.so.5.8.0
PCLFeature: CMakeFiles/PCLFeature.dir/build.make
PCLFeature: CMakeFiles/PCLFeature.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable PCLFeature"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PCLFeature.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/PCLFeature.dir/build: PCLFeature
.PHONY : CMakeFiles/PCLFeature.dir/build

CMakeFiles/PCLFeature.dir/requires: CMakeFiles/PCLFeature.dir/PCLFeature.cpp.o.requires
.PHONY : CMakeFiles/PCLFeature.dir/requires

CMakeFiles/PCLFeature.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PCLFeature.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PCLFeature.dir/clean

CMakeFiles/PCLFeature.dir/depend:
	cd /home/bean/Projects/PCLFeature && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bean/Projects/PCLFeature /home/bean/Projects/PCLFeature /home/bean/Projects/PCLFeature /home/bean/Projects/PCLFeature /home/bean/Projects/PCLFeature/CMakeFiles/PCLFeature.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PCLFeature.dir/depend
