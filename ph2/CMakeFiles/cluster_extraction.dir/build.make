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
CMAKE_COMMAND = /usr/local/Cellar/cmake/2.8.10.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/2.8.10.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/local/Cellar/cmake/2.8.10.2/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/younes/Desktop/ph

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/younes/Desktop/ph

# Include any dependencies generated for this target.
include CMakeFiles/cluster_extraction.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cluster_extraction.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cluster_extraction.dir/flags.make

CMakeFiles/cluster_extraction.dir/robot.cpp.o: CMakeFiles/cluster_extraction.dir/flags.make
CMakeFiles/cluster_extraction.dir/robot.cpp.o: robot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/younes/Desktop/ph/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/cluster_extraction.dir/robot.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cluster_extraction.dir/robot.cpp.o -c /Users/younes/Desktop/ph/robot.cpp

CMakeFiles/cluster_extraction.dir/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cluster_extraction.dir/robot.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/younes/Desktop/ph/robot.cpp > CMakeFiles/cluster_extraction.dir/robot.cpp.i

CMakeFiles/cluster_extraction.dir/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cluster_extraction.dir/robot.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/younes/Desktop/ph/robot.cpp -o CMakeFiles/cluster_extraction.dir/robot.cpp.s

CMakeFiles/cluster_extraction.dir/robot.cpp.o.requires:
.PHONY : CMakeFiles/cluster_extraction.dir/robot.cpp.o.requires

CMakeFiles/cluster_extraction.dir/robot.cpp.o.provides: CMakeFiles/cluster_extraction.dir/robot.cpp.o.requires
	$(MAKE) -f CMakeFiles/cluster_extraction.dir/build.make CMakeFiles/cluster_extraction.dir/robot.cpp.o.provides.build
.PHONY : CMakeFiles/cluster_extraction.dir/robot.cpp.o.provides

CMakeFiles/cluster_extraction.dir/robot.cpp.o.provides.build: CMakeFiles/cluster_extraction.dir/robot.cpp.o

# Object files for target cluster_extraction
cluster_extraction_OBJECTS = \
"CMakeFiles/cluster_extraction.dir/robot.cpp.o"

# External object files for target cluster_extraction
cluster_extraction_EXTERNAL_OBJECTS =

cluster_extraction: CMakeFiles/cluster_extraction.dir/robot.cpp.o
cluster_extraction: CMakeFiles/cluster_extraction.dir/build.make
cluster_extraction: /opt/local/lib/libboost_system-mt.dylib
cluster_extraction: /opt/local/lib/libboost_filesystem-mt.dylib
cluster_extraction: /opt/local/lib/libboost_thread-mt.dylib
cluster_extraction: /opt/local/lib/libboost_date_time-mt.dylib
cluster_extraction: /opt/local/lib/libboost_iostreams-mt.dylib
cluster_extraction: /usr/local/lib/libpcl_common.dylib
cluster_extraction: /usr/local/lib/libpcl_octree.dylib
cluster_extraction: /usr/lib/libOpenNI.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkCommon.5.10.0.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkRendering.5.10.0.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkHybrid.5.10.0.dylib
cluster_extraction: /usr/local/lib/libpcl_io.dylib
cluster_extraction: /usr/local/Cellar/flann/1.8.2/lib/libflann_cpp_s.a
cluster_extraction: /usr/local/lib/libpcl_kdtree.dylib
cluster_extraction: /usr/local/lib/libpcl_search.dylib
cluster_extraction: /usr/local/lib/libpcl_sample_consensus.dylib
cluster_extraction: /usr/local/lib/libpcl_filters.dylib
cluster_extraction: /usr/local/lib/libpcl_segmentation.dylib
cluster_extraction: /usr/local/lib/libpcl_visualization.dylib
cluster_extraction: /usr/local/lib/libpcl_features.dylib
cluster_extraction: /usr/local/lib/libqhullstatic.a
cluster_extraction: /usr/local/lib/libpcl_surface.dylib
cluster_extraction: /usr/local/lib/libpcl_registration.dylib
cluster_extraction: /usr/local/lib/libpcl_keypoints.dylib
cluster_extraction: /usr/local/lib/libpcl_tracking.dylib
cluster_extraction: /usr/local/lib/libpcl_apps.dylib
cluster_extraction: /usr/local/lib/libopencv_calib3d.dylib
cluster_extraction: /usr/local/lib/libopencv_contrib.dylib
cluster_extraction: /usr/local/lib/libopencv_core.dylib
cluster_extraction: /usr/local/lib/libopencv_features2d.dylib
cluster_extraction: /usr/local/lib/libopencv_flann.dylib
cluster_extraction: /usr/local/lib/libopencv_gpu.dylib
cluster_extraction: /usr/local/lib/libopencv_highgui.dylib
cluster_extraction: /usr/local/lib/libopencv_imgproc.dylib
cluster_extraction: /usr/local/lib/libopencv_legacy.dylib
cluster_extraction: /usr/local/lib/libopencv_ml.dylib
cluster_extraction: /usr/local/lib/libopencv_nonfree.dylib
cluster_extraction: /usr/local/lib/libopencv_objdetect.dylib
cluster_extraction: /usr/local/lib/libopencv_photo.dylib
cluster_extraction: /usr/local/lib/libopencv_stitching.dylib
cluster_extraction: /usr/local/lib/libopencv_ts.dylib
cluster_extraction: /usr/local/lib/libopencv_video.dylib
cluster_extraction: /usr/local/lib/libopencv_videostab.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkRendering.5.10.0.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkGraphics.5.10.0.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkImaging.5.10.0.dylib
cluster_extraction: /opt/local/lib/libQtGui.dylib
cluster_extraction: /opt/local/lib/libQtSql.dylib
cluster_extraction: /opt/local/lib/libQtCore.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkIO.5.10.0.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkFiltering.5.10.0.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtkCommon.5.10.0.dylib
cluster_extraction: /opt/local/lib/vtk-5.10/libvtksys.5.10.0.dylib
cluster_extraction: CMakeFiles/cluster_extraction.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable cluster_extraction"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cluster_extraction.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cluster_extraction.dir/build: cluster_extraction
.PHONY : CMakeFiles/cluster_extraction.dir/build

CMakeFiles/cluster_extraction.dir/requires: CMakeFiles/cluster_extraction.dir/robot.cpp.o.requires
.PHONY : CMakeFiles/cluster_extraction.dir/requires

CMakeFiles/cluster_extraction.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cluster_extraction.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cluster_extraction.dir/clean

CMakeFiles/cluster_extraction.dir/depend:
	cd /Users/younes/Desktop/ph && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/younes/Desktop/ph /Users/younes/Desktop/ph /Users/younes/Desktop/ph /Users/younes/Desktop/ph /Users/younes/Desktop/ph/CMakeFiles/cluster_extraction.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cluster_extraction.dir/depend

