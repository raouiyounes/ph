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
CMAKE_SOURCE_DIR = /Users/younes/Desktop/bigDir/ph

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/younes/Desktop/bigDir/ph

# Include any dependencies generated for this target.
include CMakeFiles/ph.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ph.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ph.dir/flags.make

CMakeFiles/ph.dir/coucou.cpp.o: CMakeFiles/ph.dir/flags.make
CMakeFiles/ph.dir/coucou.cpp.o: coucou.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/younes/Desktop/bigDir/ph/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ph.dir/coucou.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ph.dir/coucou.cpp.o -c /Users/younes/Desktop/bigDir/ph/coucou.cpp

CMakeFiles/ph.dir/coucou.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ph.dir/coucou.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/younes/Desktop/bigDir/ph/coucou.cpp > CMakeFiles/ph.dir/coucou.cpp.i

CMakeFiles/ph.dir/coucou.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ph.dir/coucou.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/younes/Desktop/bigDir/ph/coucou.cpp -o CMakeFiles/ph.dir/coucou.cpp.s

CMakeFiles/ph.dir/coucou.cpp.o.requires:
.PHONY : CMakeFiles/ph.dir/coucou.cpp.o.requires

CMakeFiles/ph.dir/coucou.cpp.o.provides: CMakeFiles/ph.dir/coucou.cpp.o.requires
	$(MAKE) -f CMakeFiles/ph.dir/build.make CMakeFiles/ph.dir/coucou.cpp.o.provides.build
.PHONY : CMakeFiles/ph.dir/coucou.cpp.o.provides

CMakeFiles/ph.dir/coucou.cpp.o.provides.build: CMakeFiles/ph.dir/coucou.cpp.o

# Object files for target ph
ph_OBJECTS = \
"CMakeFiles/ph.dir/coucou.cpp.o"

# External object files for target ph
ph_EXTERNAL_OBJECTS =

ph: CMakeFiles/ph.dir/coucou.cpp.o
ph: CMakeFiles/ph.dir/build.make
ph: /opt/local/lib/libboost_system-mt.dylib
ph: /opt/local/lib/libboost_filesystem-mt.dylib
ph: /opt/local/lib/libboost_thread-mt.dylib
ph: /opt/local/lib/libboost_date_time-mt.dylib
ph: /opt/local/lib/libboost_iostreams-mt.dylib
ph: /usr/local/lib/libpcl_common.dylib
ph: /usr/local/lib/libpcl_octree.dylib
ph: /usr/lib/libOpenNI.dylib
ph: /opt/local/lib/vtk-5.10/libvtkCommon.5.10.0.dylib
ph: /opt/local/lib/vtk-5.10/libvtkRendering.5.10.0.dylib
ph: /opt/local/lib/vtk-5.10/libvtkHybrid.5.10.0.dylib
ph: /usr/local/lib/libpcl_io.dylib
ph: /usr/local/Cellar/flann/1.8.2/lib/libflann_cpp_s.a
ph: /usr/local/lib/libpcl_kdtree.dylib
ph: /usr/local/lib/libpcl_search.dylib
ph: /usr/local/lib/libpcl_sample_consensus.dylib
ph: /usr/local/lib/libpcl_filters.dylib
ph: /usr/local/lib/libpcl_segmentation.dylib
ph: /usr/local/lib/libpcl_visualization.dylib
ph: /usr/local/lib/libpcl_features.dylib
ph: /usr/local/lib/libqhullstatic.a
ph: /usr/local/lib/libpcl_surface.dylib
ph: /usr/local/lib/libpcl_registration.dylib
ph: /usr/local/lib/libpcl_keypoints.dylib
ph: /usr/local/lib/libpcl_tracking.dylib
ph: /usr/local/lib/libpcl_apps.dylib
ph: /usr/local/lib/libopencv_calib3d.dylib
ph: /usr/local/lib/libopencv_contrib.dylib
ph: /usr/local/lib/libopencv_core.dylib
ph: /usr/local/lib/libopencv_features2d.dylib
ph: /usr/local/lib/libopencv_flann.dylib
ph: /usr/local/lib/libopencv_gpu.dylib
ph: /usr/local/lib/libopencv_highgui.dylib
ph: /usr/local/lib/libopencv_imgproc.dylib
ph: /usr/local/lib/libopencv_legacy.dylib
ph: /usr/local/lib/libopencv_ml.dylib
ph: /usr/local/lib/libopencv_nonfree.dylib
ph: /usr/local/lib/libopencv_objdetect.dylib
ph: /usr/local/lib/libopencv_photo.dylib
ph: /usr/local/lib/libopencv_stitching.dylib
ph: /usr/local/lib/libopencv_ts.dylib
ph: /usr/local/lib/libopencv_video.dylib
ph: /usr/local/lib/libopencv_videostab.dylib
ph: /opt/local/lib/vtk-5.10/libvtkRendering.5.10.0.dylib
ph: /opt/local/lib/vtk-5.10/libvtkGraphics.5.10.0.dylib
ph: /opt/local/lib/vtk-5.10/libvtkImaging.5.10.0.dylib
ph: /opt/local/lib/libQtGui.dylib
ph: /opt/local/lib/libQtSql.dylib
ph: /opt/local/lib/libQtCore.dylib
ph: /opt/local/lib/vtk-5.10/libvtkIO.5.10.0.dylib
ph: /opt/local/lib/vtk-5.10/libvtkFiltering.5.10.0.dylib
ph: /opt/local/lib/vtk-5.10/libvtkCommon.5.10.0.dylib
ph: /opt/local/lib/vtk-5.10/libvtksys.5.10.0.dylib
ph: CMakeFiles/ph.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ph"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ph.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ph.dir/build: ph
.PHONY : CMakeFiles/ph.dir/build

CMakeFiles/ph.dir/requires: CMakeFiles/ph.dir/coucou.cpp.o.requires
.PHONY : CMakeFiles/ph.dir/requires

CMakeFiles/ph.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ph.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ph.dir/clean

CMakeFiles/ph.dir/depend:
	cd /Users/younes/Desktop/bigDir/ph && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/younes/Desktop/bigDir/ph /Users/younes/Desktop/bigDir/ph /Users/younes/Desktop/bigDir/ph /Users/younes/Desktop/bigDir/ph /Users/younes/Desktop/bigDir/ph/CMakeFiles/ph.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ph.dir/depend

