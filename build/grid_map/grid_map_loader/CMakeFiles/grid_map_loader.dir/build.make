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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/namwob44/new_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/namwob44/new_ws/build

# Include any dependencies generated for this target.
include grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/depend.make

# Include the progress variables for this target.
include grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/progress.make

# Include the compile flags for this target's objects.
include grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/flags.make

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/flags.make
grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o: /home/namwob44/new_ws/src/grid_map/grid_map_loader/src/grid_map_loader_node.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/namwob44/new_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o"
	cd /home/namwob44/new_ws/build/grid_map/grid_map_loader && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o -c /home/namwob44/new_ws/src/grid_map/grid_map_loader/src/grid_map_loader_node.cpp

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.i"
	cd /home/namwob44/new_ws/build/grid_map/grid_map_loader && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/namwob44/new_ws/src/grid_map/grid_map_loader/src/grid_map_loader_node.cpp > CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.i

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.s"
	cd /home/namwob44/new_ws/build/grid_map/grid_map_loader && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/namwob44/new_ws/src/grid_map/grid_map_loader/src/grid_map_loader_node.cpp -o CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.s

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o.requires:
.PHONY : grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o.requires

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o.provides: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o.requires
	$(MAKE) -f grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/build.make grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o.provides.build
.PHONY : grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o.provides

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o.provides.build: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/flags.make
grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o: /home/namwob44/new_ws/src/grid_map/grid_map_loader/src/GridMapLoader.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/namwob44/new_ws/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o"
	cd /home/namwob44/new_ws/build/grid_map/grid_map_loader && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o -c /home/namwob44/new_ws/src/grid_map/grid_map_loader/src/GridMapLoader.cpp

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.i"
	cd /home/namwob44/new_ws/build/grid_map/grid_map_loader && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/namwob44/new_ws/src/grid_map/grid_map_loader/src/GridMapLoader.cpp > CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.i

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.s"
	cd /home/namwob44/new_ws/build/grid_map/grid_map_loader && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/namwob44/new_ws/src/grid_map/grid_map_loader/src/GridMapLoader.cpp -o CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.s

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o.requires:
.PHONY : grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o.requires

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o.provides: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o.requires
	$(MAKE) -f grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/build.make grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o.provides.build
.PHONY : grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o.provides

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o.provides.build: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o

# Object files for target grid_map_loader
grid_map_loader_OBJECTS = \
"CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o" \
"CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o"

# External object files for target grid_map_loader
grid_map_loader_EXTERNAL_OBJECTS =

/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/build.make
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /home/namwob44/new_ws/devel/lib/libgrid_map_ros.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libroscpp.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /home/namwob44/new_ws/devel/lib/libgrid_map_core.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libcv_bridge.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librosconsole.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/liblog4cxx.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librostime.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libcpp_common.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /home/namwob44/new_ws/devel/lib/libgrid_map_cv.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /home/namwob44/new_ws/devel/lib/libgrid_map_core.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libcv_bridge.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librosbag.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librosbag_storage.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libroslz4.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/liblz4.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libtopic_tools.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libroscpp.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librosconsole.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/liblog4cxx.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/librostime.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /opt/ros/indigo/lib/libcpp_common.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader"
	cd /home/namwob44/new_ws/build/grid_map/grid_map_loader && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/grid_map_loader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/build: /home/namwob44/new_ws/devel/lib/grid_map_loader/grid_map_loader
.PHONY : grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/build

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/requires: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/grid_map_loader_node.cpp.o.requires
grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/requires: grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/src/GridMapLoader.cpp.o.requires
.PHONY : grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/requires

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/clean:
	cd /home/namwob44/new_ws/build/grid_map/grid_map_loader && $(CMAKE_COMMAND) -P CMakeFiles/grid_map_loader.dir/cmake_clean.cmake
.PHONY : grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/clean

grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/depend:
	cd /home/namwob44/new_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/namwob44/new_ws/src /home/namwob44/new_ws/src/grid_map/grid_map_loader /home/namwob44/new_ws/build /home/namwob44/new_ws/build/grid_map/grid_map_loader /home/namwob44/new_ws/build/grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : grid_map/grid_map_loader/CMakeFiles/grid_map_loader.dir/depend

