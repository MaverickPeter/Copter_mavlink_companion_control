# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/maverickp/zhongkongbei/artag-try

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maverickp/zhongkongbei/artag-try/build

# Include any dependencies generated for this target.
include CMakeFiles/marker_AR.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/marker_AR.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/marker_AR.dir/flags.make

CMakeFiles/marker_AR.dir/main.cpp.o: CMakeFiles/marker_AR.dir/flags.make
CMakeFiles/marker_AR.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maverickp/zhongkongbei/artag-try/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/marker_AR.dir/main.cpp.o"
	/usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/marker_AR.dir/main.cpp.o -c /home/maverickp/zhongkongbei/artag-try/main.cpp

CMakeFiles/marker_AR.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/marker_AR.dir/main.cpp.i"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maverickp/zhongkongbei/artag-try/main.cpp > CMakeFiles/marker_AR.dir/main.cpp.i

CMakeFiles/marker_AR.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/marker_AR.dir/main.cpp.s"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maverickp/zhongkongbei/artag-try/main.cpp -o CMakeFiles/marker_AR.dir/main.cpp.s

CMakeFiles/marker_AR.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/marker_AR.dir/main.cpp.o.requires

CMakeFiles/marker_AR.dir/main.cpp.o.provides: CMakeFiles/marker_AR.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/marker_AR.dir/build.make CMakeFiles/marker_AR.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/marker_AR.dir/main.cpp.o.provides

CMakeFiles/marker_AR.dir/main.cpp.o.provides.build: CMakeFiles/marker_AR.dir/main.cpp.o


CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o: CMakeFiles/marker_AR.dir/flags.make
CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o: ../MarkerDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maverickp/zhongkongbei/artag-try/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o"
	/usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o -c /home/maverickp/zhongkongbei/artag-try/MarkerDetector.cpp

CMakeFiles/marker_AR.dir/MarkerDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/marker_AR.dir/MarkerDetector.cpp.i"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maverickp/zhongkongbei/artag-try/MarkerDetector.cpp > CMakeFiles/marker_AR.dir/MarkerDetector.cpp.i

CMakeFiles/marker_AR.dir/MarkerDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/marker_AR.dir/MarkerDetector.cpp.s"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maverickp/zhongkongbei/artag-try/MarkerDetector.cpp -o CMakeFiles/marker_AR.dir/MarkerDetector.cpp.s

CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o.requires:

.PHONY : CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o.requires

CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o.provides: CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/marker_AR.dir/build.make CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o.provides.build
.PHONY : CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o.provides

CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o.provides.build: CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o


CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o: CMakeFiles/marker_AR.dir/flags.make
CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o: ../GeometryTypes.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maverickp/zhongkongbei/artag-try/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o"
	/usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o -c /home/maverickp/zhongkongbei/artag-try/GeometryTypes.cpp

CMakeFiles/marker_AR.dir/GeometryTypes.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/marker_AR.dir/GeometryTypes.cpp.i"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maverickp/zhongkongbei/artag-try/GeometryTypes.cpp > CMakeFiles/marker_AR.dir/GeometryTypes.cpp.i

CMakeFiles/marker_AR.dir/GeometryTypes.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/marker_AR.dir/GeometryTypes.cpp.s"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maverickp/zhongkongbei/artag-try/GeometryTypes.cpp -o CMakeFiles/marker_AR.dir/GeometryTypes.cpp.s

CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o.requires:

.PHONY : CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o.requires

CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o.provides: CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o.requires
	$(MAKE) -f CMakeFiles/marker_AR.dir/build.make CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o.provides.build
.PHONY : CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o.provides

CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o.provides.build: CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o


CMakeFiles/marker_AR.dir/Marker.cpp.o: CMakeFiles/marker_AR.dir/flags.make
CMakeFiles/marker_AR.dir/Marker.cpp.o: ../Marker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maverickp/zhongkongbei/artag-try/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/marker_AR.dir/Marker.cpp.o"
	/usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/marker_AR.dir/Marker.cpp.o -c /home/maverickp/zhongkongbei/artag-try/Marker.cpp

CMakeFiles/marker_AR.dir/Marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/marker_AR.dir/Marker.cpp.i"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maverickp/zhongkongbei/artag-try/Marker.cpp > CMakeFiles/marker_AR.dir/Marker.cpp.i

CMakeFiles/marker_AR.dir/Marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/marker_AR.dir/Marker.cpp.s"
	/usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maverickp/zhongkongbei/artag-try/Marker.cpp -o CMakeFiles/marker_AR.dir/Marker.cpp.s

CMakeFiles/marker_AR.dir/Marker.cpp.o.requires:

.PHONY : CMakeFiles/marker_AR.dir/Marker.cpp.o.requires

CMakeFiles/marker_AR.dir/Marker.cpp.o.provides: CMakeFiles/marker_AR.dir/Marker.cpp.o.requires
	$(MAKE) -f CMakeFiles/marker_AR.dir/build.make CMakeFiles/marker_AR.dir/Marker.cpp.o.provides.build
.PHONY : CMakeFiles/marker_AR.dir/Marker.cpp.o.provides

CMakeFiles/marker_AR.dir/Marker.cpp.o.provides.build: CMakeFiles/marker_AR.dir/Marker.cpp.o


# Object files for target marker_AR
marker_AR_OBJECTS = \
"CMakeFiles/marker_AR.dir/main.cpp.o" \
"CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o" \
"CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o" \
"CMakeFiles/marker_AR.dir/Marker.cpp.o"

# External object files for target marker_AR
marker_AR_EXTERNAL_OBJECTS =

marker_AR: CMakeFiles/marker_AR.dir/main.cpp.o
marker_AR: CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o
marker_AR: CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o
marker_AR: CMakeFiles/marker_AR.dir/Marker.cpp.o
marker_AR: CMakeFiles/marker_AR.dir/build.make
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
marker_AR: /usr/lib/x86_64-linux-gnu/libGLU.so
marker_AR: /usr/lib/x86_64-linux-gnu/libGL.so
marker_AR: /usr/lib/x86_64-linux-gnu/libglut.so
marker_AR: /usr/lib/x86_64-linux-gnu/libXmu.so
marker_AR: /usr/lib/x86_64-linux-gnu/libXi.so
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
marker_AR: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
marker_AR: CMakeFiles/marker_AR.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maverickp/zhongkongbei/artag-try/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable marker_AR"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/marker_AR.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/marker_AR.dir/build: marker_AR

.PHONY : CMakeFiles/marker_AR.dir/build

CMakeFiles/marker_AR.dir/requires: CMakeFiles/marker_AR.dir/main.cpp.o.requires
CMakeFiles/marker_AR.dir/requires: CMakeFiles/marker_AR.dir/MarkerDetector.cpp.o.requires
CMakeFiles/marker_AR.dir/requires: CMakeFiles/marker_AR.dir/GeometryTypes.cpp.o.requires
CMakeFiles/marker_AR.dir/requires: CMakeFiles/marker_AR.dir/Marker.cpp.o.requires

.PHONY : CMakeFiles/marker_AR.dir/requires

CMakeFiles/marker_AR.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/marker_AR.dir/cmake_clean.cmake
.PHONY : CMakeFiles/marker_AR.dir/clean

CMakeFiles/marker_AR.dir/depend:
	cd /home/maverickp/zhongkongbei/artag-try/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maverickp/zhongkongbei/artag-try /home/maverickp/zhongkongbei/artag-try /home/maverickp/zhongkongbei/artag-try/build /home/maverickp/zhongkongbei/artag-try/build /home/maverickp/zhongkongbei/artag-try/build/CMakeFiles/marker_AR.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/marker_AR.dir/depend

