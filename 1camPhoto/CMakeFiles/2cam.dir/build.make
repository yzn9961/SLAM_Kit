# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/telemoro/桌面/SLAM/Code/2camPhoto

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/telemoro/桌面/SLAM/Code/2camPhoto

# Include any dependencies generated for this target.
include CMakeFiles/2cam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/2cam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/2cam.dir/flags.make

CMakeFiles/2cam.dir/main.cpp.o: CMakeFiles/2cam.dir/flags.make
CMakeFiles/2cam.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/telemoro/桌面/SLAM/Code/2camPhoto/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/2cam.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/2cam.dir/main.cpp.o -c /home/telemoro/桌面/SLAM/Code/2camPhoto/main.cpp

CMakeFiles/2cam.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/2cam.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/telemoro/桌面/SLAM/Code/2camPhoto/main.cpp > CMakeFiles/2cam.dir/main.cpp.i

CMakeFiles/2cam.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/2cam.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/telemoro/桌面/SLAM/Code/2camPhoto/main.cpp -o CMakeFiles/2cam.dir/main.cpp.s

CMakeFiles/2cam.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/2cam.dir/main.cpp.o.requires

CMakeFiles/2cam.dir/main.cpp.o.provides: CMakeFiles/2cam.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/2cam.dir/build.make CMakeFiles/2cam.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/2cam.dir/main.cpp.o.provides

CMakeFiles/2cam.dir/main.cpp.o.provides.build: CMakeFiles/2cam.dir/main.cpp.o


# Object files for target 2cam
2cam_OBJECTS = \
"CMakeFiles/2cam.dir/main.cpp.o"

# External object files for target 2cam
2cam_EXTERNAL_OBJECTS =

2cam: CMakeFiles/2cam.dir/main.cpp.o
2cam: CMakeFiles/2cam.dir/build.make
2cam: /usr/local/lib/libopencv_dnn.so.4.4.0
2cam: /usr/local/lib/libopencv_gapi.so.4.4.0
2cam: /usr/local/lib/libopencv_highgui.so.4.4.0
2cam: /usr/local/lib/libopencv_ml.so.4.4.0
2cam: /usr/local/lib/libopencv_objdetect.so.4.4.0
2cam: /usr/local/lib/libopencv_photo.so.4.4.0
2cam: /usr/local/lib/libopencv_stitching.so.4.4.0
2cam: /usr/local/lib/libopencv_video.so.4.4.0
2cam: /usr/local/lib/libopencv_videoio.so.4.4.0
2cam: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
2cam: /usr/local/lib/libopencv_calib3d.so.4.4.0
2cam: /usr/local/lib/libopencv_features2d.so.4.4.0
2cam: /usr/local/lib/libopencv_flann.so.4.4.0
2cam: /usr/local/lib/libopencv_imgproc.so.4.4.0
2cam: /usr/local/lib/libopencv_core.so.4.4.0
2cam: CMakeFiles/2cam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/telemoro/桌面/SLAM/Code/2camPhoto/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 2cam"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/2cam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/2cam.dir/build: 2cam

.PHONY : CMakeFiles/2cam.dir/build

CMakeFiles/2cam.dir/requires: CMakeFiles/2cam.dir/main.cpp.o.requires

.PHONY : CMakeFiles/2cam.dir/requires

CMakeFiles/2cam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/2cam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/2cam.dir/clean

CMakeFiles/2cam.dir/depend:
	cd /home/telemoro/桌面/SLAM/Code/2camPhoto && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/telemoro/桌面/SLAM/Code/2camPhoto /home/telemoro/桌面/SLAM/Code/2camPhoto /home/telemoro/桌面/SLAM/Code/2camPhoto /home/telemoro/桌面/SLAM/Code/2camPhoto /home/telemoro/桌面/SLAM/Code/2camPhoto/CMakeFiles/2cam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/2cam.dir/depend

