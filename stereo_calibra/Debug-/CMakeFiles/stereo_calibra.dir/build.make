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
CMAKE_SOURCE_DIR = /home/telemoro/桌面/SLAM/Code/stereo_calibra

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/telemoro/桌面/SLAM/Code/stereo_calibra/Debug-

# Include any dependencies generated for this target.
include CMakeFiles/stereo_calibra.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_calibra.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_calibra.dir/flags.make

CMakeFiles/stereo_calibra.dir/main.cpp.o: CMakeFiles/stereo_calibra.dir/flags.make
CMakeFiles/stereo_calibra.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/telemoro/桌面/SLAM/Code/stereo_calibra/Debug-/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_calibra.dir/main.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_calibra.dir/main.cpp.o -c /home/telemoro/桌面/SLAM/Code/stereo_calibra/main.cpp

CMakeFiles/stereo_calibra.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_calibra.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/telemoro/桌面/SLAM/Code/stereo_calibra/main.cpp > CMakeFiles/stereo_calibra.dir/main.cpp.i

CMakeFiles/stereo_calibra.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_calibra.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/telemoro/桌面/SLAM/Code/stereo_calibra/main.cpp -o CMakeFiles/stereo_calibra.dir/main.cpp.s

CMakeFiles/stereo_calibra.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/stereo_calibra.dir/main.cpp.o.requires

CMakeFiles/stereo_calibra.dir/main.cpp.o.provides: CMakeFiles/stereo_calibra.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_calibra.dir/build.make CMakeFiles/stereo_calibra.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_calibra.dir/main.cpp.o.provides

CMakeFiles/stereo_calibra.dir/main.cpp.o.provides.build: CMakeFiles/stereo_calibra.dir/main.cpp.o


# Object files for target stereo_calibra
stereo_calibra_OBJECTS = \
"CMakeFiles/stereo_calibra.dir/main.cpp.o"

# External object files for target stereo_calibra
stereo_calibra_EXTERNAL_OBJECTS =

stereo_calibra: CMakeFiles/stereo_calibra.dir/main.cpp.o
stereo_calibra: CMakeFiles/stereo_calibra.dir/build.make
stereo_calibra: /usr/local/lib/libopencv_dnn.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_gapi.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_highgui.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_ml.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_objdetect.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_photo.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_stitching.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_video.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_videoio.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_calib3d.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_features2d.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_flann.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_imgproc.so.4.4.0
stereo_calibra: /usr/local/lib/libopencv_core.so.4.4.0
stereo_calibra: CMakeFiles/stereo_calibra.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/telemoro/桌面/SLAM/Code/stereo_calibra/Debug-/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stereo_calibra"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_calibra.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_calibra.dir/build: stereo_calibra

.PHONY : CMakeFiles/stereo_calibra.dir/build

CMakeFiles/stereo_calibra.dir/requires: CMakeFiles/stereo_calibra.dir/main.cpp.o.requires

.PHONY : CMakeFiles/stereo_calibra.dir/requires

CMakeFiles/stereo_calibra.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_calibra.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_calibra.dir/clean

CMakeFiles/stereo_calibra.dir/depend:
	cd /home/telemoro/桌面/SLAM/Code/stereo_calibra/Debug- && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/telemoro/桌面/SLAM/Code/stereo_calibra /home/telemoro/桌面/SLAM/Code/stereo_calibra /home/telemoro/桌面/SLAM/Code/stereo_calibra/Debug- /home/telemoro/桌面/SLAM/Code/stereo_calibra/Debug- /home/telemoro/桌面/SLAM/Code/stereo_calibra/Debug-/CMakeFiles/stereo_calibra.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_calibra.dir/depend

