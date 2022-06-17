# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/tpp/Downloads/trunk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tpp/Downloads/trunk/cmake

# Include any dependencies generated for this target.
include utils/CMakeFiles/reslam_monocular.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/reslam_monocular.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/reslam_monocular.dir/flags.make

utils/CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.o: utils/CMakeFiles/reslam_monocular.dir/flags.make
utils/CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.o: ../utils/monocular_slam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tpp/Downloads/trunk/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utils/CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.o"
	cd /home/tpp/Downloads/trunk/cmake/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.o -c /home/tpp/Downloads/trunk/utils/monocular_slam.cpp

utils/CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.i"
	cd /home/tpp/Downloads/trunk/cmake/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tpp/Downloads/trunk/utils/monocular_slam.cpp > CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.i

utils/CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.s"
	cd /home/tpp/Downloads/trunk/cmake/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tpp/Downloads/trunk/utils/monocular_slam.cpp -o CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.s

utils/CMakeFiles/reslam_monocular.dir/inputreader.cpp.o: utils/CMakeFiles/reslam_monocular.dir/flags.make
utils/CMakeFiles/reslam_monocular.dir/inputreader.cpp.o: ../utils/inputreader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tpp/Downloads/trunk/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object utils/CMakeFiles/reslam_monocular.dir/inputreader.cpp.o"
	cd /home/tpp/Downloads/trunk/cmake/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/reslam_monocular.dir/inputreader.cpp.o -c /home/tpp/Downloads/trunk/utils/inputreader.cpp

utils/CMakeFiles/reslam_monocular.dir/inputreader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/reslam_monocular.dir/inputreader.cpp.i"
	cd /home/tpp/Downloads/trunk/cmake/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tpp/Downloads/trunk/utils/inputreader.cpp > CMakeFiles/reslam_monocular.dir/inputreader.cpp.i

utils/CMakeFiles/reslam_monocular.dir/inputreader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/reslam_monocular.dir/inputreader.cpp.s"
	cd /home/tpp/Downloads/trunk/cmake/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tpp/Downloads/trunk/utils/inputreader.cpp -o CMakeFiles/reslam_monocular.dir/inputreader.cpp.s

# Object files for target reslam_monocular
reslam_monocular_OBJECTS = \
"CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.o" \
"CMakeFiles/reslam_monocular.dir/inputreader.cpp.o"

# External object files for target reslam_monocular
reslam_monocular_EXTERNAL_OBJECTS =

utils/reslam_monocular: utils/CMakeFiles/reslam_monocular.dir/monocular_slam.cpp.o
utils/reslam_monocular: utils/CMakeFiles/reslam_monocular.dir/inputreader.cpp.o
utils/reslam_monocular: utils/CMakeFiles/reslam_monocular.dir/build.make
utils/reslam_monocular: libs/libreslam.so.1.2.4
utils/reslam_monocular: libs/libreslam_fbow.so.1.2.4
utils/reslam_monocular: /usr/lib/gcc/x86_64-linux-gnu/9/libgomp.so
utils/reslam_monocular: /usr/lib/x86_64-linux-gnu/libpthread.so
utils/reslam_monocular: libs/libreslam_aruco.so.1.2.4
utils/reslam_monocular: libs/libreslam_xflann.so.1.2.4
utils/reslam_monocular: /usr/local/lib/libopencv_gapi.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_highgui.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_ml.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_objdetect.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_photo.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_stitching.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_video.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_calib3d.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_dnn.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_features2d.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_flann.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_videoio.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_imgcodecs.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_imgproc.so.4.5.5
utils/reslam_monocular: /usr/local/lib/libopencv_core.so.4.5.5
utils/reslam_monocular: libs/libreslam_g2o_core.so.1.2.4
utils/reslam_monocular: libs/libreslam_g2o_stuff.so.1.2.4
utils/reslam_monocular: utils/CMakeFiles/reslam_monocular.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tpp/Downloads/trunk/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable reslam_monocular"
	cd /home/tpp/Downloads/trunk/cmake/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/reslam_monocular.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/reslam_monocular.dir/build: utils/reslam_monocular

.PHONY : utils/CMakeFiles/reslam_monocular.dir/build

utils/CMakeFiles/reslam_monocular.dir/clean:
	cd /home/tpp/Downloads/trunk/cmake/utils && $(CMAKE_COMMAND) -P CMakeFiles/reslam_monocular.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/reslam_monocular.dir/clean

utils/CMakeFiles/reslam_monocular.dir/depend:
	cd /home/tpp/Downloads/trunk/cmake && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tpp/Downloads/trunk /home/tpp/Downloads/trunk/utils /home/tpp/Downloads/trunk/cmake /home/tpp/Downloads/trunk/cmake/utils /home/tpp/Downloads/trunk/cmake/utils/CMakeFiles/reslam_monocular.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/reslam_monocular.dir/depend

