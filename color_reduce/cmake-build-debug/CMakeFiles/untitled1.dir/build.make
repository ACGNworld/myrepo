# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/fjj/clion/clion-2021.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/fjj/clion/clion-2021.1.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/fjj/CLionProjects/untitled1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fjj/CLionProjects/untitled1/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/untitled1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/untitled1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/untitled1.dir/flags.make

CMakeFiles/untitled1.dir/main.cpp.o: CMakeFiles/untitled1.dir/flags.make
CMakeFiles/untitled1.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fjj/CLionProjects/untitled1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/untitled1.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/untitled1.dir/main.cpp.o -c /home/fjj/CLionProjects/untitled1/main.cpp

CMakeFiles/untitled1.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/untitled1.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fjj/CLionProjects/untitled1/main.cpp > CMakeFiles/untitled1.dir/main.cpp.i

CMakeFiles/untitled1.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/untitled1.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fjj/CLionProjects/untitled1/main.cpp -o CMakeFiles/untitled1.dir/main.cpp.s

# Object files for target untitled1
untitled1_OBJECTS = \
"CMakeFiles/untitled1.dir/main.cpp.o"

# External object files for target untitled1
untitled1_EXTERNAL_OBJECTS =

untitled1: CMakeFiles/untitled1.dir/main.cpp.o
untitled1: CMakeFiles/untitled1.dir/build.make
untitled1: /usr/local/lib/libopencv_stitching.so.3.4.15
untitled1: /usr/local/lib/libopencv_superres.so.3.4.15
untitled1: /usr/local/lib/libopencv_videostab.so.3.4.15
untitled1: /usr/local/lib/libopencv_aruco.so.3.4.15
untitled1: /usr/local/lib/libopencv_bgsegm.so.3.4.15
untitled1: /usr/local/lib/libopencv_bioinspired.so.3.4.15
untitled1: /usr/local/lib/libopencv_ccalib.so.3.4.15
untitled1: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.15
untitled1: /usr/local/lib/libopencv_dpm.so.3.4.15
untitled1: /usr/local/lib/libopencv_face.so.3.4.15
untitled1: /usr/local/lib/libopencv_freetype.so.3.4.15
untitled1: /usr/local/lib/libopencv_fuzzy.so.3.4.15
untitled1: /usr/local/lib/libopencv_hfs.so.3.4.15
untitled1: /usr/local/lib/libopencv_img_hash.so.3.4.15
untitled1: /usr/local/lib/libopencv_line_descriptor.so.3.4.15
untitled1: /usr/local/lib/libopencv_optflow.so.3.4.15
untitled1: /usr/local/lib/libopencv_reg.so.3.4.15
untitled1: /usr/local/lib/libopencv_rgbd.so.3.4.15
untitled1: /usr/local/lib/libopencv_saliency.so.3.4.15
untitled1: /usr/local/lib/libopencv_stereo.so.3.4.15
untitled1: /usr/local/lib/libopencv_structured_light.so.3.4.15
untitled1: /usr/local/lib/libopencv_surface_matching.so.3.4.15
untitled1: /usr/local/lib/libopencv_tracking.so.3.4.15
untitled1: /usr/local/lib/libopencv_xfeatures2d.so.3.4.15
untitled1: /usr/local/lib/libopencv_ximgproc.so.3.4.15
untitled1: /usr/local/lib/libopencv_xobjdetect.so.3.4.15
untitled1: /usr/local/lib/libopencv_xphoto.so.3.4.15
untitled1: /usr/local/lib/libopencv_shape.so.3.4.15
untitled1: /usr/local/lib/libopencv_highgui.so.3.4.15
untitled1: /usr/local/lib/libopencv_videoio.so.3.4.15
untitled1: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.15
untitled1: /usr/local/lib/libopencv_video.so.3.4.15
untitled1: /usr/local/lib/libopencv_datasets.so.3.4.15
untitled1: /usr/local/lib/libopencv_plot.so.3.4.15
untitled1: /usr/local/lib/libopencv_text.so.3.4.15
untitled1: /usr/local/lib/libopencv_dnn.so.3.4.15
untitled1: /usr/local/lib/libopencv_ml.so.3.4.15
untitled1: /usr/local/lib/libopencv_imgcodecs.so.3.4.15
untitled1: /usr/local/lib/libopencv_objdetect.so.3.4.15
untitled1: /usr/local/lib/libopencv_calib3d.so.3.4.15
untitled1: /usr/local/lib/libopencv_features2d.so.3.4.15
untitled1: /usr/local/lib/libopencv_flann.so.3.4.15
untitled1: /usr/local/lib/libopencv_photo.so.3.4.15
untitled1: /usr/local/lib/libopencv_imgproc.so.3.4.15
untitled1: /usr/local/lib/libopencv_core.so.3.4.15
untitled1: CMakeFiles/untitled1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fjj/CLionProjects/untitled1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable untitled1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/untitled1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/untitled1.dir/build: untitled1

.PHONY : CMakeFiles/untitled1.dir/build

CMakeFiles/untitled1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/untitled1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/untitled1.dir/clean

CMakeFiles/untitled1.dir/depend:
	cd /home/fjj/CLionProjects/untitled1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fjj/CLionProjects/untitled1 /home/fjj/CLionProjects/untitled1 /home/fjj/CLionProjects/untitled1/cmake-build-debug /home/fjj/CLionProjects/untitled1/cmake-build-debug /home/fjj/CLionProjects/untitled1/cmake-build-debug/CMakeFiles/untitled1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/untitled1.dir/depend

