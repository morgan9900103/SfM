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
CMAKE_SOURCE_DIR = /home/morganlee/projects/SfM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/morganlee/projects/SfM/build

# Include any dependencies generated for this target.
include CMakeFiles/run_sfm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/run_sfm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run_sfm.dir/flags.make

CMakeFiles/run_sfm.dir/run_sfm.cpp.o: CMakeFiles/run_sfm.dir/flags.make
CMakeFiles/run_sfm.dir/run_sfm.cpp.o: ../run_sfm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/morganlee/projects/SfM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run_sfm.dir/run_sfm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_sfm.dir/run_sfm.cpp.o -c /home/morganlee/projects/SfM/run_sfm.cpp

CMakeFiles/run_sfm.dir/run_sfm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_sfm.dir/run_sfm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/morganlee/projects/SfM/run_sfm.cpp > CMakeFiles/run_sfm.dir/run_sfm.cpp.i

CMakeFiles/run_sfm.dir/run_sfm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_sfm.dir/run_sfm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/morganlee/projects/SfM/run_sfm.cpp -o CMakeFiles/run_sfm.dir/run_sfm.cpp.s

CMakeFiles/run_sfm.dir/run_sfm.cpp.o.requires:

.PHONY : CMakeFiles/run_sfm.dir/run_sfm.cpp.o.requires

CMakeFiles/run_sfm.dir/run_sfm.cpp.o.provides: CMakeFiles/run_sfm.dir/run_sfm.cpp.o.requires
	$(MAKE) -f CMakeFiles/run_sfm.dir/build.make CMakeFiles/run_sfm.dir/run_sfm.cpp.o.provides.build
.PHONY : CMakeFiles/run_sfm.dir/run_sfm.cpp.o.provides

CMakeFiles/run_sfm.dir/run_sfm.cpp.o.provides.build: CMakeFiles/run_sfm.dir/run_sfm.cpp.o


# Object files for target run_sfm
run_sfm_OBJECTS = \
"CMakeFiles/run_sfm.dir/run_sfm.cpp.o"

# External object files for target run_sfm
run_sfm_EXTERNAL_OBJECTS =

run_sfm: CMakeFiles/run_sfm.dir/run_sfm.cpp.o
run_sfm: CMakeFiles/run_sfm.dir/build.make
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
run_sfm: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
run_sfm: CMakeFiles/run_sfm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/morganlee/projects/SfM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable run_sfm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_sfm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run_sfm.dir/build: run_sfm

.PHONY : CMakeFiles/run_sfm.dir/build

CMakeFiles/run_sfm.dir/requires: CMakeFiles/run_sfm.dir/run_sfm.cpp.o.requires

.PHONY : CMakeFiles/run_sfm.dir/requires

CMakeFiles/run_sfm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_sfm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_sfm.dir/clean

CMakeFiles/run_sfm.dir/depend:
	cd /home/morganlee/projects/SfM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/morganlee/projects/SfM /home/morganlee/projects/SfM /home/morganlee/projects/SfM/build /home/morganlee/projects/SfM/build /home/morganlee/projects/SfM/build/CMakeFiles/run_sfm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_sfm.dir/depend

