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
CMAKE_SOURCE_DIR = /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug

# Include any dependencies generated for this target.
include test/CMakeFiles/associationtest.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/associationtest.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/associationtest.dir/flags.make

test/CMakeFiles/associationtest.dir/associationtest.cpp.o: test/CMakeFiles/associationtest.dir/flags.make
test/CMakeFiles/associationtest.dir/associationtest.cpp.o: ../test/associationtest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/associationtest.dir/associationtest.cpp.o"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/associationtest.dir/associationtest.cpp.o -c /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/test/associationtest.cpp

test/CMakeFiles/associationtest.dir/associationtest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/associationtest.dir/associationtest.cpp.i"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/test/associationtest.cpp > CMakeFiles/associationtest.dir/associationtest.cpp.i

test/CMakeFiles/associationtest.dir/associationtest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/associationtest.dir/associationtest.cpp.s"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/test/associationtest.cpp -o CMakeFiles/associationtest.dir/associationtest.cpp.s

test/CMakeFiles/associationtest.dir/associationtest.cpp.o.requires:

.PHONY : test/CMakeFiles/associationtest.dir/associationtest.cpp.o.requires

test/CMakeFiles/associationtest.dir/associationtest.cpp.o.provides: test/CMakeFiles/associationtest.dir/associationtest.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/associationtest.dir/build.make test/CMakeFiles/associationtest.dir/associationtest.cpp.o.provides.build
.PHONY : test/CMakeFiles/associationtest.dir/associationtest.cpp.o.provides

test/CMakeFiles/associationtest.dir/associationtest.cpp.o.provides.build: test/CMakeFiles/associationtest.dir/associationtest.cpp.o


# Object files for target associationtest
associationtest_OBJECTS = \
"CMakeFiles/associationtest.dir/associationtest.cpp.o"

# External object files for target associationtest
associationtest_EXTERNAL_OBJECTS =

test/associationtest: test/CMakeFiles/associationtest.dir/associationtest.cpp.o
test/associationtest: test/CMakeFiles/associationtest.dir/build.make
test/associationtest: /usr/src/gtest/libgtest.a
test/associationtest: /usr/src/gtest/libgtest_main.a
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
test/associationtest: test/libstudent_lib.a
test/associationtest: /usr/src/gtest/libgtest.a
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
test/associationtest: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
test/associationtest: test/CMakeFiles/associationtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable associationtest"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/associationtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/associationtest.dir/build: test/associationtest

.PHONY : test/CMakeFiles/associationtest.dir/build

test/CMakeFiles/associationtest.dir/requires: test/CMakeFiles/associationtest.dir/associationtest.cpp.o.requires

.PHONY : test/CMakeFiles/associationtest.dir/requires

test/CMakeFiles/associationtest.dir/clean:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && $(CMAKE_COMMAND) -P CMakeFiles/associationtest.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/associationtest.dir/clean

test/CMakeFiles/associationtest.dir/depend:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2 /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test/CMakeFiles/associationtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/associationtest.dir/depend

