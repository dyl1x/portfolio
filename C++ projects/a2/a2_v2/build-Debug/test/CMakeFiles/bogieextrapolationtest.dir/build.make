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
include test/CMakeFiles/bogieextrapolationtest.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/bogieextrapolationtest.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/bogieextrapolationtest.dir/flags.make

test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o: test/CMakeFiles/bogieextrapolationtest.dir/flags.make
test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o: ../test/bogieextrapolationtest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o -c /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/test/bogieextrapolationtest.cpp

test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.i"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/test/bogieextrapolationtest.cpp > CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.i

test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.s"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/test/bogieextrapolationtest.cpp -o CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.s

test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o.requires:

.PHONY : test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o.requires

test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o.provides: test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/bogieextrapolationtest.dir/build.make test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o.provides.build
.PHONY : test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o.provides

test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o.provides.build: test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o


# Object files for target bogieextrapolationtest
bogieextrapolationtest_OBJECTS = \
"CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o"

# External object files for target bogieextrapolationtest
bogieextrapolationtest_EXTERNAL_OBJECTS =

test/bogieextrapolationtest: test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o
test/bogieextrapolationtest: test/CMakeFiles/bogieextrapolationtest.dir/build.make
test/bogieextrapolationtest: /usr/src/gtest/libgtest.a
test/bogieextrapolationtest: /usr/src/gtest/libgtest_main.a
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
test/bogieextrapolationtest: test/libstudent_lib.a
test/bogieextrapolationtest: /usr/src/gtest/libgtest.a
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
test/bogieextrapolationtest: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
test/bogieextrapolationtest: test/CMakeFiles/bogieextrapolationtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bogieextrapolationtest"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bogieextrapolationtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/bogieextrapolationtest.dir/build: test/bogieextrapolationtest

.PHONY : test/CMakeFiles/bogieextrapolationtest.dir/build

test/CMakeFiles/bogieextrapolationtest.dir/requires: test/CMakeFiles/bogieextrapolationtest.dir/bogieextrapolationtest.cpp.o.requires

.PHONY : test/CMakeFiles/bogieextrapolationtest.dir/requires

test/CMakeFiles/bogieextrapolationtest.dir/clean:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test && $(CMAKE_COMMAND) -P CMakeFiles/bogieextrapolationtest.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/bogieextrapolationtest.dir/clean

test/CMakeFiles/bogieextrapolationtest.dir/depend:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2 /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a2/a2_v2/build-Debug/test/CMakeFiles/bogieextrapolationtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/bogieextrapolationtest.dir/depend

