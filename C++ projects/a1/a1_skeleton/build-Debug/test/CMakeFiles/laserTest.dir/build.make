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
CMAKE_SOURCE_DIR = /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug

# Include any dependencies generated for this target.
include test/CMakeFiles/laserTest.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/laserTest.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/laserTest.dir/flags.make

test/CMakeFiles/laserTest.dir/test_sonar.cpp.o: test/CMakeFiles/laserTest.dir/flags.make
test/CMakeFiles/laserTest.dir/test_sonar.cpp.o: ../test/test_sonar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/laserTest.dir/test_sonar.cpp.o"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/test && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/laserTest.dir/test_sonar.cpp.o -c /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_sonar.cpp

test/CMakeFiles/laserTest.dir/test_sonar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laserTest.dir/test_sonar.cpp.i"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_sonar.cpp > CMakeFiles/laserTest.dir/test_sonar.cpp.i

test/CMakeFiles/laserTest.dir/test_sonar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laserTest.dir/test_sonar.cpp.s"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_sonar.cpp -o CMakeFiles/laserTest.dir/test_sonar.cpp.s

test/CMakeFiles/laserTest.dir/test_sonar.cpp.o.requires:

.PHONY : test/CMakeFiles/laserTest.dir/test_sonar.cpp.o.requires

test/CMakeFiles/laserTest.dir/test_sonar.cpp.o.provides: test/CMakeFiles/laserTest.dir/test_sonar.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/laserTest.dir/build.make test/CMakeFiles/laserTest.dir/test_sonar.cpp.o.provides.build
.PHONY : test/CMakeFiles/laserTest.dir/test_sonar.cpp.o.provides

test/CMakeFiles/laserTest.dir/test_sonar.cpp.o.provides.build: test/CMakeFiles/laserTest.dir/test_sonar.cpp.o


# Object files for target laserTest
laserTest_OBJECTS = \
"CMakeFiles/laserTest.dir/test_sonar.cpp.o"

# External object files for target laserTest
laserTest_EXTERNAL_OBJECTS =

test/laserTest: test/CMakeFiles/laserTest.dir/test_sonar.cpp.o
test/laserTest: test/CMakeFiles/laserTest.dir/build.make
test/laserTest: /usr/src/gtest/libgtest.a
test/laserTest: test/libstudent_lib.a
test/laserTest: test/CMakeFiles/laserTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable laserTest"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/laserTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/laserTest.dir/build: test/laserTest

.PHONY : test/CMakeFiles/laserTest.dir/build

test/CMakeFiles/laserTest.dir/requires: test/CMakeFiles/laserTest.dir/test_sonar.cpp.o.requires

.PHONY : test/CMakeFiles/laserTest.dir/requires

test/CMakeFiles/laserTest.dir/clean:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/test && $(CMAKE_COMMAND) -P CMakeFiles/laserTest.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/laserTest.dir/clean

test/CMakeFiles/laserTest.dir/depend:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Debug/test/CMakeFiles/laserTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/laserTest.dir/depend

