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
CMAKE_BINARY_DIR = /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default

# Include any dependencies generated for this target.
include test/CMakeFiles/laserTests.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/laserTests.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/laserTests.dir/flags.make

test/CMakeFiles/laserTests.dir/test_laser.cpp.o: test/CMakeFiles/laserTests.dir/flags.make
test/CMakeFiles/laserTests.dir/test_laser.cpp.o: ../test/test_laser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/laserTests.dir/test_laser.cpp.o"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/laserTests.dir/test_laser.cpp.o -c /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_laser.cpp

test/CMakeFiles/laserTests.dir/test_laser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laserTests.dir/test_laser.cpp.i"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_laser.cpp > CMakeFiles/laserTests.dir/test_laser.cpp.i

test/CMakeFiles/laserTests.dir/test_laser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laserTests.dir/test_laser.cpp.s"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_laser.cpp -o CMakeFiles/laserTests.dir/test_laser.cpp.s

test/CMakeFiles/laserTests.dir/test_laser.cpp.o.requires:

.PHONY : test/CMakeFiles/laserTests.dir/test_laser.cpp.o.requires

test/CMakeFiles/laserTests.dir/test_laser.cpp.o.provides: test/CMakeFiles/laserTests.dir/test_laser.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/laserTests.dir/build.make test/CMakeFiles/laserTests.dir/test_laser.cpp.o.provides.build
.PHONY : test/CMakeFiles/laserTests.dir/test_laser.cpp.o.provides

test/CMakeFiles/laserTests.dir/test_laser.cpp.o.provides.build: test/CMakeFiles/laserTests.dir/test_laser.cpp.o


# Object files for target laserTests
laserTests_OBJECTS = \
"CMakeFiles/laserTests.dir/test_laser.cpp.o"

# External object files for target laserTests
laserTests_EXTERNAL_OBJECTS =

test/laserTests: test/CMakeFiles/laserTests.dir/test_laser.cpp.o
test/laserTests: test/CMakeFiles/laserTests.dir/build.make
test/laserTests: /usr/src/gtest/libgtest.a
test/laserTests: test/libmock_lib.a
test/laserTests: test/CMakeFiles/laserTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable laserTests"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/laserTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/laserTests.dir/build: test/laserTests

.PHONY : test/CMakeFiles/laserTests.dir/build

test/CMakeFiles/laserTests.dir/requires: test/CMakeFiles/laserTests.dir/test_laser.cpp.o.requires

.PHONY : test/CMakeFiles/laserTests.dir/requires

test/CMakeFiles/laserTests.dir/clean:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && $(CMAKE_COMMAND) -P CMakeFiles/laserTests.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/laserTests.dir/clean

test/CMakeFiles/laserTests.dir/depend:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test/CMakeFiles/laserTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/laserTests.dir/depend

