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
include test/CMakeFiles/rawTests.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/rawTests.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/rawTests.dir/flags.make

test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o: test/CMakeFiles/rawTests.dir/flags.make
test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o: ../test/test_rawdata.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rawTests.dir/test_rawdata.cpp.o -c /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_rawdata.cpp

test/CMakeFiles/rawTests.dir/test_rawdata.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rawTests.dir/test_rawdata.cpp.i"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_rawdata.cpp > CMakeFiles/rawTests.dir/test_rawdata.cpp.i

test/CMakeFiles/rawTests.dir/test_rawdata.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rawTests.dir/test_rawdata.cpp.s"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test/test_rawdata.cpp -o CMakeFiles/rawTests.dir/test_rawdata.cpp.s

test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o.requires:

.PHONY : test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o.requires

test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o.provides: test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/rawTests.dir/build.make test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o.provides.build
.PHONY : test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o.provides

test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o.provides.build: test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o


# Object files for target rawTests
rawTests_OBJECTS = \
"CMakeFiles/rawTests.dir/test_rawdata.cpp.o"

# External object files for target rawTests
rawTests_EXTERNAL_OBJECTS =

test/rawTests: test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o
test/rawTests: test/CMakeFiles/rawTests.dir/build.make
test/rawTests: /usr/src/gtest/libgtest.a
test/rawTests: test/libstudent_lib.a
test/rawTests: test/CMakeFiles/rawTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rawTests"
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rawTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/rawTests.dir/build: test/rawTests

.PHONY : test/CMakeFiles/rawTests.dir/build

test/CMakeFiles/rawTests.dir/requires: test/CMakeFiles/rawTests.dir/test_rawdata.cpp.o.requires

.PHONY : test/CMakeFiles/rawTests.dir/requires

test/CMakeFiles/rawTests.dir/clean:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test && $(CMAKE_COMMAND) -P CMakeFiles/rawTests.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/rawTests.dir/clean

test/CMakeFiles/rawTests.dir/depend:
	cd /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test /home/chamath/git/pfms-2021s-dyl1x/scratch/assignments/a1_skeleton/build-Default/test/CMakeFiles/rawTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/rawTests.dir/depend

