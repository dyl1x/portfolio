# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.8)


find_package( OpenCV REQUIRED )
# Any project name will suffice, this has connotaions when using advanced CMake Features
set(PROJECT_NAME a2-tests)
project (${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Finding and linking against the simulator library
set(SIMULATOR_LIB ${CMAKE_SOURCE_DIR}/dep/lib/)
set(SIMULATOR_INCLUDE ${CMAKE_SOURCE_DIR}/dep/include)

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)
include_directories(
  ${SIMULATOR_INCLUDE}
)
# This uses the complete student's library
aux_source_directory(.. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "../main.cpp")
list(REMOVE_ITEM SRC_LIST "../main_default.cpp")
message ( STATUS "Compiling student_lib with following files  ${SRC_LIST}" )
add_library(student_lib ${SRC_LIST})

#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()
###################################

include_directories(${CMAKE_SOURCE_DIR}) # includes below directory

add_executable(posetest posetest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(posetest GTest::GTest GTest::Main)
target_link_libraries(posetest ${OpenCV_LIBS} student_lib simulator)
# We can finally add the test to a list of tests to be run with make test
add_test(a2test posetest)

add_executable(associationtest associationtest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(associationtest GTest::GTest GTest::Main)
target_link_libraries(associationtest ${OpenCV_LIBS} student_lib simulator)
# We can finally add the test to a list of tests to be run with make test
add_test(a2test associationtest)

add_executable(bogieextrapolationtest bogieextrapolationtest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(bogieextrapolationtest GTest::GTest GTest::Main)
target_link_libraries(bogieextrapolationtest ${OpenCV_LIBS} student_lib simulator)
# We can finally add the test to a list of tests to be run with make test
add_test(a2test bogieextrapolationtest)
