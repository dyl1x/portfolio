# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Change project name
project (a2_skeleton) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will create your library, add the cpp files needed
add_library(ranger STATIC ranger.cpp laser.cpp sonar.cpp)

# CMake will create the cell library that you will need 
add_library(cell STATIC cell.cpp)

# CMake will create the cell library that you will need
add_library(rangerfusion STATIC rangerfusion.cpp)

# CMake will create the cell library that you will need
add_library(geometry STATIC line.cpp processing.cpp circle.cpp)


# Replace <header_files> with the header files that form part of library and need to be distributed, seperated by ";"
#set_target_properties(shapes  PROPERTIES PUBLIC_HEADER "")
#set_target_properties(shapes  PROPERTIES PUBLIC_HEADER "shape.h;triangle.h;rectangle.h;circle.h;line.h")


# CMake will create executable add the cpp files needed
add_executable(${PROJECT_NAME}_ex main.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(rangerfusion geometry)

target_link_libraries(${PROJECT_NAME}_ex ranger cell rangerfusion)

# cmake .. -DBUILD_TESTS=ON (or comment out the if and endif statements to see inside qtcreator)
#if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
#endif()
