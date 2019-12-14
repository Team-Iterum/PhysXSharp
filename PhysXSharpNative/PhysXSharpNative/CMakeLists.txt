# CMakeList.txt : CMake project for PhysXSharpNative, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

include_directories("../../PhysX/pxshared/include")
include_directories("../../PhysX/physx/include")


# Add source to this project's executable.
add_executable (PhysXSharpNative "PhysXSharpNative.cpp" "PhysXSharpNative.h")

# TODO: Add tests and install targets if needed.
