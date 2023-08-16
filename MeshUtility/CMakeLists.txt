# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)
set (CMAKE_CXX_STANDARD 17)

project ("MeshUtility")

if (UNIX AND NOT APPLE)
	set(LINUX TRUE)
	message(STATUS ">>> Linux")
endif(UNIX AND NOT APPLE)

if (APPLE)
	set(MACOS TRUE)
	message(STATUS ">>> Apple")
endif(APPLE)

if (MSVC)
	message(STATUS ">>> Windows")
	set(CompilerFlags
			CMAKE_CXX_FLAGS
			CMAKE_CXX_FLAGS_DEBUG
			CMAKE_CXX_FLAGS_RELEASE
			CMAKE_C_FLAGS
			CMAKE_C_FLAGS_DEBUG
			CMAKE_C_FLAGS_RELEASE
			)
	foreach(CompilerFlag ${CompilerFlags})
		string(REPLACE "/MDd" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
	endforeach()
endif (MSVC)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(SHARP_NATIVE_SDK "../PhysXSharpNative/")
set(PHYSX_SDK "../PhysX")

include_directories("${PHYSX_SDK}/pxshared/include")
include_directories("${PHYSX_SDK}/physx/include")

include_directories("${SHARP_NATIVE_SDK}/PhysXSharpNative")

if (MSVC OR APPLE)

	if(MSVC)
		link_directories("${SHARP_NATIVE_SDK}/PhysXSharpNative/")
	else()
		link_directories("${SHARP_NATIVE_SDK}/PhysXSharpNative/")
	endif(MSVC)

	link_libraries(PhysXSharpNative)
endif (MSVC OR APPLE)

add_executable (MeshUtility "main.cpp" "main.h")

if (LINUX)
	target_link_libraries(MeshUtility libPhysXSharpNative.so)
endif(LINUX)

if (MACOS)
	target_link_libraries(MeshUtility libPhysXSharpNative.dylib)
endif(MACOS)
