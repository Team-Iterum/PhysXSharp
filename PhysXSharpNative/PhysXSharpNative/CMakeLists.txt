# CMakeList.txt : CMake project for PhysXSharpNative, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

if (UNIX)
	message(STATUS ">>> Linux")
endif(UNIX)

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
		string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
	endforeach()
endif (MSVC)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(PHYSX_SDK "../../PhysX")
if (UNIX)
	find_package(dlib REQUIRED)
endif(UNIX)

include_directories("${PHYSX_SDK}/pxshared/include")
include_directories("${PHYSX_SDK}/physx/include")


if (MSVC)
	link_directories("${PHYSX_SDK}/physx/bin/win.x86_64.vc142.mt/checked")
	link_libraries(
			PhysXCommon_64
			PhysXCooking_64
			PhysXFoundation_64
			PhysX_64

			PhysXCharacterKinematic_static_64
			PhysXExtensions_static_64
			PhysXPvdSDK_static_64
			PhysXVehicle_static_64
			Threads::Threads)
endif (MSVC)

add_library (PhysXSharpNative SHARED "PhysXSharpNative.cpp" "PhysXSharpNative.h")

if (UNIX)
	target_link_libraries(PhysXSharpNative
											libPhysXCommon_64.so
											libPhysXCooking_64.so
											libPhysXFoundation_64.so
											libPhysX_64.so

											PhysXCharacterKinematic_static_64
											PhysXExtensions_static_64
											PhysXPvdSDK_static_64
											PhysXVehicle_static_64
											Threads::Threads
											${CMAKE_DL_LIBS})
endif(UNIX)
