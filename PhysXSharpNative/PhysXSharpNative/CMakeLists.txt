# CMakeList.txt : CMake project for PhysXSharpNative, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

if (UNIX AND NOT APPLE)
	set(LINUX TRUE)
	message(STATUS ">>> Linux")
endif(UNIX AND NOT APPLE)

if (APPLE)
	set(MACOS TRUE)
	message(STATUS ">>> Apple")
endif(APPLE)

if (MSVC)
	message(STATUS ">>> Windows NOT SUPPORTED")
endif (MSVC)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)

set(PHYSX_SDK "../../PhysX")

include_directories("${PHYSX_SDK}/PxShared/include")
include_directories("${PHYSX_SDK}/PhysX_3.4/Include")

if (MACOS)
	link_directories("${PHYSX_SDK}/PhysX_3.4/Lib/osx64/")
    link_directories("${PHYSX_SDK}/PxShared/Lib/osx64/")
endif (MACOS)

if (LINUX)
	link_directories("Prebuilt/physx3.4_linux64/release/lib/")
	link_directories("Prebuilt/physx3.4_linux64/release/bin/")

	link_libraries(

		libPhysX3CharacterKinematic_x64.so
		libPhysX3Common_x64.so
		libPhysX3Cooking_x64.so
		libPhysX3_x64.so	

		libPxFoundation_x64.so
		libPxPvdSDK_x64.so

		libLowLevel.a
		libLowLevelAABB.a
		libLowLevelCloth.a
		libLowLevelDynamics.a
		libLowLevelParticles.a
		libPhysX3Extensions.a
		libPhysX3Vehicle.a
		libSceneQuery.a
		libSimulationController.a

		libPsFastXml.a
		libPxTask.a

		Threads::Threads
	)
endif (LINUX)

add_library (PhysXSharpNative SHARED "PhysXSharpNative.cpp" "PhysXSharpNative.h")


if (LINUX)
	target_link_libraries(PhysXSharpNative
	    
	    libPhysX3CharacterKinematic_x64.so
		libPhysX3Common_x64.so
		libPhysX3Cooking_x64.so
		libPhysX3_x64.so	

		libPxFoundation_x64.so
		libPxPvdSDK_x64.so

		libLowLevel.a
		libLowLevelAABB.a
		libLowLevelCloth.a
		libLowLevelDynamics.a
		libLowLevelParticles.a
		libPhysX3Extensions.a
		libPhysX3Vehicle.a
		libSceneQuery.a
		libSimulationController.a

		libPsFastXml.a
		libPxTask.a
		
		Threads::Threads

		${CMAKE_DL_LIBS}
	)
endif (LINUX)

if (MACOS)
	target_link_libraries(PhysXSharpNative
	    
	    libPhysX3CharacterKinematic_x64.so
		libPhysX3Common_x64.so
		libPhysX3Cooking_x64.so
		libPhysX3_x64.so	

		libPxFoundation_x64.so
		libPxPvdSDK_x64.so

		libLowLevel.a
		libLowLevelAABB.a
		libLowLevelCloth.a
		libLowLevelDynamics.a
		libLowLevelParticles.a
		libPhysX3Extensions.a
		libPhysX3Vehicle.a
		libSceneQuery.a
		libSimulationController.a

		libPsFastXml.a
		libPxTask.a
		
		Threads::Threads

		${CMAKE_DL_LIBS}
	)
endif (MACOS)

