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
find_package(Threads REQUIRED)

set(PHYSX_SDK "../../PhysX")

include_directories("${PHYSX_SDK}/pxshared/include")
include_directories("${PHYSX_SDK}/PhysX_3.4/include")

if (MSVC OR APPLE)

	if(MSVC)
		link_directories("${PHYSX_SDK}/physx/bin/win.x86_64.vc142.mt/release")
	else()
		link_directories("${PHYSX_SDK}/PhysX_3.4/Lib/osx64/")
        link_directories("${PHYSX_SDK}/pxshared/Lib/osx64/")
	endif(MSVC)

	link_libraries(

			
            libLowLevelAABBCHECKED.a
            libSimulationControllerCHECKED.a
            libSceneQueryCHECKED.a
            libPhysX3VehicleCHECKED.a
            libPhysX3ExtensionsCHECKED.a
            libPhysX3CookingCHECKED.a
            libPhysX3CommonCHECKED.a
            libPhysX3CHECKED.a
            libPhysX3CharacterKinematicCHECKED.a
            libLowLevelParticlesCHECKED.a
            libLowLevelDynamicsCHECKED.a
            libLowLevelClothCHECKED.a
            libLowLevelCHECKED.a
            
            libPxTaskCHECKED.a
            libPxPvdSDKCHECKED.a
            libPxFoundationCHECKED.a
            libPsFastXmlCHECKED.a

			Threads::Threads)
endif (MSVC OR APPLE)

add_library (PhysXSharpNative SHARED "PhysXSharpNative.cpp" "PhysXSharpNative.h")

if (LINUX)
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
endif(LINUX)

if (MACOS)
	target_link_libraries(
            PhysXSharpNative
            
            libLowLevelAABBCHECKED.a
            libSimulationControllerCHECKED.a
            libSceneQueryCHECKED.a
            libPhysX3VehicleCHECKED.a
            libPhysX3ExtensionsCHECKED.a
            libPhysX3CookingCHECKED.a
            libPhysX3CommonCHECKED.a
            libPhysX3CHECKED.a
            libPhysX3CharacterKinematicCHECKED.a
            libLowLevelParticlesCHECKED.a
            libLowLevelDynamicsCHECKED.a
            libLowLevelClothCHECKED.a
            libLowLevelCHECKED.a
            
            libPxTaskCHECKED.a
            libPxPvdSDKCHECKED.a
            libPxFoundationCHECKED.a
            libPsFastXmlCHECKED.a
            
			Threads::Threads
			${CMAKE_DL_LIBS})
endif(MACOS)
