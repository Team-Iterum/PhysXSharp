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
	set(WIN TRUE)
	message(STATUS ">>> Windows")
endif (MSVC)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)

set(PHYSX_SDK "../../PhysX")

include_directories("${PHYSX_SDK}/PxShared/include")
include_directories("${PHYSX_SDK}/PhysX_3.4/Include")

if (MACOS)
    link_directories("../../Prebuilt/physx3.4_osx64/libChecked/")
endif (MACOS)

if (WIN)
    link_directories("../../Prebuilt/physx3.4_win64/")

    link_libraries(

		PhysX3CharacterKinematicPROFILE_x64.dll
		PhysX3CommonPROFILE_x64.dll
		PhysX3CookingPROFILE_x64.dll
		PhysX3PROFILE_x64.dll


		PxFoundationPROFILE_x64.lib
		PxPvdSDKPROFILE_x64.lib

		LowLevelPROFILE.lib
		LowLevelAABBPROFILE.lib
		LowLevelClothPROFILE.lib
		LowLevelDynamicsPROFILE.lib
		LowLevelParticlesPROFILE.lib
		PhysX3ExtensionsPROFILE.lib
		PhysX3VehiclePROFILE.lib
		SceneQueryPROFILE.lib
		SimulationControllerPROFILE.lib

		PsFastXmlPROFILE_x64.lib
		PxTaskPROFILE_x64.lib
	)

endif (WIN)

if (LINUX)
	link_directories("../../Prebuilt/physx3.4_linux64/release/lib/")
	link_directories("../../Prebuilt/physx3.4_linux64/release/bin/")

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
	    
	    libPhysX3CharacterKinematic.a
		libPhysX3Common.a
		libPhysX3Cooking.a
		libPhysX3.a

		libPxFoundation.a
		libPxPvdSDK.a

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