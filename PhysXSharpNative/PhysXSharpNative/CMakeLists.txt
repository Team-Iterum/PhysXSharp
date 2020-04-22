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

if (MACOS)
	link_directories("${PHYSX_SDK}/PhysX_3.4/Lib/osx64/")
    link_directories("${PHYSX_SDK}/pxshared/Lib/osx64/")
endif (MACOS)

if(LINUX)
	link_directories("${PHYSX_SDK}/PhysX_3.4/Lib/linux64/")
    link_directories("${PHYSX_SDK}/pxshared/Lib/linux64/")
endif (LINUX)

link_libraries(		
	LowLevelAABBCHECKED
	SimulationControllerCHECKED
	SceneQueryCHECKED
	PhysX3VehicleCHECKED
	PhysX3ExtensionsCHECKED
	PhysX3CookingCHECKED
	PhysX3CommonCHECKED
	PhysX3CHECKED
	PhysX3CharacterKinematicCHECKED
	LowLevelParticlesCHECKED
	LowLevelDynamicsCHECKED
	LowLevelClothCHECKED
	LowLevelCHECKED

	PxTaskCHECKED
	PxPvdSDKCHECKED
	PxFoundationCHECKED
	PsFastXmlCHECKED

	Threads::Threads
)

add_library (PhysXSharpNative SHARED "PhysXSharpNative.cpp" "PhysXSharpNative.h")


target_link_libraries(
    PhysXSharpNative
    
    LowLevelAABBCHECKED
    SimulationControllerCHECKED
    SceneQueryCHECKED
    PhysX3VehicleCHECKED
    PhysX3ExtensionsCHECKED
    PhysX3CookingCHECKED
    PhysX3CommonCHECKED
    PhysX3CHECKED
    PhysX3CharacterKinematicCHECKED
    LowLevelParticlesCHECKED
    LowLevelDynamicsCHECKED
    LowLevelClothCHECKED
    LowLevelCHECKED
    
    PxTaskCHECKED
    PxPvdSDKCHECKED
    PxFoundationCHECKED
    PsFastXmlCHECKED
    
	Threads::Threads

	${CMAKE_DL_LIBS}
)
