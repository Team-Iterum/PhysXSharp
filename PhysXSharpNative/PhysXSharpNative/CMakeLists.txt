# CMakeList.txt : CMake project for PhysXSharpNative, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
if(${PLATFORM} STREQUAL "LINUX")
	find_package(dlib REQUIRED)
endif()

set(PHYSX_SDK "../../PhysX")

if(${PLATFORM} STREQUAL "WIN")
	set(PHYSX_LIB_DIR "physx/bin/win.x86_64.vc142.mt/release")
else()
	set(PHYSX_LIB_DIR "physx/bin/linux.clang/release")
endif()

include_directories("${PHYSX_SDK}/pxshared/include")
include_directories("${PHYSX_SDK}/physx/include")

link_directories("${PHYSX_SDK}/${PHYSX_LIB_DIR}")

if(${PLATFORM} STREQUAL "WIN")
	link_libraries(SceneQuery_static_64 SimulationController_static_64 SnippetRender_static_64 SnippetUtils_static_64 LowLevel_static_64 LowLevelAABB_static_64 LowLevelDynamics_static_64 PhysX_64 PhysXCharacterKinematic_static_64 PhysXCommon_64 PhysXCooking_64 PhysXExtensions_static_64 PhysXFoundation_64 PhysXPvdSDK_static_64 PhysXTask_static_64 PhysXVehicle_static_64 SampleBase_static_64)
else()
	link_libraries(libPhysXCharacterKinematic_static_64.a libPhysXCommon_static_64.a libPhysXCooking_static_64.a libPhysXExtensions_static_64.a libPhysXFoundation_static_64.a libPhysXGpu_64.so libPhysXPvdSDK_static_64.a libPhysXVehicle_static_64.a libPhysX_static_64.a)
endif()

# Add source to this project's executable.
add_library (PhysXSharpNative SHARED "PhysXSharpNative.cpp" "PhysXSharpNative.h")

if(${PLATFORM} STREQUAL "LINUX")
	target_link_libraries(PhysXSharpNative Threads::Threads  ${CMAKE_DL_LIBS})
else()
	target_link_libraries(PhysXSharpNative Threads::Threads)
endif()
