# CMakeList.txt : CMake project for PhysXSharpNative, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(dlib REQUIRED)

set(PHYSX_SDK "../../PhysX")
set(PHYSX_LIB_DIR "libs/test2.chekced")

include_directories("${PHYSX_SDK}/pxshared/include")
include_directories("${PHYSX_SDK}/physx/include")

add_library (PhysXSharpNative SHARED "PhysXSharpNative.cpp" "PhysXSharpNative.h")

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
#target_link_libraries(PhysXSharpNative PhysX_static_64
#	                                   PhysXCharacterKinematic_static_64
#									   PhysXCommon_static_64
#									   PhysXCooking_static_64
#									   PhysXExtensions_static_64
#									   PhysXFoundation_static_64
#									   PhysXPvdSDK_static_64
#									   PhysXVehicle_static_64
#		                               Threads::Threads
#										${CMAKE_DL_LIBS})