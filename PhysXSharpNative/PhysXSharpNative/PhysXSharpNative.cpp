// PhysXSharpNative.cpp : Defines the entry point for the application.
//

#include "PhysXSharpNative.h"

using namespace std;
using namespace physx;

EXPORT ErrorCallback* CreateErrorCallback(ErrorCallbackFunc func)
{
	const auto callback = std::make_shared<ErrorCallback>(func);
	return callback.get(); 
}

EXPORT PxDefaultAllocator* CreateDefaultAllocator()
{
	return nullptr;
}


EXPORT PxVec3* CreateVec3(float x, float y, float z)
{
	const auto vec = std::make_shared<PxVec3>(x, y,z);
	return vec.get();
}

EXPORT void SetVec3(PxVec3* vec3, float x, float y, float z)
{
	vec3->x = x;
	vec3->y = y;
	vec3->z = z;
}

EXPORT PxFoundation* CreateFoundation(ErrorCallback* errorCallback, PxDefaultAllocator* allocator)
{

	return PxCreateFoundation(PX_PHYSICS_VERSION, *allocator, *errorCallback);
}

EXPORT PxPvd* CreatePvd(PxFoundation* foundation, const char* host)
{

	auto gPvd = PxCreatePvd(*foundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(host, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	return gPvd;
}

EXPORT PxPhysics* CreatePhysics(PxFoundation* foundation, PxPvd* pvd)
{
	return PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(),true, pvd);
}

EXPORT PxSceneDesc* CreateSceneDesc(PxPhysics* physics, PxVec3 gravity, PxCpuDispatcher* cpuDispatcher)
{
	auto sceneDesc = std::make_shared<PxSceneDesc>(physics->getTolerancesScale());
	
	sceneDesc->gravity = gravity;
	sceneDesc->cpuDispatcher	= cpuDispatcher;
	sceneDesc->filterShader	= PxDefaultSimulationFilterShader;

	return sceneDesc.get();
}

EXPORT PxScene* CreateScene(PxPhysics* physics, PxSceneDesc* desc)
{	
	auto scene = physics->createScene(*desc);

	PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	
	return scene;
}

EXPORT PxCpuDispatcher* CreateCpuDispatcher(int numThreads)
{
	return PxDefaultCpuDispatcherCreate(numThreads);
}

EXPORT PxMaterial* CreateMaterial(PxPhysics* physics, float staticFriction, float dynamicFriction, float restitution)
{
	return physics->createMaterial(staticFriction, dynamicFriction, restitution);
}

