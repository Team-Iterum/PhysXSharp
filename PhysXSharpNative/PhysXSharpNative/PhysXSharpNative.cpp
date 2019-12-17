// PhysXSharpNative.cpp : Defines the entry point for the application.
//

#include "PhysXSharpNative.h"
#include <map>

using namespace std;
using namespace physx;

// Reference lists
refMap(PxControllerManager)
refMap(PxController)
refMap(PxRigidStatic)
refMap(PxRigidDynamic)
refMap(PxScene)
refMapNonPtr(OverlapBuffer)
refMapNonPtr(SharedPxGeometry);

// Global
PxPhysics* gPhysics = nullptr;
PxFoundation* gFoundation = nullptr;
PxDefaultAllocator gAllocator;
PxPvd* gPvd = nullptr;
PxDefaultCpuDispatcher*	gDispatcher = nullptr;

std::shared_ptr<ErrorCallback> gErrorCallback;

PxMaterial* gMaterial	= nullptr;


EXPORT void setControllerDirection(long ref, APIVec3 dir)
{
	refPxControllers[ref]->setUpDirection(ToPxVec3(dir));
}

EXPORT int sceneOverlap(long refScene, long refGeo, APIVec3 pos, OverlapCallback callback)
{
	
	refPxScenes[refScene]->overlap(*refSharedPxGeometrys[refGeo], PxTransform(ToPxVec3(pos)), refOverlapBuffers[refScene]);
	const auto count = refOverlapBuffers[refScene].getNbTouches();
	for (PxU32 i = 0; i < count; ++i)
	{
		const auto touch = refOverlapBuffers[refScene].getTouch(i);
	 	const auto ref = reinterpret_cast<long>(touch.actor->userData);
		
		callback(ref);
	}

	return count;
	
}
EXPORT long createSphereGeometry(float radius)
{
	const SharedPxGeometry geo = std::make_shared<PxSphereGeometry>(radius);
	insertMapNoUserData(SharedPxGeometry, geo);
	return insertRef;
}
EXPORT long createBoxGeometry(APIVec3 half)
{
	const auto geo = std::make_shared<PxBoxGeometry>(ToPxVec3(half));
	insertMapNoUserData(SharedPxGeometry, geo);

	return insertRef;
}
EXPORT void cleanupGeometry(long ref)
{
	refSharedPxGeometrys.erase(ref);
}
EXPORT long createRigidStatic(long refGeo, long refScene, APIVec3 pos, APIQuat quat)
{
	const auto rigid = gPhysics->createRigidStatic(PxTransform(ToVec3(pos), ToQuat(quat)));

	insertMap(PxRigidStatic, rigid);

	return insertRef;
}
EXPORT void destroyRigidStatic(long ref)
{
	releaseMap(PxRigidStatic, ref)
}

EXPORT APIVec3 getRigidStaticPosition(long ref)
{
 	return ToVec3(refPxRigidStatics[ref]->getGlobalPose().p);
}

EXPORT APIQuat getRigidStaticRotation(long ref)
{
	return ToQuat(refPxRigidStatics[ref]->getGlobalPose().q);
}
EXPORT void setRigidStaticPosition(long ref, APIVec3 p)
{
	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToPxVec3(p)));
}
EXPORT void setRigidStaticRotation(long ref, APIQuat q)
{
	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToQuat(q)));	
}
EXPORT long createRigidDynamic(APIVec3 pos, APIQuat quat)
{
	const auto rigid = gPhysics->createRigidDynamic(PxTransform(ToVec3(pos), ToQuat(quat)));
	insertMap(PxRigidDynamic, rigid)

	return insertRef;
}
EXPORT void destroyRigidDynamic(long ref)
{
	releaseMap(PxRigidDynamic, ref)
}
EXPORT long createCapsuleCharacter(long refScene, APIVec3 pos, APIVec3 up, float height, float radius, float stepOffset)
{
	const auto insertRef = refCountPxController++;
	
	PxCapsuleControllerDesc desc;
	desc.height = height;
	desc.radius = radius;
	desc.position = ToPxVec3d(pos);
	desc.upDirection = ToPxVec3(up);
	desc.stepOffset = stepOffset;
	desc.material = gMaterial;
	
	const auto c = refPxControllerManagers[refScene]->createController(desc);
	c->setUserData(reinterpret_cast<void*>(insertRef));
	
	refPxControllers.insert({insertRef, c});;

	return insertRef;
}
EXPORT void destroyController(long ref)
{
	releaseMap(PxController, ref);
}

EXPORT APIDoubleVec3 getControllerPosition(long ref)
{
	return ToVec3d(refPxControllers[ref]->getPosition());
}

EXPORT APIDoubleVec3 getControllerFootPosition(long ref)
{
	return ToVec3d(refPxControllers[ref]->getFootPosition());
}

EXPORT void setControllerPosition(long ref, APIDoubleVec3 p)
{
	refPxControllers[ref]->setPosition(ToPxVec3d(p));
}

EXPORT void setControllerFootPosition(long ref, APIDoubleVec3 p)
{
	refPxControllers[ref]->setFootPosition(ToPxVec3d(p));
}

EXPORT long createScene(APIVec3 gravity)
{
	const auto insertRef = refCountPxScene++;
	
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = ToPxVec3(gravity);
	
	
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	auto scene = gPhysics->createScene(sceneDesc);

	auto controllerManager = PxCreateControllerManager(*scene, true);
	refPxControllerManagers.insert({insertRef, controllerManager});;
	refPxScenes.insert({insertRef, scene});
	
	PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	return insertRef;
}

EXPORT void stepPhysics(long ref, float dt)
{
	refPxScenes[ref]->simulate(dt);
	refPxScenes[ref]->fetchResults(true);
}

EXPORT void cleanupScene(long ref)
{
	releaseMap(PxScene, ref)
}
EXPORT long getSceneTimestamp(long ref)
{
	return refPxScenes[ref]->getTimestamp();
}

EXPORT void initPhysics(bool isCreatePvd, int numThreads, ErrorCallbackFunc func)
{
	gErrorCallback = std::make_shared<ErrorCallback>(func);
	
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, *gErrorCallback);

	if(isCreatePvd)
	{
		gPvd = PxCreatePvd(*gFoundation);
		PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
		gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
	}

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

	gDispatcher = PxDefaultCpuDispatcherCreate(numThreads);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);
	
}
EXPORT void cleanupPhysics()
{
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();

		gPvd->release();
		gPvd = nullptr;
		
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
}