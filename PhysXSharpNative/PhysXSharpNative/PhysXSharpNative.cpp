// PhysXSharpNative.cpp : Defines the entry point for the application.
//

#include "PhysXSharpNative.h"

using namespace std;
using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = nullptr;
PxPhysics*				gPhysics	= nullptr;

//PxDefaultCpuDispatcher*	gDispatcher = nullptr;
//PxScene*				gScene		= nullptr;
//
//PxMaterial*				gMaterial	= nullptr;
//
//PxPvd*                  gPvd        = nullptr;

//
//PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
//{
//	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
//	dynamic->setAngularDamping(0.5f);
//	dynamic->setLinearVelocity(velocity);
//	gScene->addActor(*dynamic);
//	return dynamic;
//}

int main()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,nullptr);

	//PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	//sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

	//gDispatcher = PxDefaultCpuDispatcherCreate(2);
	//sceneDesc.cpuDispatcher	= gDispatcher;
	//sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	//gScene = gPhysics->createScene(sceneDesc);

	//PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	//if(pvdClient)
	//{
	//	pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
	//	pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
	//	pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	//}
	//gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	//PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	//gScene->addActor(*groundPlane);

	//auto actor = createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,-50,-100));
	//
	//static const PxU32 FRAME_COUNT = 100;
	//for(PxU32 i=0; i<FRAME_COUNT; i++)
	//{
	//	gScene->simulate(1.0f/60.0f);
	//	gScene->fetchResults(true);

	//	cout << gScene->getTimestamp() << " ("  << actor->getGlobalPose().p.x << ", " << actor->getGlobalPose().p.y << ", " << actor->getGlobalPose().p.z << ")" << endl;
	//	
	//}


	return 0;
}


