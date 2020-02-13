// PhysXSharpNative.cpp : Defines the entry point for the application.
//

#include "PhysXSharpNative.h"
#include <map>
#include <thread>
#include <mutex>
#include <chrono>

using namespace std;
using namespace physx;

// Reference lists
refMap(PxControllerManager)

long refOverlap;
map<long, PxController*> refPxControllers;
map<long, PxVec3> refControllersDir;

map<long, PxRigidStatic*> refPxRigidStatics;
map<long, PxRigidDynamic*> refPxRigidDynamics;

refMap(PxScene)
map<long, std::shared_ptr<ContactReport>> refContactReports;

refMap(PxTriangleMesh)
refMap(PxConvexMesh)

refMapNonPtr(OverlapBuffer)
refMapNonPtr(SharedPxGeometry);

// Global
PxPhysics* gPhysics = nullptr;
PxFoundation* gFoundation = nullptr;
PxCooking* gCooking = nullptr;
PxDefaultAllocator gAllocator;
PxPvd* gPvd = nullptr;
PxDefaultCpuDispatcher*	gDispatcher = nullptr;

std::shared_ptr<ErrorCallback> gErrorCallback;

PxMaterial* gMaterial	= nullptr;

std::mutex step_mutex;
#define lock_step() const std::lock_guard<std::mutex> lockStep(step_mutex);

PxOverlapBufferN<1000> buffer;

std::thread workerThread;


EXPORT void charactersUpdate(float elapsed, float minDist)
{
	for (auto pair : refPxControllers)
	{
		pair.second->move(refControllersDir[pair.first], 0.05f, elapsed, PxControllerFilters());
	}
}


EXPORT void setControllerDirection(long ref, APIVec3 dir)
{
	refControllersDir[ref] = ToPxVec3(dir);
}

EXPORT int sceneOverlap(long refScene, long refGeo, APIVec3 pos, OverlapCallback callback)
{
	lock_step()
	
	refPxScenes[refScene]->overlap(*refSharedPxGeometrys[refGeo], PxTransform(ToPxVec3(pos)), buffer);
	
	for (PxU32 i = 0; i < buffer.nbTouches; ++i)
	{
		const auto touch = buffer.touches[i];
	 	const auto ref = reinterpret_cast<long>(touch.actor->userData);
		
		callback(ref);
	}

	return buffer.nbTouches;
	
}
EXPORT long createSphereGeometry(float radius)
{
	const SharedPxGeometry geo = std::make_shared<PxSphereGeometry>(radius);
	insertMapNoUserData(SharedPxGeometry, geo)
	return insertRef;
}

template<PxConvexMeshCookingType::Enum convexMeshCookingType, bool directInsertion, PxU32 gaussMapLimit>
long createConvexMesh(PxU32 numVerts, const PxVec3* verts)
{
	PxCookingParams params = gCooking->getParams();

	// Use the new (default) PxConvexMeshCookingType::eQUICKHULL
	params.convexMeshCookingType = convexMeshCookingType;

	// If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
	// If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
	params.gaussMapLimit = gaussMapLimit;
	gCooking->setParams(params);

	// Setup the convex mesh descriptor
	PxConvexMeshDesc desc;

	// We provide points only, therefore the PxConvexFlag::eCOMPUTE_CONVEX flag must be specified
	desc.points.data = verts;
	desc.points.count = numVerts;
	desc.points.stride = sizeof(PxVec3);
	desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	PxU32 meshSize = 0;
	PxConvexMesh* convex = nullptr;

	if(directInsertion)
	{
		// Directly insert mesh into PhysX
		convex = gCooking->createConvexMesh(desc, gPhysics->getPhysicsInsertionCallback());
		PXS_ASSERT(convex)
	}
	else
	{
		// Serialize the cooked mesh into a stream.
		PxDefaultMemoryOutputStream outStream;
		bool res = gCooking->cookConvexMesh(desc, outStream);
		PX_UNUSED(res);
		PXS_ASSERT(res)

		// Create the mesh from a stream.
		PxDefaultMemoryInputData inStream(outStream.getData(), outStream.getSize());
		convex = gPhysics->createConvexMesh(inStream);
		PXS_ASSERT(convex)
	}

	insertMapNoUserData(PxConvexMesh, convex)

	return insertRef;
}

void createBV33TriangleMesh(const char* name, PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices)
{
	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(PxU32);

	PxCookingParams params = gCooking->getParams();

	params.midphaseDesc = PxMeshMidPhase::eBVH33;
	params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eSIM_PERFORMANCE;

	gCooking->setParams(params);

	PxDefaultFileOutputStream outBuffer(name);

	gCooking->cookTriangleMesh(meshDesc, outBuffer);

}


EXPORT void createTriangleMesh(const char* name, PxVec3 vertices[], int pointsCount, uint32_t indices[], int triCount)
{
	createBV33TriangleMesh(name, pointsCount, vertices, triCount, indices);
}

EXPORT long loadTriangleMesh(const char* name)
{
	PxDefaultFileInputData stream(name);
	auto triMesh = gPhysics->createTriangleMesh(stream);

	PXS_ASSERT(triMesh)
	insertMapNoUserData(PxTriangleMesh, triMesh)

	return insertRef;
}

EXPORT long createConvexMesh(APIVec3* vertices, int pointsCount)
{
	lock_step()

	const auto verticesPx = reinterpret_cast<PxVec3*>(vertices);
	return createConvexMesh<PxConvexMeshCookingType::eQUICKHULL, true, 256>(pointsCount, verticesPx);

}
EXPORT void cleanupTriangleMesh(long ref)
{
	lock_step()
	
	refPxTriangleMeshs[ref]->release();
	refPxTriangleMeshs[ref] = nullptr;
	refPxTriangleMeshs.erase(ref);
}

EXPORT void cleanupConvexMesh(long ref)
{
	lock_step()
	
	refPxConvexMeshs[ref]->release();
	refPxConvexMeshs[ref] = nullptr;
	refPxConvexMeshs.erase(ref);
}

EXPORT long createBoxGeometry(APIVec3 half)
{
	lock_step()

	const auto geo = std::make_shared<PxBoxGeometry>(ToPxVec3(half));
	const auto insertRef = refCountSharedPxGeometry++;
  	refSharedPxGeometrys.insert({insertRef, geo});

	return insertRef;
}
EXPORT void cleanupGeometry(long ref)
{
	lock_step()
	refSharedPxGeometrys.erase(ref);
}

void setupGeometryType(int type, long refGeo, PxRigidActor* rigid)
{
	if(type == 1)
	{
		PxRigidActorExt::createExclusiveShape(*rigid, *refSharedPxGeometrys[refGeo], *gMaterial);
	}
	else if(type == 2)
	{
		PxConvexMeshGeometry geo;
		geo.convexMesh = refPxConvexMeshs[refGeo];
		
		PxRigidActorExt::createExclusiveShape(*rigid, geo, *gMaterial);
	}
	else if(type == 3)
	{
		PxTriangleMeshGeometry geo;
		geo.triangleMesh = refPxTriangleMeshs[refGeo];
		
		PxRigidActorExt::createExclusiveShape(*rigid, geo, *gMaterial);
	}
}

/// RIGID STATIC
// create
EXPORT long createRigidStatic(int geoType, long refGeo, long refScene, APIVec3 pos, APIQuat quat)
{
	lock_step()

	const auto rigid = gPhysics->createRigidStatic(PxTransform(ToVec3(pos), ToQuat(quat)));
	
	
	const auto insertRef = refOverlap++;
	refPxRigidStatics.insert({insertRef, rigid});
	rigid->userData = reinterpret_cast<void*>(insertRef);

	setupGeometryType(geoType, refGeo, rigid);
	
	refPxScenes[refScene]->addActor(*rigid);
	
	return insertRef;
}
EXPORT void destroyRigidStatic(long ref)
{
	lock_step()
	
	const auto actor = refPxRigidStatics[ref];
	actor->getScene()->removeActor(*actor);
	
	refPxRigidStatics[ref]->release();
	refPxRigidStatics[ref] = nullptr;
	refPxRigidStatics.erase(ref);
}


// get
EXPORT APIVec3 getRigidStaticPosition(long ref)
{
	lock_step()
	
 	return ToVec3(refPxRigidStatics[ref]->getGlobalPose().p);
}
EXPORT APIQuat getRigidStaticRotation(long ref)
{
	lock_step()
	
	return ToQuat(refPxRigidStatics[ref]->getGlobalPose().q);
}

// set
EXPORT void setRigidStaticPosition(long ref, APIVec3 p)
{
	lock_step()
	
	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToPxVec3(p)));
}
EXPORT void setRigidStaticRotation(long ref, APIQuat q)
{
	lock_step()
	
	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToQuat(q)));	
}

/// RIGID DYNAMIC
// set
EXPORT void setRigidDynamicTransform(long ref, APIVec3 pos, APIQuat q)
{
	lock_step()

	refPxRigidDynamics[ref]->setGlobalPose(PxTransform(ToVec3(pos), ToQuat(q)));
}


EXPORT void setRigidDynamicKinematicTarget(long ref, APIVec3 p, APIQuat q)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setKinematicTarget(PxTransform(ToPxVec3(p), ToPxQuat(q)));
}

EXPORT void setRigidDynamicLinearVelocity(long ref, APIVec3 v)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setLinearVelocity(ToPxVec3(v), true);
}

EXPORT APIVec3 getRigidDynamicLinearVelocity(long ref)
{
	lock_step()

	return ToVec3(refPxRigidDynamics[ref]->getLinearVelocity());
}

EXPORT void setRigidDynamicLinearDamping(long ref, float v)
{
	lock_step()

	refPxRigidDynamics[ref]->setLinearDamping(v);
}

EXPORT void setRigidDynamicAngularDamping(long ref, float v)
{
	lock_step()

	refPxRigidDynamics[ref]->setAngularDamping(v);
}

EXPORT void addRigidDynamicForce(long ref, APIVec3 v, PxForceMode::Enum forceMode)
{
	lock_step()

	refPxRigidDynamics[ref]->addForce(ToPxVec3(v), forceMode);
}

EXPORT void addRigidDynamicTorque(long ref, APIVec3 v, PxForceMode::Enum forceMode)
{
	lock_step()

	refPxRigidDynamics[ref]->addTorque(ToPxVec3(v), forceMode);
}
EXPORT void setRigidDynamicAngularVelocity(long ref, APIVec3 v)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setAngularVelocity(ToPxVec3(v), true);
}


EXPORT void setRigidDynamicMaxLinearVelocity(long ref, float v)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setMaxLinearVelocity(v);
}
EXPORT void setRigidDynamicMaxAngularVelocity(long ref, float v)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setMaxAngularVelocity(v);
}

// get
EXPORT APIVec3 getRigidDynamicPosition(long ref)
{
	lock_step()

	return ToVec3(refPxRigidDynamics[ref]->getGlobalPose().p);
}
EXPORT APIQuat getRigidDynamicRotation(long ref)
{
	lock_step()

	return ToQuat(refPxRigidDynamics[ref]->getGlobalPose().q);
}

EXPORT APIVec3 getRigidDynamicAngularVelocity(long ref)
{
	lock_step()

	return ToVec3(refPxRigidDynamics[ref]->getAngularVelocity());
}

EXPORT float getRigidDynamicMaxAngularVelocity(long ref)
{
	lock_step()
	return refPxRigidDynamics[ref]->getMaxAngularVelocity();
}
EXPORT float getRigidDynamicMaxLinearVelocity(long ref)
{
	lock_step()

	return refPxRigidDynamics[ref]->getMaxLinearVelocity();
}



// create
EXPORT long createRigidDynamic(int geoType, long refGeo, long refScene, bool kinematic, bool ccd, bool retain, float mass, APIVec3 pos, APIQuat quat)
{
	lock_step()
	
	const auto rigid = gPhysics->createRigidDynamic(PxTransform(ToVec3(pos), ToQuat(quat)));

	const auto insertRef = refOverlap++;
	refPxRigidDynamics.insert({insertRef, rigid});
	rigid->userData = reinterpret_cast<void*>(insertRef);

	rigid->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, kinematic);
	rigid->setRigidBodyFlag(PxRigidBodyFlag::eRETAIN_ACCELERATIONS, retain);

	if(!kinematic)
		rigid->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, ccd);

	rigid->setMass(mass);

	setupGeometryType(geoType, refGeo, rigid);
	
	refPxScenes[refScene]->addActor(*rigid);
	return insertRef;
}
EXPORT void destroyRigidDynamic(long ref)
{
	lock_step()
	
	const auto actor = refPxRigidDynamics[ref];
	actor->getScene()->removeActor(*actor);
	
	refPxRigidDynamics[ref]->release();
	refPxRigidDynamics[ref] = nullptr;
	refPxRigidDynamics.erase(ref);
}

/// CAPSULE CHARACTER
EXPORT long createCapsuleCharacter(long refScene, APIVec3 pos, APIVec3 up, float height, float radius, float stepOffset)
{
	lock_step()
	const auto insertRef = refOverlap++;
	
	PxCapsuleControllerDesc desc;
	desc.height = height;
	desc.radius = radius;
	desc.position = ToPxVec3d(pos);
	desc.upDirection = ToPxVec3(up);
	desc.stepOffset = stepOffset;
	desc.material = gMaterial;
	
	const auto c = refPxControllerManagers[refScene]->createController(desc);
	c->setUserData(reinterpret_cast<void*>(insertRef));
	c->getActor()->userData = reinterpret_cast<void*>(insertRef);
	
	
	refPxControllers.insert({insertRef, c});
	refControllersDir.insert({insertRef, PxVec3(0, 0, 0)});

	return insertRef;
}
EXPORT void destroyController(long ref)
{
	lock_step()
	releaseMap(PxController, ref)
}

// get
EXPORT APIDoubleVec3 getControllerPosition(long ref)
{
	lock_step()
	return ToVec3d(refPxControllers[ref]->getPosition());
}
EXPORT APIDoubleVec3 getControllerFootPosition(long ref)
{
	lock_step()
	return ToVec3d(refPxControllers[ref]->getFootPosition());
}

// set
EXPORT void setControllerPosition(long ref, APIDoubleVec3 p)
{
	lock_step()
	refPxControllers[ref]->setPosition(ToPxVec3d(p));
}

EXPORT void setControllerFootPosition(long ref, APIDoubleVec3 p)
{
	lock_step()
	refPxControllers[ref]->setFootPosition(ToPxVec3d(p));
}

static PxFilterFlags filterShader(
	PxFilterObjectAttributes attributes0,
	PxFilterData filterData0,
	PxFilterObjectAttributes attributes1,
	PxFilterData filterData1,
	PxPairFlags& pairFlags,
	const void* constantBlock,
	PxU32 constantBlockSize)
{
    PX_UNUSED(filterData0);
    PX_UNUSED(filterData1);
    PX_UNUSED(constantBlockSize);
    PX_UNUSED(constantBlock);

    pairFlags = PxPairFlag::eSOLVE_CONTACT
                | PxPairFlag::eDETECT_DISCRETE_CONTACT
                | PxPairFlag::eDETECT_CCD_CONTACT
                | PxPairFlag::eNOTIFY_TOUCH_FOUND
                | PxPairFlag::eNOTIFY_TOUCH_PERSISTS
                | PxPairFlag::eNOTIFY_CONTACT_POINTS;

    if (PxFilterObjectIsKinematic(attributes0) && PxFilterObjectIsKinematic(attributes1))
    {
        pairFlags &= ~PxPairFlag::eSOLVE_CONTACT;
    }
	return PxFilterFlag::eDEFAULT;
}


/// SCENE
EXPORT long createScene(APIVec3 gravity, ContactReportCallbackFunc func)
{
	const auto insertRef = refCountPxScene++;
	
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = ToPxVec3(gravity);
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

	sceneDesc.kineKineFilteringMode = PxPairFilteringMode::eKEEP;
    sceneDesc.staticKineFilteringMode = PxPairFilteringMode::eKEEP;

	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader = filterShader;

	auto contactReport = std::make_shared<ContactReport>(func);
    sceneDesc.simulationEventCallback = contactReport.get();

	auto scene = gPhysics->createScene(sceneDesc);

	auto controllerManager = PxCreateControllerManager(*scene, false);
	refPxControllerManagers.insert({insertRef, controllerManager});;
	refPxScenes.insert({insertRef, scene});
	refContactReports.insert({insertRef, contactReport});
	
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
	lock_step()

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

EXPORT void initLog(DebugLogFunc func, DebugLogErrorFunc func2)
{
	debugLog = func;
	debugLogError = func2;
}

EXPORT void initPhysics(bool isCreatePvd, int numThreads, float toleranceLength, float toleranceSpeed, ErrorCallbackFunc func)
{
 	debugLog("init physics native library v4 kinematic pairs");

	gErrorCallback = std::make_shared<ErrorCallback>(func);
	
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, *gErrorCallback);

	PxTolerancesScale scale;
	scale.length = toleranceLength;        // typical length of an object
	scale.speed = toleranceSpeed;        // typical speed of an object, gravity*1s is a reasonable choice

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(scale));
		
	if(isCreatePvd)
	{
		gPvd = PxCreatePvd(*gFoundation);
		PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 100);
		gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
	}

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale,true,gPvd);

	gDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
	
}
EXPORT void initGlobalMaterial(float staticFriction, float dynamicFriction, float restitution)
{
	gMaterial = gPhysics->createMaterial(staticFriction, dynamicFriction, restitution);
}

EXPORT void cleanupPhysics()
{
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();

		gPvd->release();
		gPvd = nullptr;
		
		PX_RELEASE(transport)
	}
	PX_RELEASE(gFoundation);
}