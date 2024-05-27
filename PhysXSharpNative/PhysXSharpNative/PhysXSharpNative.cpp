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

map<uint64_t, uint64_t> refOverlaps;
map<uint64_t, PxController*> refPxControllers;
map<uint64_t, PxVec3> refControllersDir;
map<uint64_t, PxControllerCollisionFlags> refControllersFlags;

map<uint64_t, PxRigidStatic*> refPxRigidStatics;
map<uint64_t, map<uint64_t, PxRigidDynamic*>> refPxRigidDynamics;

refMap(PxScene)
map<uint64_t, std::shared_ptr<ContactReport>> refContactReports;

refMap(PxTriangleMesh)
refMap(PxConvexMesh)
refMap(PxMaterial)

uint64_t refTerrain;
map<uint64_t, PxRigidStatic*> refTerrains;

refMapNonPtr(OverlapBuffer)
refMapNonPtr(RaycastBuffer)

refMapNonPtr(SharedPxGeometry);

// Global
PxPhysics* gPhysics = nullptr;
PxFoundation* gFoundation = nullptr;
PxCooking* gCooking = nullptr;
PxDefaultAllocator gAllocator;
PxPvd* gPvd = nullptr;
PxDefaultCpuDispatcher*	gDispatcher = nullptr;

std::shared_ptr<ErrorCallback> gErrorCallback;

// locksteps
//std::mutex step_mutex;
//#define lock_step() const std::lock_guard<std::mutex> lockStep(step_mutex);

EXPORT void characterUpdate(uint64_t ref, float elapsed, float minDist)
{
	//lock_step()
	refControllersFlags[ref] = refPxControllers[ref]->move(refControllersDir[ref], minDist, elapsed, PxControllerFilters());
}


EXPORT void charactersUpdate(uint64_t refScene, float elapsed, float minDist)
{
	//lock_step()
	const auto controllers = refPxControllerManagers[refScene];
	for (size_t i = 0; i < controllers->getNbControllers(); i++)
	{
		const auto controller = controllers->getController(i);
		const auto ref = reinterpret_cast<uint64_t>(controller->getUserData());

		refControllersFlags[ref] = controller->move(refControllersDir[ref], minDist, elapsed, PxControllerFilters());
		
	}
}


EXPORT void setControllerDirection(uint64_t ref, APIVec3 dir)
{
	refControllersDir[ref] = ToPxVec3(dir);
}

EXPORT void setControllerHeight(uint64_t ref, PxReal height)
{
	refPxControllers[ref]->resize(height);
}

EXPORT bool isControllerCollisionUp(uint64_t ref) { return refControllersFlags[ref].isSet(PxControllerCollisionFlag::eCOLLISION_UP); }
EXPORT bool isControllerCollisionDown(uint64_t ref) { return refControllersFlags[ref].isSet(PxControllerCollisionFlag::eCOLLISION_DOWN); }
EXPORT bool isControllerCollisionSides(uint64_t ref) { return refControllersFlags[ref].isSet(PxControllerCollisionFlag::eCOLLISION_SIDES); }
EXPORT bool isControllerMovingUp(uint64_t ref) {
	PxControllerState state;
	refPxControllers[ref]->getState(state);
	return state.isMovingUp;
}



////
////// OVERLAPS
EXPORT uint64_t createOverlapBuffer(uint32_t max)
{

	uint64_t insertRef = 0;

	if (max <= 100) {
		insertRef = refCountOverlapBuffer++;
		OverlapBuffer buffer = std::make_shared<OverlapBufferN<100>>(max);
		refOverlapBuffers.insert({ insertRef, buffer });
	}
	else if (max <= 1000) {
		insertRef = refCountOverlapBuffer++;
		OverlapBuffer buffer = std::make_shared<OverlapBufferN<1000>>(max);
		refOverlapBuffers.insert({ insertRef, buffer });
	}
	else if (max <= 5000) {
		insertRef = refCountOverlapBuffer++;
		OverlapBuffer buffer = std::make_shared<OverlapBufferN<5000>>(max);
		refOverlapBuffers.insert({ insertRef, buffer });
	}
	else if (max <= 15000) {
		insertRef = refCountOverlapBuffer++;
		OverlapBuffer buffer = std::make_shared<OverlapBufferN<15000>>(max);
		refOverlapBuffers.insert({ insertRef, buffer });
	}
	
	
	return insertRef;
}

///
/// RAYCASTS
EXPORT uint64_t createRaycastBuffer(uint32_t max)
{
	uint64_t insertRef = 0;

	if (max <= 10) {
		insertRef = refCountRaycastBuffer++;
		RaycastBuffer buffer = std::make_shared<RaycastBufferN<10>>(max);
		refRaycastBuffers.insert({ insertRef, buffer });
	}
	else if (max <= 100) {
		insertRef = refCountRaycastBuffer++;
		RaycastBuffer buffer = std::make_shared<RaycastBufferN<100>>(max);
		refRaycastBuffers.insert({ insertRef, buffer });
	}

	return insertRef;
}

EXPORT void cleanupRaycastBuffer(uint64_t ref)
{
	refRaycastBuffers[ref] = nullptr;
	refRaycastBuffers.erase(ref);
}

EXPORT void cleanupOverlapBuffer(uint64_t ref)
{
	refOverlapBuffers[ref] = nullptr;
	refOverlapBuffers.erase(ref);
}


EXPORT int sceneRaycast(uint64_t refScene, uint64_t refRaycastBuffer, APIVec3 origin, APIVec3 unitDir, float distance, RaycastCallback callback)
{
    // lock_step()
    
    auto buffer = *refRaycastBuffers[refRaycastBuffer];

    refPxScenes[refScene]->raycast(ToVec3(origin), ToVec3(unitDir), distance, buffer);
    
    for (PxU32 i = 0; i < buffer.nbTouches; ++i)
    {
        const auto touch = buffer.touches[i];
        const auto ref = reinterpret_cast<uint64_t>(touch.actor->userData);
        
        callback(i, ref, touch.distance, ToVec3(touch.position), ToVec3(touch.normal));
    }

    return buffer.nbTouches;
}


EXPORT int sceneOverlap(uint64_t refScene, uint64_t refBuffer, uint64_t* refs, uint64_t refGeo, APIVec3 pos, uint32_t allDynamicStatic)
{
	// lock_step()

    auto buffer = *refOverlapBuffers[refBuffer];

	auto flags = PxQueryFilterData(PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC);
	if (allDynamicStatic == 1) flags = PxQueryFilterData(PxQueryFlag::eDYNAMIC);
	if (allDynamicStatic == 2) flags = PxQueryFilterData(PxQueryFlag::eSTATIC);

    refPxScenes[refScene]->overlap(*refSharedPxGeometrys[refGeo], PxTransform(ToPxVec3(pos)), buffer, flags);


    for (PxU32 i = 0; i < buffer.nbTouches; ++i)
    {
        const auto touch = buffer.touches[i];
        const auto ref = reinterpret_cast<uint64_t>(touch.actor->userData);
		refs[i] = ref;
     
    }

	return buffer.nbTouches;
    
}


EXPORT uint64_t createSphereGeometry(float radius)
{
	const SharedPxGeometry geo = std::make_shared<PxSphereGeometry>(radius);
	insertMapNoUserData(SharedPxGeometry, geo)
	return insertRef;
}

EXPORT uint64_t createCapsuleGeometry(PxReal radius, PxReal halfHeight)
{
	const SharedPxGeometry geo = std::make_shared<PxCapsuleGeometry>(radius, halfHeight);
	insertMapNoUserData(SharedPxGeometry, geo)
	return insertRef;
}

template<PxConvexMeshCookingType::Enum convexMeshCookingType, bool directInsertion, PxU32 gaussMapLimit>
uint64_t createConvexMesh(PxU32 numVerts, const PxVec3* verts)
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


    // Print the elapsed time for comparison
    printf("\t -----------------------------------------------\n");
    printf("\t Create (BV33) triangle mesh with %d triangles: \n", numTriangles);

}

// Setup common cooking params
void setupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData)
{
        // we suppress the triangle mesh remap table computation to gain some speed, as we will not need it
    // in this snippet
    params.suppressTriangleMeshRemapTable = true;

    // If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid.
    // The following conditions are true for a valid triangle mesh :
    //  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
    //  2. There are no large triangles(within specified PxTolerancesScale.)
    // It is recommended to run a separate validation check in debug/checked builds, see below.

    if (!skipMeshCleanup)
        params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
    else
        params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

    // If DISABLE_ACTIVE_EDGES_PREDOCOMPUTE is set, the cooking does not compute the active (convex) edges, and instead
    // marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change
    // the collision behavior, as all edges of the triangle mesh will now be considered active.
    if (!skipEdgeData)
        params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
    else
        params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
}

// Creates a triangle mesh using BVH34 midphase with different settings.
void createBV34TriangleMesh(const char* name, PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices,
    bool skipMeshCleanup, bool skipEdgeData, const PxU32 numTrisPerLeaf)
{
    

    PxTriangleMeshDesc meshDesc;
    meshDesc.points.count = numVertices;
    meshDesc.points.data = vertices;
    meshDesc.points.stride = sizeof(PxVec3);
    meshDesc.triangles.count = numTriangles;
    meshDesc.triangles.data = indices;
    meshDesc.triangles.stride = 3 * sizeof(PxU32);

    PxCookingParams params = gCooking->getParams();

    // Create BVH34 midphase
    params.midphaseDesc = PxMeshMidPhase::eBVH34;

    // setup common cooking params
    setupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);

    // Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
    // and worse cooking performance. Cooking time is better when more triangles per leaf are used.
#if PX_PHYSICS_VERSION_MAJOR==3
    params.midphaseDesc.mBVH34Desc.numTrisPerLeaf = numTrisPerLeaf;
#else
	params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = numTrisPerLeaf;
#endif



    gCooking->setParams(params);
    
    PxDefaultFileOutputStream outBuffer(name);
    gCooking->cookTriangleMesh(meshDesc, outBuffer);

    // Print the elapsed time for comparison
    printf("\t -----------------------------------------------\n");
    printf("\t Create triangle mesh with %d triangles: \n", numTriangles);
    !skipEdgeData ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
    !skipMeshCleanup ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
    printf("\t\t Num triangles per leaf: %d \n", numTrisPerLeaf);
}



void createTriangleMesh(const char* name, PxVec3 vertices[], int pointsCount, uint32_t indices[], int triCount)
{
    // Favor runtime speed, cleaning the mesh and precomputing active edges. Store the mesh in a stream.
    // These are the default settings, suitable for offline cooking.
    createBV33TriangleMesh(name, pointsCount, vertices, triCount, indices);
}

uint64_t loadTriangleMesh(const char* name)
{
	PxDefaultFileInputData stream(name);
	auto triMesh = gPhysics->createTriangleMesh(stream);

	PXS_ASSERT(triMesh)
	insertMapNoUserData(PxTriangleMesh, triMesh)

	return insertRef;
}

EXPORT uint64_t createConvexMesh(APIVec3* vertices, int pointsCount)
{
	//lock_step()

	const auto verticesPx = reinterpret_cast<PxVec3*>(vertices);
	return createConvexMesh<PxConvexMeshCookingType::eQUICKHULL, true, 256>(pointsCount, verticesPx);

}
EXPORT void cleanupTriangleMesh(uint64_t ref)
{
	//lock_step()
	
	refPxTriangleMeshs[ref]->release();
	refPxTriangleMeshs[ref] = nullptr;
	refPxTriangleMeshs.erase(ref);
}

EXPORT void cleanupConvexMesh(uint64_t ref)
{
	//lock_step()
	
	refPxConvexMeshs[ref]->release();
	refPxConvexMeshs[ref] = nullptr;
	refPxConvexMeshs.erase(ref);
}

EXPORT uint64_t createBoxGeometry(APIVec3 half)
{
	//lock_step()

	const auto geo = std::make_shared<PxBoxGeometry>(ToPxVec3(half));
	const auto insertRef = refCountSharedPxGeometry++;
  	refSharedPxGeometrys.insert({insertRef, geo});

	return insertRef;
}
EXPORT void cleanupGeometry(uint64_t ref)
{
	//lock_step()
	refSharedPxGeometrys.erase(ref);
}

void setupGeometryType(int type, int refGeoCount, uint64_t refGeo[], PxRigidActor* rigid, PxMaterial& mat)
{
    for (int i = 0; i < refGeoCount; i++)
    {
        if(type == 1)
        {
            PxRigidActorExt::createExclusiveShape(*rigid, *refSharedPxGeometrys[refGeo[i]], mat);
        }
        else if(type == 2)
        {
            PxConvexMeshGeometry geo;
            geo.convexMesh = refPxConvexMeshs[refGeo[i]];
            
            PxRigidActorExt::createExclusiveShape(*rigid, geo, mat);
        }
        else if(type == 3)
        {
            PxTriangleMeshGeometry geo;
            geo.triangleMesh = refPxTriangleMeshs[refGeo[i]];
            
            PxRigidActorExt::createExclusiveShape(*rigid, geo, mat);
            
        }
    }
}

/// RIGID STATIC
// create
EXPORT uint64_t createRigidStatic(int geoType, uint64_t refGeo, uint64_t refScene, uint64_t refMat, APIVec3 pos, APIQuat quat, bool isTrigger)
{
	//lock_step()

	const auto rigid = gPhysics->createRigidStatic(PxTransform(ToVec3(pos), ToQuat(quat)));
	
	
	const auto insertRef = refOverlaps[refScene]++;
	refPxRigidStatics.insert({insertRef, rigid});
	rigid->userData = reinterpret_cast<void*>(insertRef);

    uint64_t refGeoArr[] = { refGeo };
	setupGeometryType(geoType, 1, refGeoArr, rigid, *refPxMaterials[refMat]);
	
    if(isTrigger)
    {
        PxShape* triggerShape;
        rigid->getShapes(&triggerShape, 1);
        triggerShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
        triggerShape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
        
        PxFilterData filterData;
        filterData.word0 = 1;
        
        triggerShape->setSimulationFilterData(filterData);

    }
    
	refPxScenes[refScene]->addActor(*rigid);
	
	return insertRef;
}
EXPORT void destroyRigidStatic(uint64_t ref)
{
	//lock_step()
	
	const auto actor = refPxRigidStatics[ref];
	actor->getScene()->removeActor(*actor);
	
	refPxRigidStatics[ref]->release();
	refPxRigidStatics[ref] = nullptr;
	refPxRigidStatics.erase(ref);
}

EXPORT void destroyTerrain(uint64_t ref)
{
	//lock_step()

	const auto actor = refTerrains[ref];
	actor->getScene()->removeActor(*actor);

	refTerrains[ref]->release();
	refTerrains[ref] = nullptr;
	refTerrains.erase(ref);
}

EXPORT APIVec3 getTerrainPosition(uint64_t ref)
{
	//lock_step()

	return ToVec3(refTerrains[ref]->getGlobalPose().p);
}

EXPORT void setTerrainPosition(uint64_t ref, APIVec3 p)
{
	//lock_step()

	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToPxVec3(p)));
}


// get
EXPORT APIVec3 getRigidStaticPosition(uint64_t ref)
{
	//lock_step()
	
 	return ToVec3(refPxRigidStatics[ref]->getGlobalPose().p);
}
EXPORT APIQuat getRigidStaticRotation(uint64_t ref)
{
	//lock_step()
	
	return ToQuat(refPxRigidStatics[ref]->getGlobalPose().q);
}

// set
EXPORT void setRigidDynamicLockFlag(uint64_t refScene, uint64_t ref, PxRigidDynamicLockFlag::Enum lockFlag, bool value)
{
	//lock_step()

	refPxRigidDynamics[refScene][ref]->setRigidDynamicLockFlag(lockFlag, value);
}



// set
EXPORT void setRigidStaticPosition(uint64_t ref, APIVec3 p)
{
	//lock_step()
	
	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToPxVec3(p)));
}
EXPORT void setRigidStaticRotation(uint64_t ref, APIQuat q)
{
	//lock_step()
	
	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToQuat(q)));	
}

/// RIGID DYNAMIC
// set
EXPORT void setRigidDynamicTransform(uint64_t refScene, uint64_t ref, APITransform t)
{
	//lock_step()
	if (refPxRigidDynamics[refScene][ref] == nullptr) return;

	refPxRigidDynamics[refScene][ref]->setGlobalPose(ToPxTrans(t));
}


EXPORT APIBounds3 getBounds(uint64_t refScene, uint64_t ref, float inflation = 1.01F)
{
	//lock_step()
	const auto bounds = refPxRigidDynamics[refScene][ref]->getWorldBounds(inflation);
	return ToBounds3(bounds);
}


EXPORT void setRigidDynamicKinematicTarget(uint64_t refScene, uint64_t ref, APITransform t)
{
	//lock_step()
	
	refPxRigidDynamics[refScene][ref]->setKinematicTarget(ToPxTrans(t));
}


EXPORT void setRigidDynamicLinearVelocity(uint64_t refScene, uint64_t ref, APIVec3 v)
{
	//lock_step()
	
	refPxRigidDynamics[refScene][ref]->setLinearVelocity(ToPxVec3(v), true);
}

EXPORT APIVec3 getRigidDynamicLinearVelocity(uint64_t refScene, uint64_t ref)
{
	//lock_step()

	return ToVec3(refPxRigidDynamics[refScene][ref]->getLinearVelocity());
}

EXPORT void setRigidDynamicLinearDamping(uint64_t refScene, uint64_t ref, float v)
{
	//lock_step()

	refPxRigidDynamics[refScene][ref]->setLinearDamping(v);
}

EXPORT void setRigidDynamicAngularDamping(uint64_t refScene, uint64_t ref, float v)
{
	//lock_step()

	refPxRigidDynamics[refScene][ref]->setAngularDamping(v);
}

EXPORT void addRigidDynamicForce(uint64_t refScene, uint64_t ref, APIVec3 v, PxForceMode::Enum forceMode)
{
	//lock_step()

	refPxRigidDynamics[refScene][ref]->addForce(ToPxVec3(v), forceMode);
}

EXPORT void addRigidDynamicTorque(uint64_t refScene, uint64_t ref, APIVec3 v, PxForceMode::Enum forceMode)
{
	//lock_step()

	refPxRigidDynamics[refScene][ref]->addTorque(ToPxVec3(v), forceMode);
}
EXPORT void setRigidDynamicAngularVelocity(uint64_t refScene, uint64_t ref, APIVec3 v)
{
	//lock_step()
	
	refPxRigidDynamics[refScene][ref]->setAngularVelocity(ToPxVec3(v), true);
}


EXPORT void setRigidDynamicMaxLinearVelocity(uint64_t refScene, uint64_t ref, float v)
{
	//lock_step()
	
#if PX_PHYSICS_VERSION_MAJOR==4
	refPxRigidDynamics[refScene][ref]->setMaxLinearVelocity(v);
#endif

}
EXPORT void setRigidDynamicMaxAngularVelocity(uint64_t refScene, uint64_t ref, float v)
{
	//lock_step()
	
	refPxRigidDynamics[refScene][ref]->setMaxAngularVelocity(v);
}
EXPORT void setRigidDynamicWord(uint64_t refScene, uint64_t ref, uint32_t word)
{
    //lock_step()
    
    PxShape* shape;
	refPxRigidDynamics[refScene][ref]->getShapes(&shape, 1);
    
    auto filterData = shape->getSimulationFilterData();
    filterData.word1 = word;
    
    shape->setSimulationFilterData(filterData);
    
}
EXPORT void setRigidDynamicDisable(uint64_t refScene, uint64_t ref, bool disabled)
{
    //lock_step()
    const auto flags = refPxRigidDynamics[refScene][ref]->getActorFlags();
    refPxRigidDynamics[refScene][ref]->setActorFlags(disabled ? flags | PxActorFlag::eDISABLE_SIMULATION :
                                                     flags & ~PxActorFlag::eDISABLE_SIMULATION);
}

// get
EXPORT APITransform getRigidDynamicTransform(uint64_t refScene, uint64_t ref)
{
	//lock_step()
	auto rigid = refPxRigidDynamics[refScene][ref];
	if (rigid == nullptr) {
		debugLogError("nullptr");
		debugLogError(std::to_string(ref).c_str());
		return APITransform();
	}
    return ToTrans(rigid->getGlobalPose());
}


EXPORT APIVec3 getRigidDynamicAngularVelocity(uint64_t refScene, uint64_t ref)
{
	//lock_step()

	return ToVec3(refPxRigidDynamics[refScene][ref]->getAngularVelocity());
}

EXPORT float getRigidDynamicMaxAngularVelocity(uint64_t refScene, uint64_t ref)
{
	//lock_step()
	return refPxRigidDynamics[refScene][ref]->getMaxAngularVelocity();
	

}
EXPORT float getRigidDynamicMaxLinearVelocity(uint64_t refScene, uint64_t ref)
{
	//lock_step()

#if PX_PHYSICS_VERSION_MAJOR==4
	return refPxRigidDynamics[refScene][ref]->getMaxLinearVelocity();
#else
	return 0;
#endif
    
}



// create
EXPORT uint64_t createRigidDynamic(int geoType, int refGeoCount, uint64_t refGeo[], uint64_t refScene, uint64_t refMat, bool kinematic, bool ccd, bool retain, bool disableGravity, bool isTrigger, float mass, unsigned int word, APIVec3 pos, APIQuat quat)
{
	//lock_step()
	
	const auto rigid = gPhysics->createRigidDynamic(PxTransform(ToVec3(pos), ToQuat(quat)));

	const auto insertRef = refOverlaps[refScene]++;
	refPxRigidDynamics[refScene].insert({insertRef, rigid});
	
    rigid->userData = reinterpret_cast<void*>(insertRef);
    
	rigid->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, kinematic);
	if(!kinematic)
		rigid->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, ccd);

    if(disableGravity)
        rigid->setActorFlags(PxActorFlag::Enum::eDISABLE_GRAVITY);
    
	rigid->setMass(mass);

	setupGeometryType(geoType, refGeoCount, refGeo, rigid, *refPxMaterials[refMat]);
	
    
	PxShape** shapesBuffer = new PxShape * [refGeoCount];
    rigid->getShapes(shapesBuffer, refGeoCount);

        
    PxFilterData filterData;
    // word0 - common word
    filterData.word1 = word;
    // word0 - trigger word
    if(isTrigger) filterData.word0 = 1;

    for (PxU32 i = 0; i < refGeoCount; ++i)
    {
        PxShape* shape = shapesBuffer[i];
        
        shape->setSimulationFilterData(filterData);
    }
	delete shapesBuffer;

	refPxScenes[refScene]->addActor(*rigid);

	return insertRef;
}


EXPORT void destroyRigidDynamic(uint64_t refScene, uint64_t ref)
{
	//lock_step()
	
	const auto actor = refPxRigidDynamics[refScene][ref];
	actor->getScene()->removeActor(*actor);
	
	refPxRigidDynamics[refScene][ref]->release();
	refPxRigidDynamics[refScene][ref] = nullptr;
	refPxRigidDynamics[refScene].erase(ref);
}

/// CAPSULE CHARACTER
EXPORT uint64_t createCapsuleCharacter(uint64_t refScene, uint64_t refMat, APIVec3 pos, APIVec3 up, float height, float radius, float stepOffset)
{
	//lock_step()
	const auto insertRef = refOverlaps[refScene]++;
	
	PxCapsuleControllerDesc desc;
	desc.height = height;
	desc.radius = radius;
	desc.position = ToPxVec3d(pos);
	desc.upDirection = ToPxVec3(up);
	desc.stepOffset = stepOffset;
	desc.material = refPxMaterials[refMat];
	
	const auto c = refPxControllerManagers[refScene]->createController(desc);
	c->setUserData(reinterpret_cast<void*>(insertRef));
	c->getActor()->userData = reinterpret_cast<void*>(insertRef);
	
	
	refPxControllers.insert({insertRef, c});
	refControllersDir.insert({insertRef, PxVec3(0, 0, 0)});
	refControllersFlags.insert({ insertRef, PxControllerCollisionFlag::eCOLLISION_SIDES});

	return insertRef;
}
EXPORT void destroyController(uint64_t ref)
{
	//lock_step()
	releaseMap(PxController, ref)
}

// get
EXPORT APIDoubleVec3 getControllerPosition(uint64_t ref)
{
	//lock_step()
	return ToVec3d(refPxControllers[ref]->getPosition());
}
EXPORT APIDoubleVec3 getControllerFootPosition(uint64_t ref)
{
	//lock_step()
	return ToVec3d(refPxControllers[ref]->getFootPosition());
}

// set
EXPORT void setControllerPosition(uint64_t ref, APIDoubleVec3 p)
{
	//lock_step()
	refPxControllers[ref]->setPosition(ToPxVec3d(p));
}

EXPORT void setControllerFootPosition(uint64_t ref, APIDoubleVec3 p)
{
	//lock_step()
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
    
    if((filterData0.word0 == 1 ) || (filterData1.word0 == 1 ))
    {
        pairFlags &= ~PxPairFlag::eSOLVE_CONTACT;
        //pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
    }
    
    if(filterData0.word1 == filterData1.word1)
    {
       pairFlags &= ~PxPairFlag::eSOLVE_CONTACT;
       return PxFilterFlag::eKILL;
    }
    
    
    return PxFilterFlag::eDEFAULT;
}

PxHeightFieldDesc createHeighfieldDesc(const PxI16* heightmap, const uint64_t nbCols, const uint64_t nbRows,
                                       const PxReal thickness = -1, const PxReal convexEdgeThreshold = 0,
                                       const bool noBoundaries = false)
{
	const PxU32 hfNumVerts = nbCols * nbRows;

	const auto samples = static_cast<PxHeightFieldSample*>(platformAlignedAlloc(
		sizeof(PxHeightFieldSample) * hfNumVerts));
	memset(samples, 0, hfNumVerts * sizeof(PxHeightFieldSample));

	for (PxU32 x = 0; x < nbRows; x++)
	{
		for (PxU32 y = 0; y < nbCols; y++)
		{
			samples[x + y * nbRows].height = heightmap[y + x * nbRows];
		}
	}

	PxHeightFieldDesc hfDesc;

	hfDesc.format = PxHeightFieldFormat::eS16_TM;
	hfDesc.nbColumns = nbCols;
	hfDesc.nbRows = nbRows;
	// hfDesc.thickness = thickness;
	hfDesc.convexEdgeThreshold = convexEdgeThreshold;

	if (noBoundaries)
		hfDesc.flags = PxHeightFieldFlag::eNO_BOUNDARY_EDGES;

	hfDesc.samples.data = samples;
	hfDesc.samples.stride = sizeof(PxHeightFieldSample);
	return hfDesc;
}


EXPORT uint64_t createTerrain(PxI16* heightmap, uint64_t hfSize,
							  PxReal thickness, PxReal convexEdgeThreshold, bool noBoundaries,
							  PxReal heightScale, PxReal rowScale, PxReal columnScale,
							  uint64_t refScene, uint64_t refMat, APIVec3 pos)
{
	const auto hfDesc = createHeighfieldDesc(heightmap, hfSize, hfSize, thickness, convexEdgeThreshold, noBoundaries);

	const auto aHeightField = gCooking->createHeightField(hfDesc, gPhysics->getPhysicsInsertionCallback());

	const PxHeightFieldGeometry hfGeom(aHeightField, PxMeshGeometryFlags(), heightScale, rowScale, columnScale);
	const auto rigid = gPhysics->createRigidStatic(PxTransform(ToVec3(pos), PxQuat(PxIdentity)));

	const auto insertRef = refOverlaps[refScene]++;
	refTerrains.insert({insertRef, rigid});
	rigid->userData = reinterpret_cast<void*>(insertRef);

	PxRigidActorExt::createExclusiveShape(*rigid, hfGeom, *refPxMaterials[refMat]);
	refPxScenes[refScene]->addActor(*rigid);

	return insertRef;
}



EXPORT PxReal sampleTerrainHeight(uint64_t ref, APIVec3 position)
{
	PxShape* shape;
	refTerrains[ref]->getShapes(&shape, 1);
	if (shape->getGeometryType() != PxGeometryType::eHEIGHTFIELD) {

		debugLogError("shape geometry is not heightfield");
		return 0;
	}
	const auto geo = shape->getGeometry().heightField();
	
	const auto height = geo.heightField->getHeight(position.x, position.z);

	return height;
}




EXPORT PxU16 sampleTerrainHeightRowCol(uint64_t ref, PxU32 row, PxU32 col)
{
	PxShape* shape;
	refTerrains[ref]->getShapes(&shape, 1);
	if (shape->getGeometryType() != PxGeometryType::eHEIGHTFIELD) {

		debugLogError("shape geometry is not heightfield");
		return 0;
	}
	const auto geo = shape->getGeometry().heightField();

	const auto height = geo.heightField->getSample(row, col).height;

	return height;
}

EXPORT PxReal sampleTerrainHeightNorm(uint64_t ref, APIVec3 normPos)
{
	PxShape* shape;
	refTerrains[ref]->getShapes(&shape, 1);
	if (shape->getGeometryType() != PxGeometryType::eHEIGHTFIELD) {

		debugLogError("shape geometry is not heightfield");
		return 0;
	}
	const auto geo = shape->getGeometry().heightField();

	const auto height = geo.heightField->getSample(static_cast<PxU32>(normPos.x * geo.heightField->getNbRows()),
	                                               static_cast<PxU32>(normPos.z * geo.heightField->getNbColumns())).height;

	return height;
}


EXPORT void modifyTerrain(uint64_t ref, PxI16* heightmap, uint64_t startCol, uint64_t startRow, uint64_t countCols, uint64_t countRows, PxReal heightScale, bool shrinkBounds)
{
	PxShape* shape;
	refTerrains[ref]->getShapes(&shape, 1);
	if (shape->getGeometryType() != PxGeometryType::eHEIGHTFIELD) {

		debugLogError("shape geometry is not heightfield");
		return;
	}
	const auto geo = shape->getGeometry().heightField();

	auto subfieldDesc = createHeighfieldDesc(heightmap, heightScale, countCols, countRows);

	// subfieldDesc.thickness = geo.heightField->getThickness();
	subfieldDesc.format = geo.heightField->getFormat();
	subfieldDesc.flags = geo.heightField->getFlags();

	geo.heightField->modifySamples(startCol, startRow, subfieldDesc, shrinkBounds);
}

/// SCENE
EXPORT uint64_t createScene(APIVec3 gravity, ContactReportCallbackFunc func, TriggerReportCallbackFunc triggerFunc, bool enableCCD, bool enableEnhancedDetermenism)
{
	const auto insertRef = refCountPxScene++;
	
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = ToPxVec3(gravity);
	if (enableEnhancedDetermenism)
		sceneDesc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
	if (enableCCD)
		sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;

	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader = filterShader;

	auto contactReport = std::make_shared<ContactReport>(func, triggerFunc);
    sceneDesc.simulationEventCallback = contactReport.get();

	auto scene = gPhysics->createScene(sceneDesc);

	scene->userData = reinterpret_cast<void*>(insertRef);

	auto controllerManager = PxCreateControllerManager(*scene, false);
	refPxControllerManagers.insert({insertRef, controllerManager});;
	
	refPxScenes.insert({insertRef, scene});
	refContactReports.insert({insertRef, contactReport});
	refOverlaps.insert({ insertRef, 0 });
	refPxRigidDynamics.insert({ insertRef, map<uint64_t, PxRigidDynamic*>() });
	
	PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	return insertRef;
}

EXPORT void stepPhysics(uint64_t ref, float dt)
{
	// lock_step()

    refPxScenes[ref]->simulate(dt);
	refPxScenes[ref]->fetchResults(true);
    

}

PxGeometry getGeometryByType(uint64_t refGeo, int geoType)
{
	
	if (geoType == 1) {
		return *refSharedPxGeometrys[refGeo].get();
	}
	
	if (geoType == 2) {
		PxConvexMeshGeometry geo;
		geo.convexMesh = refPxConvexMeshs[refGeo];
		
		return geo;
	}
	
	if (geoType == 3) {
		PxTriangleMeshGeometry geo;
		geo.triangleMesh = refPxTriangleMeshs[refGeo];
		return geo;
	}

}

EXPORT APIVec4 computePenetration(uint64_t refGeo1, int geoType1, uint16_t refGeo2, int geoType2, APITransform t1, APITransform t2)
{
	
	if (geoType1 != 2) return {0, 0, 0, 0 };

	PxConvexMeshGeometry geom1;
	geom1.convexMesh = refPxConvexMeshs[refGeo1];
	PxVec3 direction;
	PxF32 depth;


	if (geoType2 == 3) // triangle
	{
		PxTriangleMeshGeometry geom2;
		geom2.triangleMesh = refPxTriangleMeshs[refGeo2];

		auto tr1 = ToPxTrans(t1);
		auto tr2 = ToPxTrans(t2);

		bool isPenetrating = PxComputeTriangleMeshPenetration(direction, depth,
			geom1, tr1,
			geom2, tr2, 4);


	}
	else if(geoType2 == 2) // convex
	{
		PxConvexMeshGeometry geom2;
		geom2.convexMesh = refPxConvexMeshs[refGeo2];

		auto tr1 = ToPxTrans(t1);
		auto tr2 = ToPxTrans(t2);
		
		bool isPenetrating = PxGeometryQuery::computePenetration(direction, depth,
			geom1, ToPxTrans(t1),
			geom2, ToPxTrans(t2));
	}



	return { direction.x , direction.y , direction.z , depth };
}

EXPORT void cleanupScene(uint64_t ref)
{
	releaseMap(PxScene, ref)
}
EXPORT uint64_t getSceneTimestamp(uint64_t ref)
{
	return refPxScenes[ref]->getTimestamp();
}

void initLog(DebugLogFunc func, DebugLogErrorFunc func2)
{
	debugLog = func;
	debugLogError = func2;
}

void initPhysics(bool isCreatePvd, int numThreads, float toleranceLength, float toleranceSpeed, ErrorCallbackFunc func)
{
 	debugLog("init physics native library v1.9.9.1.9 controller single update");
	debugLog(std::to_string(PX_PHYSICS_VERSION_MAJOR).c_str());
	debugLog(std::to_string(PX_PHYSICS_VERSION_MINOR).c_str());
	debugLog(std::to_string(PX_PHYSICS_VERSION_BUGFIX).c_str());


	gErrorCallback = std::make_shared<ErrorCallback>(func);

#if PX_PHYSICS_VERSION_MAJOR==3	
	gFoundation = PxCreateFoundation(0x01000000, gAllocator, *gErrorCallback);
#else
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, *gErrorCallback);
#endif

	PxTolerancesScale scale;
	scale.length = toleranceLength;        // typical length of an object
	scale.speed = toleranceSpeed;        // typical speed of an object, gravity*1s is a reasonable choice

#if PX_PHYSICS_VERSION_MAJOR==3
	gCooking = PxCreateCooking(0x01000000, *gFoundation, PxCookingParams(scale));
#else 	
	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(scale));
#endif
	
	if(isCreatePvd)
	{
		gPvd = PxCreatePvd(*gFoundation);
		PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 300);
        gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
	}

#if PX_PHYSICS_VERSION_MAJOR==3
	gPhysics = PxCreatePhysics(0x03040300, *gFoundation, scale,true,gPvd);
#else
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale,true,gPvd);
#endif

#if PX_PHYSICS_VERSION_MAJOR==3
	PxRegisterUnifiedHeightFields(*gPhysics);
#else
	PxRegisterHeightFields(*gPhysics);
#endif
	

	gDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
	
}

EXPORT uint64_t createMaterial(float staticFriction, float dynamicFriction, float restitution)
{
	const auto insertRef = refCountPxMaterial++;
	const auto mat = gPhysics->createMaterial(staticFriction, dynamicFriction, restitution);
	refPxMaterials.insert({ insertRef, mat });
	return insertRef;
}

EXPORT void cleanupMaterial(uint64_t ref)
{
	//lock_step()

	refPxMaterials[ref]->release();
	refPxMaterials[ref] = nullptr;
	refPxMaterials.erase(ref);
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
