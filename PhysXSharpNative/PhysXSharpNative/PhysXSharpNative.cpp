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

uint64_t refOverlap;
map<uint64_t, PxController*> refPxControllers;
map<uint64_t, PxVec3> refControllersDir;

map<uint64_t, PxRigidStatic*> refPxRigidStatics;
map<uint64_t, PxRigidDynamic*> refPxRigidDynamics;

refMap(PxScene)
map<uint64_t, std::shared_ptr<ContactReport>> refContactReports;

refMap(PxTriangleMesh)
refMap(PxConvexMesh)

refMapNonPtr(RaycastBuffer)
refMapNonPtr(ptrOverlapBuffer1000)
refMapNonPtr(ptrOverlapBuffer10)
refMapNonPtr(ptrOverlapBuffer1)
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


EXPORT void characterUpdate(uint64_t ref, float elapsed, float minDist)
{
	lock_step()
	refPxControllers[ref]->move(refControllersDir[ref], minDist, elapsed, PxControllerFilters());
}


EXPORT void charactersUpdate(uint64_t refScene, float elapsed, float minDist)
{
	lock_step()
	auto controllers = refPxControllerManagers[refScene];
	for (size_t i = 0; i < controllers->getNbControllers(); i++)
	{
		auto controller = controllers->getController(i);
		const auto ref = reinterpret_cast<uint64_t>(controller->getUserData());

		controller->move(refControllersDir[ref], minDist, elapsed, PxControllerFilters());
	}
}


EXPORT void setControllerDirection(uint64_t ref, APIVec3 dir)
{
	refControllersDir[ref] = ToPxVec3(dir);
}


EXPORT uint64_t createRaycastBuffer10()
{
    lock_step()
    const auto insertRef = refCountRaycastBuffer++;

    RaycastBuffer buffer = std::make_shared<RaycastBuffer10>();
    
    refRaycastBuffers.insert({insertRef, buffer});
    
    return insertRef;
}



EXPORT uint64_t createOverlapBuffer1000()
{
    lock_step()
    const auto insertRef = refCountptrOverlapBuffer1000++;

    ptrOverlapBuffer1000 buffer = std::make_shared<OverlapBuffer1000>();
    
    refptrOverlapBuffer1000s.insert({insertRef, buffer});
    
    return insertRef;

}

EXPORT uint64_t createOverlapBuffer1()
{
    lock_step()
    const auto insertRef = refCountptrOverlapBuffer1++;

    ptrOverlapBuffer1 buffer = std::make_shared<OverlapBuffer1>();
    
    refptrOverlapBuffer1s.insert({insertRef, buffer});
    
    return insertRef;

}


EXPORT uint64_t createOverlapBuffer10()
{
    lock_step()
    const auto insertRef = refCountptrOverlapBuffer10++;

    ptrOverlapBuffer10 buffer = std::make_shared<OverlapBuffer10>();
    
    refptrOverlapBuffer10s.insert({insertRef, buffer});
    
    return insertRef;

}


EXPORT int sceneRaycast(uint64_t refScene, uint64_t refRaycastBuffer, APIVec3 origin, APIVec3 unitDir, float distance, RaycastCallback callback)
{
    lock_step()
    
    auto buffer = *refRaycastBuffers[refRaycastBuffer];
    refPxScenes[refScene]->raycast(ToVec3(origin), ToVec3(unitDir), distance, buffer);
    
    for (PxU32 i = 0; i < buffer.nbTouches; ++i)
    {
        const auto touch = buffer.touches[i];
        const auto ref = reinterpret_cast<uint64_t>(touch.actor->userData);
        
        callback(i, ref);
    }

    return buffer.nbTouches;
}


EXPORT int sceneOverlap1000(uint64_t refScene, uint64_t refBuffer, uint64_t refGeo, APIVec3 pos, OverlapCallback callback)
{
	lock_step()

    auto buffer = *refptrOverlapBuffer1000s[refBuffer];

    refPxScenes[refScene]->overlap(*refSharedPxGeometrys[refGeo], PxTransform(ToPxVec3(pos)), buffer);
   

    for (PxU32 i = 0; i < buffer.nbTouches; ++i)
    {
        const auto touch = buffer.touches[i];
        const auto ref = reinterpret_cast<uint64_t>(touch.actor->userData);
        
        callback(i, ref);
    }

    return buffer.nbTouches;
    
}

EXPORT int sceneOverlap10(uint64_t refScene, uint64_t refBuffer, uint64_t refGeo, APIVec3 pos, OverlapCallback callback)
{
    lock_step()
    
    auto buffer = *refptrOverlapBuffer10s[refBuffer];
    
    refPxScenes[refScene]->overlap(*refSharedPxGeometrys[refGeo], PxTransform(ToPxVec3(pos)), buffer);
    
    for (PxU32 i = 0; i < buffer.nbTouches; ++i)
    {
        const auto touch = buffer.touches[i];
        const auto ref = reinterpret_cast<uint64_t>(touch.actor->userData);
        
        callback(i, ref);
    }

    return buffer.nbTouches;
    
}


EXPORT int sceneOverlap1(uint64_t refScene, uint64_t refBuffer, uint64_t refGeo, APIVec3 pos, OverlapCallback callback)
{
    lock_step()
    
    auto buffer = *refptrOverlapBuffer1s[refBuffer];
    
    refPxScenes[refScene]->overlap(*refSharedPxGeometrys[refGeo], PxTransform(ToPxVec3(pos)), buffer);
    
    for (PxU32 i = 0; i < buffer.nbTouches; ++i)
    {
        const auto touch = buffer.touches[i];
        const auto ref = reinterpret_cast<uint64_t>(touch.actor->userData);
        
        callback(i, ref);
    }

    return buffer.nbTouches;
    
}


EXPORT uint64_t createSphereGeometry(float radius)
{
	const SharedPxGeometry geo = std::make_shared<PxSphereGeometry>(radius);
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
    params.midphaseDesc.mBVH34Desc.numTrisPerLeaf = numTrisPerLeaf;

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
    createBV34TriangleMesh(name, pointsCount, vertices, triCount, indices, false, false, 4);
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
	lock_step()

	const auto verticesPx = reinterpret_cast<PxVec3*>(vertices);
	return createConvexMesh<PxConvexMeshCookingType::eQUICKHULL, true, 256>(pointsCount, verticesPx);

}
EXPORT void cleanupTriangleMesh(uint64_t ref)
{
	lock_step()
	
	refPxTriangleMeshs[ref]->release();
	refPxTriangleMeshs[ref] = nullptr;
	refPxTriangleMeshs.erase(ref);
}

EXPORT void cleanupConvexMesh(uint64_t ref)
{
	lock_step()
	
	refPxConvexMeshs[ref]->release();
	refPxConvexMeshs[ref] = nullptr;
	refPxConvexMeshs.erase(ref);
}

EXPORT uint64_t createBoxGeometry(APIVec3 half)
{
	lock_step()

	const auto geo = std::make_shared<PxBoxGeometry>(ToPxVec3(half));
	const auto insertRef = refCountSharedPxGeometry++;
  	refSharedPxGeometrys.insert({insertRef, geo});

	return insertRef;
}
EXPORT void cleanupGeometry(uint64_t ref)
{
	lock_step()
	refSharedPxGeometrys.erase(ref);
}

void setupGeometryType(int type, int refGeoCount, uint64_t refGeo[], PxRigidActor* rigid)
{
    for (int i = 0; i < refGeoCount; i++)
    {
        if(type == 1)
        {
            PxRigidActorExt::createExclusiveShape(*rigid, *refSharedPxGeometrys[refGeo[i]], *gMaterial);
        }
        else if(type == 2)
        {
            PxConvexMeshGeometry geo;
            geo.convexMesh = refPxConvexMeshs[refGeo[i]];
            
            PxRigidActorExt::createExclusiveShape(*rigid, geo, *gMaterial);
        }
        else if(type == 3)
        {
            PxTriangleMeshGeometry geo;
            geo.triangleMesh = refPxTriangleMeshs[refGeo[i]];
            
            PxRigidActorExt::createExclusiveShape(*rigid, geo, *gMaterial);
            
        }
    }
}

/// RIGID STATIC
// create
EXPORT uint64_t createRigidStatic(int geoType, uint64_t refGeo, uint64_t refScene, APIVec3 pos, APIQuat quat, bool isTrigger)
{
	lock_step()

	const auto rigid = gPhysics->createRigidStatic(PxTransform(ToVec3(pos), ToQuat(quat)));
	
	
	const auto insertRef = refOverlap++;
	refPxRigidStatics.insert({insertRef, rigid});
	rigid->userData = reinterpret_cast<void*>(insertRef);

    uint64_t refGeoArr[] = { refGeo };
	setupGeometryType(geoType, 1, refGeoArr, rigid);
	
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
	lock_step()
	
	const auto actor = refPxRigidStatics[ref];
	actor->getScene()->removeActor(*actor);
	
	refPxRigidStatics[ref]->release();
	refPxRigidStatics[ref] = nullptr;
	refPxRigidStatics.erase(ref);
}


// get
EXPORT APIVec3 getRigidStaticPosition(uint64_t ref)
{
	lock_step()
	
 	return ToVec3(refPxRigidStatics[ref]->getGlobalPose().p);
}
EXPORT APIQuat getRigidStaticRotation(uint64_t ref)
{
	lock_step()
	
	return ToQuat(refPxRigidStatics[ref]->getGlobalPose().q);
}


// set
EXPORT void setRigidStaticPosition(uint64_t ref, APIVec3 p)
{
	lock_step()
	
	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToPxVec3(p)));
}
EXPORT void setRigidStaticRotation(uint64_t ref, APIQuat q)
{
	lock_step()
	
	refPxRigidStatics[ref]->setGlobalPose(PxTransform(ToQuat(q)));	
}

/// RIGID DYNAMIC
// set
EXPORT void setRigidDynamicTransform(uint64_t ref, APITransform t)
{
	lock_step()

	refPxRigidDynamics[ref]->setGlobalPose(ToPxTrans(t));
}


EXPORT void setRigidDynamicKinematicTarget(uint64_t ref, APITransform t)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setKinematicTarget(ToPxTrans(t));
}


EXPORT void setRigidDynamicLinearVelocity(uint64_t ref, APIVec3 v)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setLinearVelocity(ToPxVec3(v), true);
}

EXPORT APIVec3 getRigidDynamicLinearVelocity(uint64_t ref)
{
	lock_step()

	return ToVec3(refPxRigidDynamics[ref]->getLinearVelocity());
}

EXPORT void setRigidDynamicLinearDamping(uint64_t ref, float v)
{
	lock_step()

	refPxRigidDynamics[ref]->setLinearDamping(v);
}

EXPORT void setRigidDynamicAngularDamping(uint64_t ref, float v)
{
	lock_step()

	refPxRigidDynamics[ref]->setAngularDamping(v);
}

EXPORT void addRigidDynamicForce(uint64_t ref, APIVec3 v, PxForceMode::Enum forceMode)
{
	lock_step()

	refPxRigidDynamics[ref]->addForce(ToPxVec3(v), forceMode);
}

EXPORT void addRigidDynamicTorque(uint64_t ref, APIVec3 v, PxForceMode::Enum forceMode)
{
	lock_step()

	refPxRigidDynamics[ref]->addTorque(ToPxVec3(v), forceMode);
}
EXPORT void setRigidDynamicAngularVelocity(uint64_t ref, APIVec3 v)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setAngularVelocity(ToPxVec3(v), true);
}


EXPORT void setRigidDynamicMaxLinearVelocity(uint64_t ref, float v)
{
	lock_step()
	
	//refPxRigidDynamics[ref]->setMaxLinearVelocity(v);
}
EXPORT void setRigidDynamicMaxAngularVelocity(uint64_t ref, float v)
{
	lock_step()
	
	refPxRigidDynamics[ref]->setMaxAngularVelocity(v);
}
EXPORT void setRigidDynamicWord(uint64_t ref, uint32_t word)
{
    lock_step()
    
    PxShape* shape;
    refPxRigidDynamics[ref]->getShapes(&shape, 1);
    
    auto filterData = shape->getSimulationFilterData();
    filterData.word1 = word;
    
    shape->setSimulationFilterData(filterData);
    
}
EXPORT void setRigidDynamicDisable(uint64_t ref, bool disabled)
{
    lock_step()
    const auto flags = refPxRigidDynamics[ref]->getActorFlags();
    refPxRigidDynamics[ref]->setActorFlags(disabled ? flags | PxActorFlag::eDISABLE_SIMULATION :
                                                     flags & ~PxActorFlag::eDISABLE_SIMULATION);
}

// get
EXPORT APITransform getRigidDynamicTransform(uint64_t ref)
{
	lock_step()
    return ToTrans(refPxRigidDynamics[ref]->getGlobalPose());
}


EXPORT APIVec3 getRigidDynamicAngularVelocity(uint64_t ref)
{
	lock_step()

	return ToVec3(refPxRigidDynamics[ref]->getAngularVelocity());
}

EXPORT float getRigidDynamicMaxAngularVelocity(uint64_t ref)
{
	lock_step()
	return refPxRigidDynamics[ref]->getMaxAngularVelocity();
}
EXPORT float getRigidDynamicMaxLinearVelocity(uint64_t ref)
{
	lock_step()

    return 0;//refPxRigidDynamics[ref]->getMaxLinearVelocity();
}



// create
EXPORT uint64_t createRigidDynamic(int geoType, int refGeoCount, uint64_t refGeo[], uint64_t refScene, bool kinematic, bool ccd, bool retain, bool disableGravity, bool isTrigger, float mass, unsigned int word, APIVec3 pos, APIQuat quat)
{
	lock_step()
	
	const auto rigid = gPhysics->createRigidDynamic(PxTransform(ToVec3(pos), ToQuat(quat)));

	const auto insertRef = refOverlap++;
	refPxRigidDynamics.insert({insertRef, rigid});
	
    rigid->userData = reinterpret_cast<void*>(insertRef);
    
	rigid->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, kinematic);
	if(!kinematic)
		rigid->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, ccd);

    if(disableGravity)
        rigid->setActorFlags(PxActorFlag::Enum::eDISABLE_GRAVITY);
    
	rigid->setMass(mass);

	setupGeometryType(geoType, refGeoCount, refGeo, rigid);
	
    
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


EXPORT void destroyRigidDynamic(uint64_t ref)
{
	lock_step()
	
	const auto actor = refPxRigidDynamics[ref];
	actor->getScene()->removeActor(*actor);
	
	refPxRigidDynamics[ref]->release();
	refPxRigidDynamics[ref] = nullptr;
	refPxRigidDynamics.erase(ref);
}

/// CAPSULE CHARACTER
EXPORT uint64_t createCapsuleCharacter(uint64_t refScene, APIVec3 pos, APIVec3 up, float height, float radius, float stepOffset)
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
EXPORT void destroyController(uint64_t ref)
{
	lock_step()
	releaseMap(PxController, ref)
}

// get
EXPORT APIDoubleVec3 getControllerPosition(uint64_t ref)
{
	lock_step()
	return ToVec3d(refPxControllers[ref]->getPosition());
}
EXPORT APIDoubleVec3 getControllerFootPosition(uint64_t ref)
{
	lock_step()
	return ToVec3d(refPxControllers[ref]->getFootPosition());
}

// set
EXPORT void setControllerPosition(uint64_t ref, APIDoubleVec3 p)
{
	lock_step()
	refPxControllers[ref]->setPosition(ToPxVec3d(p));
}

EXPORT void setControllerFootPosition(uint64_t ref, APIDoubleVec3 p)
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


/// SCENE
EXPORT uint64_t createScene(APIVec3 gravity, ContactReportCallbackFunc func, TriggerReportCallbackFunc triggerFunc)
{
	const auto insertRef = refCountPxScene++;
	
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = ToPxVec3(gravity);
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader = filterShader;

	auto contactReport = std::make_shared<ContactReport>(func, triggerFunc);
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

EXPORT void stepPhysics(uint64_t ref, float dt)
{
	lock_step()

    refPxScenes[ref]->simulate(dt);
	refPxScenes[ref]->fetchResults(true);
    
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
 	debugLog("init physics native library v1.6.2 buffers");

	gErrorCallback = std::make_shared<ErrorCallback>(func);
	
	gFoundation = PxCreateFoundation(0x01000000, gAllocator, *gErrorCallback);

	PxTolerancesScale scale;
	scale.length = toleranceLength;        // typical length of an object
	scale.speed = toleranceSpeed;        // typical speed of an object, gravity*1s is a reasonable choice

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(scale));
		
	if(isCreatePvd)
	{
		gPvd = PxCreatePvd(*gFoundation);
		PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 300);
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
