// PhysXSharpNative.h : Include file for standard system include files,
// or project specific include files.

#pragma once


#define NDEBUG

#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <utility>
#include "PxPhysicsAPI.h"

#if defined(_MSC_VER)
    // #define PX_DEBUG 1
    // #define PX_CHECKED 1
#endif

#if defined(_MSC_VER)
    //  Microsoft
    #define EXPORT extern "C" __declspec(dllexport)
#else
	#define EXPORT extern "C" __attribute__((visibility("default")))
#endif


struct APIVec3
{
    float x;
    float y;
    float z;
};

struct APIDoubleVec3
{
    double x;
    double y;
    double z;
};

struct APIQuat
{
    float x;
    float y;
    float z;
    float w;
};

struct APITransform
{
    APIQuat q;
    APIVec3 p;
};

typedef void (*OverlapCallback)(int index, int64_t t1);
typedef void (*RaycastCallback)(int index, int64_t t1);
typedef void (*ErrorCallbackFunc)(const char* message);
typedef void (*DebugLogFunc)(const char* message);
typedef void (*DebugLogErrorFunc)(const char* message);
typedef void (*ContactReportCallbackFunc)(const int64_t ref0, const int64_t ref1, APIVec3 normal, APIVec3 position, APIVec3 impulse, float separation);
typedef void (*TriggerReportCallbackFunc)(const int64_t ref0, const int64_t ref1);

#define ToPxVec3(v) physx::PxVec3(v.x, v.y, v.z)
#define ToPxVec3d(v) physx::PxExtendedVec3(v.x, v.y, v.z)
#define ToPxQuat(q) physx::PxQuat(q.x, q.y, q.z, q.w)
#define ToPxTrans(t) PxTransform(ToPxVec3(t.p), ToPxQuat(t.q))
#define ToVec3(v) { v.x, v.y, v.z }
#define ToVec3d(v) { v.x, v.y, v.z }
#define ToQuat(q) { q.x, q.y, q.z, q.w }
#define ToTrans(t) { ToQuat(t.q), ToVec3(t.p) }

#define PX_RELEASE(x) if(x) { x->release(); x = nullptr; }
#define refMap(x) map<int64_t, x*> ref##x##s; int64_t refCount##x;
#define refMapNonPtr(x) map<int64_t, x> ref##x##s; int64_t refCount##x;

#define insertMapNoUserData(x, y) const auto insertRef = refCount##x++; \
							      ref##x##s.insert({insertRef, y});

#define releaseMap(x, ref) ref##x##s[ref]->release(); \
						   ref##x##s[ref] = nullptr; ref##x##s.erase(ref);

DebugLogFunc debugLog;
DebugLogErrorFunc debugLogError;

#if defined(_MSC_VER)
    void AssertError(const char* exp, const char* file, int line, bool& ignoreError)
    {
        std::stringstream oss;
        oss << "Assert: " << exp;
        oss << " " << file << ":" << std::to_string(line);

        debugLogError(oss.str().c_str());
    }

    #define PXS_ASSERT(exp)                                                                                  \
    {                                                                                                        \
        static bool _ignore = false;                                                                         \
        ((void)((!!(exp)) || (!_ignore && (AssertError(#exp, __FILE__, __LINE__, _ignore), false))));        \
        __analysis_assume(!!(exp))                                                                           \
    }

#else
    #define PXS_ASSERT(exp) {}
#endif


typedef physx::PxOverlapBufferN<1000> OverlapBuffer1000;
typedef physx::PxRaycastBufferN<10> RaycastBuffer10;

typedef std::shared_ptr<OverlapBuffer1000> OverlapBuffer;
typedef std::shared_ptr<RaycastBuffer10> RaycastBuffer;

typedef std::shared_ptr<physx::PxGeometry> SharedPxGeometry;

class ErrorCallback final : public physx::PxErrorCallback
{
private:
	ErrorCallbackFunc callback;
public:
	explicit ErrorCallback(ErrorCallbackFunc func)
	{
		callback = func;
	}

	void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line) override
	{
		callback(message);
	}
};

struct FilterGroup
{
    enum Enum
    {
        eOBJECT        = (1 << 0),
        eTRIGGER       = (1 << 1),
    };
};

class ContactReport final : public physx::PxSimulationEventCallback
{
private:
    ContactReportCallbackFunc callback;
    TriggerReportCallbackFunc triggerCallback;
public:
    explicit ContactReport(ContactReportCallbackFunc func, TriggerReportCallbackFunc triggerFunc)
    {
        callback = func;
        triggerCallback = triggerFunc;
    }
public:
    void onContact(const physx::PxContactPairHeader &pairHeader,
                   const physx::PxContactPair *pairs,
                   physx::PxU32 nbPairs) override
    {

        if(pairHeader.actors[0] != pairHeader.actors[1])
        {
            int64_t ref0 = reinterpret_cast<const int64_t>(pairHeader.actors[0]->userData);
            int64_t ref1 = reinterpret_cast<const int64_t>(pairHeader.actors[1]->userData);

            bool isTrigger = false;
            // Contact information
            APIVec3 normal = {0, 0, 0};
            APIVec3 position = {0, 0, 0};
            APIVec3 impulse = {0, 0, 0};
            float separation = 100;


            const physx::PxU32 bufferSize = 32;
            physx::PxContactPairPoint contacts[bufferSize];


            for (physx::PxU32 i = 0; i < nbPairs; i++)
            {
                const physx::PxU32 nbContacts = pairs[i].contactCount;

                if(nbContacts == 0) continue;

                pairs[i].extractContacts(contacts, bufferSize);

                auto shape0 = pairs[i].shapes[0];
                auto shape1 = pairs[i].shapes[1];
                
                auto actor0 = shape0->getActor();
                auto actor1 = shape1->getActor();
                ref0 = reinterpret_cast<const int64_t>(actor0->userData);
                ref1 = reinterpret_cast<const int64_t>(actor1->userData);
                
                if((shape0->getSimulationFilterData().word0 == 1) ||
                   (shape1->getSimulationFilterData().word0 == 1))
                {
                    isTrigger = true;
                }   
                

                for (int j = 0; j < nbContacts; j++)
                {
                    if(separation > contacts[j].separation)
                    {
                        normal = ToVec3(contacts[j].normal);
                        position = ToVec3(contacts[j].position);
                        impulse = ToVec3(contacts[j].impulse);
                        separation = contacts[j].separation;
                    }
                }
            }

            if(isTrigger)
            {
                triggerCallback(ref0, ref1);
            }
            else
            {
                callback(ref0, ref1, normal, position, impulse, separation);
            }
        }
    }

    void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count)	override { PX_UNUSED(constraints); PX_UNUSED(count); }
    void onWake(physx::PxActor** actors, physx::PxU32 count)							override { PX_UNUSED(actors); PX_UNUSED(count); }
    void onSleep(physx::PxActor** actors, physx::PxU32 count)							override { PX_UNUSED(actors); PX_UNUSED(count); }
    
    void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count)                     override { PX_UNUSED(pairs); PX_UNUSED(count); }
    void onAdvance(const physx::PxRigidBody*const*, const physx::PxTransform*, const physx::PxU32) override {}
};


void createBV33TriangleMesh(const char* name, physx::PxU32 numVertices, const physx::PxVec3* vertices, physx::PxU32 numTriangles, const physx::PxU32* indices);

void createTriangleMesh(const char* name, physx::PxVec3 vertices[], int pointsCount, uint32_t indices[], int triCount);

EXPORT void initLog(DebugLogFunc func, DebugLogErrorFunc func2);

EXPORT void initPhysics(bool isCreatePvd, int numThreads, float toleranceLength, float toleranceSpeed, ErrorCallbackFunc func);

EXPORT int64_t loadTriangleMesh(const char* name);
