// PhysXSharpNative.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <utility>
#include "PxPhysicsAPI.h"

#if defined(_MSC_VER)
    #define PX_DEBUG 1
    #define PX_CHECKED 1
#endif

#if defined(_MSC_VER)
    //  Microsoft
    #define EXPORT extern "C" __declspec(dllexport)
#else
	#define EXPORT extern "C" __attribute__((visibility("default")))
#endif

typedef void (*OverlapCallback)(long t1);
typedef void (*ErrorCallbackFunc)(const char* message);
typedef void (*DebugLogFunc)(const char* message);
typedef void (*DebugLogErrorFunc)(const char* message);
typedef void (*ContactReportCallbackFunc)(const long ref0, const long ref1);


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

#define ToPxVec3(v) physx::PxVec3(v.x, v.y, v.z)
#define ToPxVec3d(v) physx::PxExtendedVec3(v.x, v.y, v.z)
#define ToPxQuat(q) physx::PxQuat(q.x, q.y, q.z, q.w)

#define ToVec3(v) { v.x, v.y, v.z }
#define ToVec3d(v) { v.x, v.y, v.z }
#define ToQuat(q) { q.x, q.y, q.z, q.w }

#define PX_RELEASE(x) if(x) { x->release(); x = nullptr; }
#define refMap(x) map<long, x*> ref##x##s; long refCount##x;
#define refMapNonPtr(x) map<long, x> ref##x##s; long refCount##x;

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


typedef physx::PxOverlapBuffer OverlapBuffer;

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

class ContactReport final : public physx::PxSimulationEventCallback
{
private:
    ContactReportCallbackFunc callback;
public:
    explicit ContactReport(ContactReportCallbackFunc func)
    {
        callback = func;
    }
public:
    void onContact(const physx::PxContactPairHeader &pairHeader,
                   const physx::PxContactPair *pairs,
                   physx::PxU32 nbPairs) override
    {
        PX_UNUSED(pairs);
        PX_UNUSED(nbPairs);

        if(pairHeader.actors[0] != pairHeader.actors[1])
        {
            const long ref0 = reinterpret_cast<const long>(pairHeader.actors[0]->userData);
            const long ref1 = reinterpret_cast<const long>(pairHeader.actors[1]->userData);

            callback(ref0, ref1);
        }
    }

    void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count)	override { PX_UNUSED(constraints); PX_UNUSED(count); }
    void onWake(physx::PxActor** actors, physx::PxU32 count)							override { PX_UNUSED(actors); PX_UNUSED(count); }
    void onSleep(physx::PxActor** actors, physx::PxU32 count)							override { PX_UNUSED(actors); PX_UNUSED(count); }
    void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count)					    override { PX_UNUSED(pairs); PX_UNUSED(count); }
    void onAdvance(const physx::PxRigidBody*const*, const physx::PxTransform*, const physx::PxU32) override {}
};

