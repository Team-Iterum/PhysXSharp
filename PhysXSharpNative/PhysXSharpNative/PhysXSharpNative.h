// PhysXSharpNative.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#define PX_DEBUG 1
#define PX_CHECKED 1



#include <memory>
#include "PxPhysicsAPI.h"



// TODO: Reference additional headers your program requires here.

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

#define insertMap(x, y) const auto insertRef = refCount##x++; \
						ref##x##s.insert({insertRef, y}); \
						y->userData = reinterpret_cast<void*>(insertRef);

#define insertMapNoUserData(x, y) const auto insertRef = refCount##x++; \
							      ref##x##s.insert({insertRef, y});

#define releaseMap(x, ref) ref##x##s[ref]->release(); \
						   ref##x##s[ref] = nullptr; ref##x##s.erase(ref);




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
	~ErrorCallback() = default;;

	virtual void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line) override
	{
		callback(message);
	}
};

