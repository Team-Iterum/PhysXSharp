// PhysXSharpNative.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <memory>
#include "PxPhysicsAPI.h"

// TODO: Reference additional headers your program requires here.

#if defined(_MSC_VER)
    //  Microsoft 
    #define EXPORT extern "C" __declspec(dllexport)
#elif defined(__GNUC__)
    //  GCC
    #define EXPORT extern "C" __attribute__((visibility("default")))
#else
    //  do nothing and hope for the best?
    #define EXPORT
#endif


typedef void (*ErrorCallbackFunc)(const char* message, const char* file, int line);

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
		callback(message, file, line);
	}
};

