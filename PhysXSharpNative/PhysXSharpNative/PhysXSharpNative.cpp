// PhysXSharpNative.cpp : Defines the entry point for the application.
//

#include "PhysXSharpNative.h"

using namespace std;
using namespace physx;

int main()
{
	PxDefaultAllocator		gAllocator;
	PxDefaultErrorCallback	gErrorCallback;
	auto foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	
	cout << "Hello CMake." << endl;
	return 0;
}
