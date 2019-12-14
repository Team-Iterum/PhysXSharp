// PhysXSharpNative.cpp : Defines the entry point for the application.
//

#include "PhysXSharpNative.h"

using namespace std;
using namespace physx;

int main()
{
	PxDefaultAllocator		gAllocator;
	PxDefaultErrorCallback	gErrorCallback;
	const auto foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	
	cout << "Hello CMake." << foundation << endl;
	return 0;
}
