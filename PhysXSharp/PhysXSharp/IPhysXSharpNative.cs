using System;

namespace PhysXSharp
{
    public delegate void ErrorCallbackFunc(string message, string file, int line);
    public interface IPhysXSharpNative
    {
        
        IntPtr CreateVec3(float x, float y, float z);
        void SetVec3(IntPtr vec3, float x, float y, float z);

        IntPtr CreateErrorCallback(ErrorCallbackFunc func);
        IntPtr CreateDefaultAllocator();
        IntPtr CreateFoundation(IntPtr errorCallback, IntPtr allocator);
        IntPtr CreatePvd(IntPtr foundation, string host);
        IntPtr CreatePhysics(IntPtr foundation, IntPtr pvd);
        IntPtr CreateSceneDesc(IntPtr physics, IntPtr gravity, IntPtr cpuDispatcher);
        IntPtr CreateScene(IntPtr physics, IntPtr desc);
        IntPtr CreateCpuDispatcher(int numThreads);
        IntPtr CreateMaterial(IntPtr physics, float staticFriction, float dynamicFriction, float restitution);
    }
}
