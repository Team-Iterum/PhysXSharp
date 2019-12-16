using System;
// ReSharper disable InconsistentNaming

namespace PhysXSharp.PhysX
{
    public sealed class PxVec3 : PxPointerRef
    {
        private IPhysXSharpNative native;

        public float x { get; set; }
        public float y { get; set; }
        public float z { get; set; }

        public PxVec3()
        {
            x = y = z = 0;
            Create();
        }
        
        public PxVec3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;

            Create();
        }

        public void Update()
        {
            native.SetVec3(Ref, x, y, z);
        }

        protected override void Release()
        {
            //native.ReleaseVec3(Ref);
        }

        protected override void Create()
        {
            Ref = native.CreateVec3(x, y, z);   
        }
    }
}
