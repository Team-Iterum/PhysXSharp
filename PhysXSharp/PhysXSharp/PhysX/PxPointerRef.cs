using System;
using System.Collections.Generic;
using System.Text;

namespace PhysXSharp.PhysX
{
    public abstract class PxPointerRef : IDisposable
    {
        protected IntPtr Ref;
        protected abstract void Release();
        protected abstract void Create();

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        private void Dispose(bool disposing)
        {
            if (disposing)
            {
                // get rid of managed resources
            }

            // get rid of unmanaged resources
            Release();
        }

        // only if you use unmanaged resources directly in B
        ~PxPointerRef()
        {
            Dispose(false);
        }


    }
}
