# PhysXSharpNative

![PhysXSharpNative](https://github.com/Team-Iterum/PhysXSharp/workflows/C/C++%20CI/badge.svg)

Build instructions
----------------

### Required software
- Linux / MacOS
- CMake
- g++, clang

### Linux

```bash
cp ../../PhysX/PhysX_3.4/Bin/linux64/libPhysX3CharacterKinematic_x64.so /root/libs  
cp ../../PhysX/PhysX_3.4/Bin/linux64/libPhysX3Common_x64.so             /root/libs  
cp ../../PhysX/PhysX_3.4/Bin/linux64/libPhysX3Cooking_x64.so            /root/libs  
cp ../../PhysX/PhysX_3.4/Bin/linux64/libPhysX3_x64.so                   /root/libs  

cp ../../PhysX/PxShared/bin/linux64/libPxFoundation_x64.so              /root/libs  
cp ../../PhysX/PxShared/bin/linux64/libPxPvdSDK_x64.so                  /root/libs  

cp /root/libs/* /root/deploy
```

*App launch*  

```bash
LD_LIBRARY_PATH=. ./App
```

### MacOS
