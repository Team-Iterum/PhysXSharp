Prebuild instructions
----------------

#### Software
- Windows 10 with WSL
- Visual Studio *(for PhysX build)*
- CLion *(for WSL and Windows library build)*

#### PhysX prepare

1. Update `git submodule`
2. Linux/windows presets changes
- PX_BUILDSNIPPETS=False
- PX_BUILDPUBLICSAMPLES=False
```xml
<cmakeSwitch name="PX_BUILDSNIPPETS" value="False" comment="Generate the snippets" />
<cmakeSwitch name="PX_BUILDPUBLICSAMPLES" value="False" comment="Generate the samples projects" />
```
- PX_GENERATE_STATIC_LIBRARIES = False
```xml
<cmakeSwitch name="PX_GENERATE_STATIC_LIBRARIES" value="False" comment="Generate static libraries" />
```
#### CLion configuration
1. Add WSL Toolchain and configure connection to WSL
2. Add VisualStudio **(amd64)** toolchain
3. Add CMake configrations:
    + *Release-WSL*
    + *Debug-Visual Studio*
    + *Release-Visual Studio*
 
Windows Build
-----------------
1) Run `generate_projects.bat`
2) Open in **Visual Studio** `physx\compiler\vc16win64\PhysXSDK.sln` & build all
3) Open `PhysXSharpNative` in **CLion** and run build configurations
*Debug-Visual Studio** and *Release-Visual Studio* 

WSL Build
-------
```
// presets/linux.xml
// - <platform targetPlatform="linux" compiler="clang" />
// + <platform targetPlatform="linux" compiler="gcc" />
```

1) Run `generate_projects.sh`
2) Run `cd PhysX/physx/compiler/linux-release && make`
3) **IMPORTANT** Copy physx compiled shared and static libraries to `/usr/sbin`
3) Open `PhysXSharpNative` in **CLion** and run build configuration *Release-WSL* 