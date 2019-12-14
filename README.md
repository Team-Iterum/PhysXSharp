Build instructions
===========================

Update git submodule

0) Linux/windows presets set
// --> False
<cmakeSwitch name="PX_BUILDSNIPPETS" value="False" comment="Generate the snippets" />
<cmakeSwitch name="PX_BUILDPUBLICSAMPLES" value="False" comment="Generate the samples projects" />
// --> True
<cmakeSwitch name="PX_GENERATE_STATIC_LIBRARIES" value="True" comment="Generate static libraries" />


Windows
-------
1) generate_projects.bat
2) open physx\compiler\vc16win64\PhysXSDK.sln & build all
3) open PhysXSharpNative in Visual Studio & build all


Linux / WSL
-------
// presets/linux.xml
// - <platform targetPlatform="linux" compiler="clang" />
// + <platform targetPlatform="linux" compiler="gcc" />

1) generate_projects.sh
2) cd PhysX/physx/compiler/linux-release && make

3) cd PhysXSharpNative && cmake && make
or
3) open PhysXSharpNative in Visual Studio & build all