Build instructions
----------------

### Required software
- Linux / MacOS / Windows 10 with WSL
- cmake

### PhysX prepare

1. Update `git submodule`
2. Required presets changes
```xml
<cmakeSwitch name="PX_BUILDSNIPPETS" value="False" comment="Generate the snippets" />
<cmakeSwitch name="PX_BUILDPUBLICSAMPLES" value="False" comment="Generate the samples projects" />

<cmakeSwitch name="PX_GENERATE_STATIC_LIBRARIES" value="False" comment="Generate static libraries" />
```

### WSL

1) Run `generate_projects.sh`
2) Run `cd PhysX/physx/compiler/linux-release && make`
3) **IMPORTANT** Copy physx compiled shared and static libraries to `/usr/sbin`
3) Open `PhysXSharpNative` in **CLion** and run build configuration *Release-WSL* 

### Linux

### MacOS
