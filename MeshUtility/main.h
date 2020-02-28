#pragma once

#include <iostream>
#include <sstream>
#include <vector>

#include <PhysXSharpNative.h>

typedef std::vector<physx::PxVec3> VertList;
typedef std::vector<unsigned int> IndList;

physx::PxVec3 getVec3(const std::string& s);

void logCritical(const char* msg)
{
    std::cout << "[Critical] " << msg << std::endl;
}
void logDebug(const char* msg)
{
    std::cout << "[Debug] " << msg << std::endl;
}
void logError(const char* msg)
{
    std::cout << "[Error] " << msg << std::endl;
}


std::string getMeshName(unsigned int meshId)
{
    std::stringstream str;
    str << "Meshes" << "/" << std::to_string(meshId) << ".mesh";
    return str.str();
}

std::string getMeshDataName(unsigned int meshId)
{
    std::stringstream str;
    str << "MeshesData" << "/" << std::to_string(meshId) << ".txt";
    return str.str();
}

