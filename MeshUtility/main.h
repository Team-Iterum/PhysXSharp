#pragma once

#include <iostream>
#include <sstream>
#include <vector>

#include <PhysXSharpNative.h>

inline std::string separator()
{
#ifdef _WIN32
    return std::string("\\");
#else
    return std::string("/");
#endif
}

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
    str << "MeshData" << "/" << std::to_string(meshId) << ".mesh";
    return str.str();
}

bool endsWithSubstring(
    const std::string& str,
    const std::string& subStr) {
    size_t strLen = str.length();
    size_t subStrLen = subStr.length();
    // Use the compare() member function to
    // check if the string ends with the substring
    return  strLen >= subStrLen &&
        str.compare(strLen - subStrLen, subStrLen, subStr) == 0;
}

