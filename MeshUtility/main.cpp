#include "main.h"

#include <fstream>
#include <string>

using namespace std;

int main (int argc, char *argv[])
{
    cout << "PhysXSharpNative Mesh Utility 1.0" << endl;
    
    const int numThreads = 2;
    const float toleranceLength = 1;
    const float toleranceSpeed = 5;
    
    initLog(logDebug, logError);
    initPhysics(false, numThreads, toleranceLength, toleranceSpeed, logCritical);
    
    for (unsigned int i = 0; i <= 62; i++)
    {
        auto vertices = make_shared<VertList>();
        auto indices = make_shared<IndList>();
        
        const auto meshDataName = getMeshDataName(i);
                
        std::ifstream file(meshDataName);
        std::string str;
        
        while (getline(file, str))
        {
            // skip comments
            if(str.substr(2) == "//") continue;
            
            // vertex
            if(str.at(0) == 'v')
            {
                const auto strVert = str.substr(2, str.size());
                const auto vec = getVec3(strVert);
                
                vertices->push_back(vec);
            }
            
            // indicies
            if(str.at(0) == 'i')
            {
                const auto strInd = str.substr(2, str.size());
                const auto ind = stoi(strInd);
                
                indices->push_back(ind);
            }
        }
        
        const auto meshName = getMeshName(i);
        cout << meshName <<  " vertCount: " << to_string(vertices->size()) << " indCount: " << to_string(indices->size()) << endl;
        
        createTriangleMesh(meshName.c_str(), vertices->data(), vertices->size(), indices->data(), indices->size());
        
        cout << endl;
    }
    
    
    
    
    return 0;
}

physx::PxVec3 getVec3(const std::string& s)
{
    physx::PxVec3 vec;
    std::string token;
    std::istringstream tokenStream(s);
    for (unsigned int i = 0; i < 3; i++)
    {
        std::getline(tokenStream, token, '/');
        
        if(i == 0)
            vec.x = stof(token);
        else  if(i == 1)
            vec.y = stof(token);
        else if(i == 2)
            vec.z = stof(token);
    }
    
    return vec;
}
