#include "main.h"

#include <fstream>
#include <string>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;


int main (int argc, const char* argv[])
{
    cout << "PhysXSharpNative Mesh Utility 1.1" << endl;
    cout << "Help args: ./MeshUtility [basePath]" << endl;
    
    const int numThreads = 2;
    const float toleranceLength = 0.5;
    const float toleranceSpeed = 5;
    
    initLog(logDebug, logError);
    initPhysics(false, numThreads, toleranceLength, toleranceSpeed, logCritical);
    
    string basePath = "";
    if(argc > 1) 
    {
        basePath = string(argv[1]);
    }

    string path = basePath + separator() + string("Mesh");
    for (const auto & entry : fs::directory_iterator(path))
    {
        const auto extension = entry.path().extension();
        if(extension != ".txt") continue;
        
        auto vertices = make_shared<VertList>();
        auto indices = make_shared<IndList>();
        
        const auto meshDataName = entry.path();
                
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
        
        
        const string finalPath = path + separator() + entry.path().filename().replace_extension(".mesh").string();
        
        cout << finalPath <<  " vertCount: " << to_string(vertices->size()) << " indCount: " << to_string(indices->size()) << endl;
        
        createTriangleMesh(finalPath.c_str(), vertices->data(), vertices->size(), indices->data(), indices->size());
        
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
