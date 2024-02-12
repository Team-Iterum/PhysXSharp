#include "main.h"

#include <fstream>
#include <string>
#include <filesystem>
#include <chrono>
#include <thread>


using namespace std;
namespace fs = std::filesystem;

void processFile(const fs::path& meshDataName, const std::string& basePath) {

    auto vertices = make_shared<VertList>();
    auto indices = make_shared<IndList>();


    std::ifstream file(meshDataName);
    std::string str;

    while (getline(file, str))
    {
        if (str.length() == 0) continue;

        // skip comments
        if (str.substr(2) == "//") continue;

        // vertex
        if (str.at(0) == 'v')
        {
            const auto strVert = str.substr(2, str.size());
            const auto vec = getVec3(strVert);

            vertices->push_back(vec);
        }

        // indicies
        if (str.at(0) == 'i')
        {
            const auto strInd = str.substr(2, str.size());
            const auto ind = stoi(strInd);

            indices->push_back(ind);
        }
    }


    const string finalPath = basePath + separator() + string("Mesh") + separator() + meshDataName.filename().replace_extension(".mesh").string();

    cout << finalPath << " vertCount: " << to_string(vertices->size()) << " indCount: " << to_string(indices->size()) << endl;

    createTriangleMesh(finalPath.c_str(), vertices->data(), vertices->size(), indices->data(), indices->size());


    cout << endl;

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


int main(int argc, const char* argv[])
{
    cout << "PhysXSharpNative Mesh Utility 1.3" << endl;
    cout << "Help args: ./MeshUtility [basePath] [filePath]" << endl;

    const int numThreads = 2;
    const float toleranceLength = 0.5;
    const float toleranceSpeed = 5;

    initLog(logDebug, logError);
    initPhysics(false, numThreads, toleranceLength, toleranceSpeed, logCritical);

    string basePath = "C:\\Users\\avsh\\LocalProjects\\_X\\x-btf-server\\X-BTF-Descriptions\\ThingData";
    string filePath = "";

   
    if (argc > 1)
    {
        basePath = string(argv[1]);
    }
    if (argc > 2)
    {
        filePath = string(argv[2]);
    }

    cout << "basePath: " << basePath << endl;
    cout << "filePath: " << filePath << endl;


    if (filePath.length() > 1) {
        
        processFile(fs::path(filePath), basePath);
        return 0;
    }

    string path = basePath + separator() + string("Mesh");
    for (const auto& entry : fs::directory_iterator(path))
    {
        const auto extension = entry.path().extension();
        if (extension != ".txt") continue;

        const auto meshDataName = entry.path();

        processFile(meshDataName, basePath);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    return 0;
}