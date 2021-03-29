#include "GCPRegister.hpp"

int main(int argc, char **argv)
{
    GCPRegister *gcpRegister = new GCPRegister();
    gcpRegister->openProject(std::string(argv[2]));
    gcpRegister->loadGCPFile(std::string(argv[1]));
    //gcpRegister->saveProject(path + "test.json");
    gcpRegister->registerProject();
    gcpRegister->saveProject(std::string(argv[2]) + ".cp");
}