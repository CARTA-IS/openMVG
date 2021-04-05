#include "GCPRegister.hpp"

int main(int argc, char **argv)
{
    GCPRegister *gcpRegister = new GCPRegister();
    gcpRegister->openProject(std::string(argv[2]));
    gcpRegister->loadGCPFile(std::string(argv[1]));
    //gcpRegister->saveProject(path + "test.json");
    if (argc < 5)
        gcpRegister->registerProject();
    //For disabling, use negative value.
    else
    {
        gcpRegister->registerProject(atof(argv[4]));
    }
    gcpRegister->saveProject(std::string(argv[3]));
}