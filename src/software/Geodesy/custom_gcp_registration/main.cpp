#include "GCPRegister.hpp"

int main(int argc, char **argv)
{
    GCPRegister *gcpRegister = new GCPRegister();
    gcpRegister->openProject(argv[2]);
    gcpRegister->loadGCPFile(argv[1]);
    //gcpRegister->saveProject(path + "test.json");
    gcpRegister->registerProject();
    gcpRegister->saveProject(argv[2] + ".cp");
}