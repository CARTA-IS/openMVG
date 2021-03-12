#include "GCPRegister.hpp"

int main(int argc, char **argv)
{
    GCPRegister *gcpRegister = new gcpRegister();
    gcpRegister->loadGCPFile(argv[1]);
    gcpRegister->openProject(argv[2]);
    //gcpRegister->saveProject();
}