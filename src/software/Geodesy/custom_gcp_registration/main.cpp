#include "GCPRegister.hpp"

int main(int argc, char **argv)
{
    GCPRegister *gcpRegister = new GCPRegister();
    std::string path = "/home/snatcherai/Desktop/Seungsu/prj_MVG/test/GCP/";
    gcpRegister->openProject(path + argv[2]);
    gcpRegister->loadGCPFile(path + argv[1]);
    gcpRegister->saveProject(path + "test.json");
    gcpRegister->registerProject();
    gcpRegister->saveProject(path + "test_cp.bin");
}