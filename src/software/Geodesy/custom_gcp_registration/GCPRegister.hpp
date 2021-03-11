#ifndef GCPREGISTER_HPP
#define GCPREGISTER_HPP

#include <string>
#include "GCPList.hpp"

class GCPRegister
{
private:
    std::string sfmFileName;
    GCPList gcpList;
    void saveProject();
    void openProject(string projectPath);
    void loadGCPFile(string gcpFile);
    void registerProject();

public:
    GCPRegister();
    ~GCPRegister();
}
#endif