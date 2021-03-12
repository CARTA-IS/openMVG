#ifndef GCPREGISTER_HPP
#define GCPREGISTER_HPP

#include <string>
#include <fstream>
#include <iostream>
#include "GCPList.hpp"
#include "document.hpp"

class GCPRegister
{
private:
    std::string prj;
    Document m_doc;
    std::string sfmFileName;
    GCPList gcpList;
    void saveProject();
    void openProject(std::string projectPath);
    void loadGCPFile(std::string gcpFile);
    void registerProject();
    void SetProjection(std::string prjStr);
    std::string GetProjection();

public:
    GCPRegister();
    ~GCPRegister();
}
#endif