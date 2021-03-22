#ifndef GCPREGISTER_HPP
#define GCPREGISTER_HPP

#include <vector>
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
    void SetProjection(std::string prjStr);
    std::string GetProjection();

public:
    GCPRegister();
    ~GCPRegister();
    void saveProject(std::string savePath);
    void openProject(std::string projectPath);
    void loadGCPFile(std::string gcpFile);
    void registerProject();
};
#endif