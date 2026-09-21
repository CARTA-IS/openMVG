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
    std::string log;
    void saveProject(std::string savePath);
    void openProject(std::string projectPath);
    void loadGCPFile(std::string gcpFile);
    // refine: intrinsic refine mode for the GCP-weighted bundle adjustment.
    // "NONE" | "ADJUST_FOCAL_LENGTH" | "ADJUST_PRINCIPAL_POINT" | "ADJUST_DISTORTION" | "ADJUST_ALL"
    // Defaults to ADJUST_ALL so the previous behaviour is unchanged.
    void registerProject(double weight = 20.0,
                         const std::string &refine = "ADJUST_ALL");
};
#endif