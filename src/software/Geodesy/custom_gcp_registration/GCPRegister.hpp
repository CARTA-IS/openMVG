#ifndef GCPREGISTER_HPP
#define GCPREGISTER_HPP

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "GCPList.hpp"
#include "document.hpp"
#include "openMVG/cameras/Camera_Common.hpp"

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
    // refine: which intrinsics the GCP-weighted bundle adjustment may move.
    // Parse the command-line string with
    // cameras::StringTo_Intrinsic_Parameter_Type so that the '|' combinations
    // main_GlobalSfM accepts work here too; it is the caller's job to reject a
    // parse failure (the helper returns Intrinsic_Parameter_Type(0)).
    // Defaults to ADJUST_ALL so the previous behaviour is unchanged.
    void registerProject(
        double weight = 20.0,
        openMVG::cameras::Intrinsic_Parameter_Type refine =
            openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL);
};
#endif