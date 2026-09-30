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
    // Second-pass (RMS report) failure: the model is already registered, so
    // record why the numbers are missing and report success.
    bool rmsReportUnavailable(const std::string &reason);

public:
    GCPRegister();
    ~GCPRegister();
    std::string log;
    // Every step reports failure. main() turns that into a non-zero exit so a
    // caller cannot mistake an unregistered model for a registered one.
    bool saveProject(std::string savePath);
    bool openProject(std::string projectPath);
    bool loadGCPFile(std::string gcpFile);
    // refine: which intrinsics the GCP-weighted bundle adjustment may move.
    // Parse the command-line string with
    // cameras::StringTo_Intrinsic_Parameter_Type so that the '|' combinations
    // main_GlobalSfM accepts work here too; it is the caller's job to reject a
    // parse failure (the helper returns Intrinsic_Parameter_Type(0)).
    // Defaults to ADJUST_ALL so the previous behaviour is unchanged.
    bool registerProject(
        double weight = 20.0,
        openMVG::cameras::Intrinsic_Parameter_Type refine =
            openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL);
};
#endif