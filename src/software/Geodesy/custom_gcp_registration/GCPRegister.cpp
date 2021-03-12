#include "GCPRegister.hpp"

//openMVG Libraries
#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/multiview/triangulation_nview.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"
#include "openMVG/geometry/rigid_transformation3D_srt.hpp"
#include "openMVG/geometry/Similarity3.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_transform.hpp"

#include "openMVG/stl/stl.hpp"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;

GCPRegister::GCPRegister() {}
GCPRegister::~GCPRegister() {}
void GCPRegister::SetProjection(std::string prjStr)
{
    prj = prjStr;
}
std::string GCPRegister::GetProjection()
{
    return prj;
}
void GCPRegister::saveProject(string dstPath)
{
    if (!m_doc.saveData(dstPath))
    {
        std::cout << "Cannot save the sfm_data file." << std::endl;
    }
}
void GCPRegister::openProject(string projectPath)
{
    if (!m_doc.loadData(sfm_data_fileName.toStdString()))
    {
        std::cout << "Cannot open the sfm_data file." << std::endl;
    }
}
void GCPRegister::loadGCPFile(std::string gcpFile)
{
    // Graphical widget to configure the control point position
    if (m_doc._sfm_data.control_points.empty())
    {
        Landmarks control_points_cpy;
        GCPList gcpList;
        if (gcpFile.is_open())
        {
            std::string line;
            int cnt = 0;
            while (getline(gcpFile, line))
            {
                if (cnt == 0) //prj string case.
                {
                    cnt++;
                }
                std::vector<std::string> gcpTokens;
                std::string token;
                std::stringstream ss(line);
                while (ss >> token)
                {
                    gcpTokens.push_back(token);
                }
                GCP tmpGCP(std::stod(gcpTokens.at(0)), std::stod(gcpTokens.at(1)), std::stod(gcpTokens.at(2)), std::stod(gcpTokens.at(3)),std::stod(gcpTokens.at(4)), gcpTokens.at(5))); // X Y Z coordinates.
                gcpList.GCPPush(tmpGCP);
            }
        }
        //transform gcpList to openMVG::Landmarks
        while (gcpList.GetSize() != 0)
        {
            GCP tmpGCP = gcpList.GCPPop();
            Landmark tmpLandmark;
            tmpLandmark.X = tmpGCP.GetX();
            for (i = 0; i < tmpGCP.markers.size(); i++)
            {
                Vec2 pt = tmpGCP.markers.at(i);
                tmpLandmark.obs.insert({tmpLandmark.obs.size(), Observation(pt, i)});
            }
            control_points_cpy.insert({control_points_cpy.size(), tmpLandmark});
        }
        m_doc._sfm_data.control_points = control_points_cpy;
    }
    else
    {
        std::cout << "Already control points are in the SfM file." << std::endl;
        return;
    }
}
void GCPRegister::registerProject() {}
