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
void GCPRegister::saveProject(std::string dstPath)
{
    if (!m_doc.saveData(dstPath))
    {
        std::cout << "Cannot save the sfm_data file." << dstPath << std::endl;
    }
}
void GCPRegister::openProject(std::string projectPath)
{
    if (!m_doc.loadData(projectPath))
    {
        std::cout << "Cannot open the sfm_data file." << projectPath << std::endl;
    }
}
void GCPRegister::loadGCPFile(std::string gcpFile)
{
    // Graphical widget to configure the control point position
    if (m_doc._sfm_data.control_points.empty())
    {
        Landmarks control_points_cpy;
        GCPList gcpList;
        std::ifstream openFile(gcpFile.data());
        if (openFile.is_open())
        {
            std::string line;
            int cnt = 0;
            while (getline(openFile, line))
            {
                if (cnt == 0) //prj string case.
                {
                    SetProjection(line);
                    cnt++;
                    continue;
                }
                std::vector<std::string> gcpTokens;
                std::string token;
                std::stringstream ss(line);
                while (ss >> token)
                {
                    gcpTokens.push_back(token);
                }
                GCP tmpGCP(std::stod(gcpTokens.at(0)), std::stod(gcpTokens.at(1)), std::stod(gcpTokens.at(2)), std::stod(gcpTokens.at(3)), std::stod(gcpTokens.at(4)), gcpTokens.at(5)); // X Y Z coordinates.
                gcpList.GCPPush(tmpGCP);
            }
        }
        openFile.close();
        //transform gcpList to openMVG::Landmarks
        int obsCnt = 0;
        while (gcpList.GetSize() != 0)
        {
            GCP tmpGCP = gcpList.GCPPop();
            Landmark *tmpLandmark = new Landmark();
            tmpLandmark->X = tmpGCP.GetX();
            //std::cout << &tmpLandmark->X(0) << " " << &tmpLandmark->X(1) << " " << &tmpLandmark->X(2) << std::endl;

            for (int i = 0; i < tmpGCP.markers.size(); i++) //the number of makers in a GCP.
            {
                Vec2 pt = tmpGCP.markers.at(i).pixels;
                //std::cout << "Observation : " << pt << std::endl;
                std::string imageName = tmpGCP.markers.at(i).imageName;
                int viewKey = -1;
                for (auto it = m_doc._sfm_data.GetViews().begin(); it != m_doc._sfm_data.GetViews().end(); ++it)
                {
                    //std::cout << it->first << " " << it->second->s_Img_path << std::endl;
                    if (imageName.compare(it->second->s_Img_path) == 0)
                    {
                        viewKey = it->first;
                        break;
                    }
                }
                if (viewKey == -1)
                {
                    std::cout << "Cannot find same image in the view..." << std::endl;
                    //not calibrated image case.
                    continue;
                }
                tmpLandmark->obs.insert({viewKey, Observation(pt, 0)});
            }
            control_points_cpy.insert({control_points_cpy.size() + 1, *tmpLandmark});
        }
        m_doc._sfm_data.control_points = control_points_cpy;
    }
    else
    {
        std::cout << "Already control points are in the SfM file." << std::endl;
        return;
    }
}
void GCPRegister::registerProject()
{
    if (m_doc._sfm_data.control_points.size() < 3)
    {
        std::cout << "At least 3 control points are required." << std::endl;
        return;
    }
    // Assert that control points can be triangulated
    for (Landmarks::const_iterator iterL = m_doc._sfm_data.control_points.begin();
         iterL != m_doc._sfm_data.control_points.end(); ++iterL)
    {
        if (iterL->second.obs.size() < 2)
        {
            std::cout << "Each control point must be defined in at least 2 pictures." << std::endl;
            return;
        }
    }

    //---
    // registration (coarse):
    // - compute the 3D points corresponding to the control point observation for the SfM scene
    // - compute a coarse registration between the controls points & the triangulated point
    // - transform the scene according the found transformation
    //---
    std::map<IndexT, Vec3> map_control_points, map_triangulated;
    std::map<IndexT, double> map_triangulation_errors;
    for (const auto &control_point_it : m_doc._sfm_data.control_points)
    {
        const Landmark &landmark = control_point_it.second;
        //Triangulate the observations:
        const Observations &obs = landmark.obs;
        std::vector<Vec3> bearing;
        std::vector<Mat34> poses;
        bearing.reserve(obs.size());
        poses.reserve(obs.size());

        for (const auto &obs_it : obs)
        {
            const View *view = m_doc._sfm_data.views.at(obs_it.first).get();
            if (!m_doc._sfm_data.IsPoseAndIntrinsicDefined(view))
                continue;
            const openMVG::cameras::IntrinsicBase *cam = m_doc._sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
            const openMVG::geometry::Pose3 pose = m_doc._sfm_data.GetPoseOrDie(view);
            const Vec2 pt = obs_it.second.x;
            bearing.emplace_back((*cam)(cam->get_ud_pixel(pt)));
            poses.emplace_back(pose.asMatrix());
        }
        const Eigen::Map<const Mat3X> bearing_matrix(bearing[0].data(), 3, bearing.size());
        Vec4 Xhomogeneous;
        if (!TriangulateNViewAlgebraic(bearing_matrix, poses, &Xhomogeneous))
        {
            std::cout << "Invalid triangulation" << std::endl;
            return;
        }
        const Vec3 X = Xhomogeneous.hnormalized();
        // Test validity of the hypothesis (front of the cameras):
        bool bCheirality = true;
        int i(0);
        double reprojection_error_sum(0.0);
        for (const auto &obs_it : obs)
        {
            const View *view = m_doc._sfm_data.views.at(obs_it.first).get();
            if (!m_doc._sfm_data.IsPoseAndIntrinsicDefined(view))
                continue;

            const Pose3 pose = m_doc._sfm_data.GetPoseOrDie(view);
            bCheirality &= CheiralityTest(bearing[i], pose, X);
            const openMVG::cameras::IntrinsicBase *cam = m_doc._sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
            const Vec2 pt = obs_it.second.x;
            const Vec2 residual = cam->residual(pose(X), pt);
            reprojection_error_sum += residual.norm();
            ++i;
        }
        if (bCheirality) // Keep the point only if it has a positive depth
        {
            map_triangulated[control_point_it.first] = X;
            map_control_points[control_point_it.first] = landmark.X;
            map_triangulation_errors[control_point_it.first] = reprojection_error_sum / (double)bearing.size();
        }
        else
        {
            std::cout << "Control Point cannot be triangulated (not in front of the cameras)" << std::endl;
            return;
        }
    }

    if (map_control_points.size() < 3)
    {
        std::cout << "Insufficient number of triangulated control points." << std::endl;
        return;
    }

    // compute the similarity
    {
        // data conversion to appropriate container
        Mat x1(3, map_control_points.size()),
            x2(3, map_control_points.size());

        IndexT id_col = 0;
        for (const auto &cp : map_control_points)
        {
            x1.col(id_col) = map_triangulated[cp.first];
            x2.col(id_col) = cp.second;
            ++id_col;
        }

        std::cout
            << "Control points observation triangulations:\n"
            << x1 << std::endl
            << std::endl
            << "Control points coords:\n"
            << x2 << std::endl
            << std::endl;

        Vec3 t;
        Mat3 R;
        double S;
        if (openMVG::geometry::FindRTS(x1, x2, &S, &t, &R))
        {
            openMVG::geometry::Refine_RTS(x1, x2, &S, &t, &R);
            std::cout << "Found transform:\n"
                      << " scale: " << S << "\n"
                      << " rotation:\n"
                      << R << "\n"
                      << " translation: " << t.transpose() << std::endl;

            //--
            // Apply the found transformation as a 3D Similarity transformation matrix // S * R * X + t
            //--

            const openMVG::geometry::Similarity3 sim(geometry::Pose3(R, -R.transpose() * t / S), S);
            openMVG::sfm::ApplySimilarity(sim, m_doc._sfm_data);

            // Display some statistics:
            std::stringstream os;
            for (Landmarks::const_iterator iterL = m_doc._sfm_data.control_points.begin();
                 iterL != m_doc._sfm_data.control_points.end(); ++iterL)
            {
                const IndexT CPIndex = iterL->first;
                // If the control point has not been used, continue...
                if (map_triangulation_errors.find(CPIndex) == map_triangulation_errors.end())
                    continue;

                os
                    << "CP index: " << CPIndex << "\n"
                    << "CP triangulation error: " << map_triangulation_errors[CPIndex] << " pixel(s)\n"
                    << "CP registration error: "
                    << (sim(map_triangulated[CPIndex]) - map_control_points[CPIndex]).norm() << " user unit(s)"
                    << "\n\n";
            }
            std::cout << os.str();

            //QMessageBox msgBox;
            //msgBox.setText(QString::fromStdString(string_pattern_replace(os.str(), "\n", "<br>")));
            //msgBox.exec();
        }
        else
        {
            std::cout << "Registration failed. Please check your Control Points coordinates." << std::endl;
        }
    }

    //---
    // Bundle adjustment with GCP
    //---
    {
        std::cout << "debug begin" << std::endl;
        using namespace openMVG::sfm;
        Bundle_Adjustment_Ceres::BA_Ceres_options options;
        Bundle_Adjustment_Ceres bundle_adjustment_obj(options);
        Control_Point_Parameter control_point_opt(20.0, true);
        if (!bundle_adjustment_obj.Adjust(m_doc._sfm_data,
                                          Optimize_Options(
                                              cameras::Intrinsic_Parameter_Type::NONE, // Keep intrinsic constant
                                              Extrinsic_Parameter_Type::ADJUST_ALL,    // Adjust camera motion
                                              Structure_Parameter_Type::ADJUST_ALL,    // Adjust structure
                                              control_point_opt                        // Use GCP and weight more their observation residuals
                                              )))
        {
            std::cout << "BA with GCP failed." << std::endl;
        }
        std::cout << "debug finish" << std::endl;
    }
}
