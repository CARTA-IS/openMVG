#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"

#include "third_party/histogram/histogram.hpp"
#include "third_party/htmlDoc/htmlDoc.hpp"
#include "third_party/progress/progress.hpp"

using namespace openMVG;
using namespace openMVG::sfm;

// Save outlier residual information
Histogram<double> histoResiduals;
std::cout << "\n"
          << "=========================\n"
          << " MSE Residual InitialPair Inlier:\n";
ComputeResidualsHistogram(&histoResiduals);
std::cout << "=========================" << std::endl;

if (!sLogging_file_.empty())
{
    using namespace htmlDocument;
    html_doc_stream_->pushInfo(htmlMarkup("h1", "Essential Matrix."));
    std::ostringstream os;
    os << std::endl
       << "-------------------------------"
       << "<br>"
       << "-- Robust Essential matrix: <" << I << "," << J << "> images: "
       << view_I->s_Img_path << ","
       << view_J->s_Img_path << "<br>"
       << "-- Threshold: " << relativePose_info.found_residual_precision << "<br>"
       << "-- Resection status: "
       << "OK"
       << "<br>"
       << "-- Nb points used for robust Essential matrix estimation: "
       << xI.cols() << "<br>"
       << "-- Nb points validated by robust estimation: "
       << sfm_data_.structure.size() << "<br>"
       << "-- % points validated: "
       << sfm_data_.structure.size() / static_cast<float>(xI.cols())
       << "<br>"
       << "-------------------------------"
       << "<br>";
    html_doc_stream_->pushInfo(os.str());

    html_doc_stream_->pushInfo(htmlMarkup("h2",
                                          "Residual of the robust estimation (Initial triangulation). Thresholded at: " + toString(relativePose_info.found_residual_precision)));

    html_doc_stream_->pushInfo(htmlMarkup("h2", "Histogram of residuals"));

    const std::vector<double> xBin = histoResiduals.GetXbinsValue();
    const auto range = autoJSXGraphViewport<double>(xBin, histoResiduals.GetHist());

    htmlDocument::JSXGraphWrapper jsxGraph;
    jsxGraph.init("InitialPairTriangulationKeptInfo", 600, 300);
    jsxGraph.addXYChart(xBin, histoResiduals.GetHist(), "line,point");
    jsxGraph.addLine(relativePose_info.found_residual_precision, 0,
                     relativePose_info.found_residual_precision, histoResiduals.GetHist().front());
    jsxGraph.UnsuspendUpdate();
    jsxGraph.setViewport(range);
    jsxGraph.close();
    html_doc_stream_->pushInfo(jsxGraph.toStr());

    html_doc_stream_->pushInfo("<hr>");

    std::ofstream htmlFileStream(std::string(stlplus::folder_append_separator(sOut_directory_) +
                                             "Reconstruction_Report.html")
                                     .c_str());
    htmlFileStream << html_doc_stream_->getDoc();
}

double SequentialSfMReconstructionEngine::ComputeResidualsHistogram(Histogram<double> *histo)
{
    // Collect residuals for each observation
    std::vector<float> vec_residuals;
    vec_residuals.reserve(sfm_data_.structure.size());
    for (const auto &landmark_entry : sfm_data_.GetLandmarks())
    {
        const Observations &obs = landmark_entry.second.obs;
        for (const auto &observation : obs)
        {
            const View *view = sfm_data_.GetViews().find(observation.first)->second.get();
            const Pose3 pose = sfm_data_.GetPoseOrDie(view);
            const auto intrinsic = sfm_data_.GetIntrinsics().find(view->id_intrinsic)->second;
            const Vec2 residual = intrinsic->residual(pose(landmark_entry.second.X), observation.second.x);
            vec_residuals.emplace_back(std::abs(residual(0)));
            vec_residuals.emplace_back(std::abs(residual(1)));
        }
    }
    // Display statistics
    if (vec_residuals.size() > 1)
    {
        float dMin, dMax, dMean, dMedian;
        minMaxMeanMedian<float>(vec_residuals.cbegin(), vec_residuals.cend(),
                                dMin, dMax, dMean, dMedian);
        if (histo)
        {
            *histo = Histogram<double>(dMin, dMax, 10);
            histo->Add(vec_residuals.cbegin(), vec_residuals.cend());
        }

        std::cout << std::endl
                  << std::endl;
        std::cout << std::endl
                  << "SequentialSfMReconstructionEngine::ComputeResidualsMSE."
                  << "\n"
                  << "\t-- #Tracks:\t" << sfm_data_.GetLandmarks().size() << std::endl
                  << "\t-- Residual min:\t" << dMin << std::endl
                  << "\t-- Residual median:\t" << dMedian << std::endl
                  << "\t-- Residual max:\t " << dMax << std::endl
                  << "\t-- Residual mean:\t " << dMean << std::endl;

        return dMean;
    }
    return -1.0;
}