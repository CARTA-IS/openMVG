#include "ReportGenerator.hpp"

ReportGenerator::ReportGenerator() {}
ReportGenerator::~ReportGenerator() {}
void ReportGenerator::openProject(std::string projectPath)
{
    if (!m_doc.loadData(projectPath))
    {
        std::cout << "Cannot open the sfm_data file." << projectPath << std::endl;
    }
}
void ReportGenerator::generateHistogram(std::string outputPath)
{
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

        std::ofstream htmlFileStream(outputPath.c_str());
        htmlFileStream << html_doc_stream_->getDoc();
    }
}
bool ReportGenerator::minMaxMeanMedianRMSE(std::vector<Vec2>::const_iterator begin, std::vector<Vec2>::const_iterator end,
                                           double &min, double &max, double &mean, double &median, double &rmse, std::vector<double> &vec_val)
{
    if (std::distance(begin, end) < 1)
    {
        return false;
    }
    vec_val.reserve(std::distance(begin, end));
    int count = 0;
    for (auto it = begin; it != end; it++)
    {
        vec_val.push_back(sqrt(pow(it->x, 2.0) + pow(it->y, 2.0)));
    }

    // Get the median value:
    const auto middle = vec_val.begin() + vec_val.size() / 2;
    std::nth_element(vec_val.begin(), middle, vec_val.end());
    median = *middle;
    min = *std::min_element(vec_val.begin(), middle);
    max = *std::max_element(middle, vec_val.end());
    mean = std::accumulate(vec_val.cbegin(), vec_val.cend(), 0.0) / static_cast<double>(vec_val.size());
    rmse = 0;
    for (auto it = vec_val.begin(); it != vec_val.end(); it++)
    {
        rmse += pow(*it, 2.0); //generate squared sum.
    }
    rmse /= static_cast<double>(vec_val.size());
    rmse = sqrt(rmse);
    return true;
}
// Actual residual calculation.
double ReportGenerator::ComputeResidualsHistogram(Histogram<double> *histo)
{
    // Collect residuals for each observation
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
            vec_residuals.emplace_back(residual);
        }
    }
    // Display statistics
    if (vec_residuals.size() > 1)
    {
        double dMin, dMax, dMean, dMedian, dRMSE;
        std::vector<double> vec_val;
        minMaxMeanMedian<float>(vec_residuals.cbegin(), vec_residuals.cend(),
                                dMin, dMax, dMean, dMedian, dRMSE, vec_val);
        if (histo)
        {
            *histo = Histogram<double>(dMin, dMax, 10);
            histo->Add(vec_val.cbegin(), vec_val.cend());
        }

        std::cout << std::endl
                  << std::endl;
        std::cout << std::endl
                  << "ComputeResidualsHistogram."
                  << "\n"
                  << "\t-- #Tracks:\t" << sfm_data_.GetLandmarks().size() << std::endl
                  << "\t-- Residual min:\t" << dMin << std::endl
                  << "\t-- Residual median:\t" << dMedian << std::endl
                  << "\t-- Residual max:\t " << dMax << std::endl
                  << "\t-- Residual mean:\t " << dMean << std::endl
                  << "\t-- Residual rmse:\t " << dRMSE << std::endl;

        return dMean;
    }
    return -1.0;
}