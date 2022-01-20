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
    std::ostringstream os;
    ComputeResidualsHistogram(&histoResiduals, os);

    std::cout << "\nHistogram of residuals:\n"
              << histoResiduals.ToString() << std::endl;

    if (!outputPath.empty())
    {
        // setup HTML logger
        html_doc_stream_ = std::make_shared<htmlDocument::htmlDocumentStream>("ReconstructionEngine SFM report.");
        html_doc_stream_->pushInfo(
            htmlDocument::htmlMarkup("h1", std::string("SfMReconstructionEngine")));
        html_doc_stream_->pushInfo("<hr>");

        html_doc_stream_->pushInfo("Dataset info:");
        html_doc_stream_->pushInfo("Views count: " +
                                   htmlDocument::toString(m_doc._sfm_data.GetViews().size()) + "<br>");

        os << "Structure from Motion process finished.";
        html_doc_stream_->pushInfo("<hr>");
        html_doc_stream_->pushInfo(htmlMarkup("h1", os.str()));

        os.str("");
        os << "-------------------------------"
           << "<br>"
           << "-- Structure from Motion (statistics):<br>"
           << "-- #Camera calibrated: " << m_doc._sfm_data.GetPoses().size()
           << " from " << m_doc._sfm_data.GetViews().size() << " input images.<br>"
           << "-- #Tracks, #3D points: " << m_doc._sfm_data.GetLandmarks().size() << "<br>"
           << "-------------------------------"
           << "<br>";
        html_doc_stream_->pushInfo(os.str());

        html_doc_stream_->pushInfo(htmlMarkup("h2", "Histogram of reprojection-residuals"));

        const std::vector<double> xBin = histoResiduals.GetXbinsValue();
        const auto range = autoJSXGraphViewport<double>(xBin, histoResiduals.GetHist());

        htmlDocument::JSXGraphWrapper jsxGraph;
        jsxGraph.init("3DtoImageResiduals", 1200, 600);
        jsxGraph.addXYChart(xBin, histoResiduals.GetHist(), "line,point");
        jsxGraph.UnsuspendUpdate();
        jsxGraph.setViewport(range);
        jsxGraph.close();
        html_doc_stream_->pushInfo(jsxGraph.toStr());

        // Save the reconstruction Log
        std::ofstream htmlFileStream(outputPath.c_str());
        htmlFileStream << html_doc_stream_->getDoc();
        std::cout << "save at " << outputPath << std::endl;
    }
    return;
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
        vec_val.push_back(sqrt(pow(static_cast<double>((*it)[0]), 2.0) + pow(static_cast<double>((*it)[0]), 2.0)));
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
void ReportGenerator::ComputeResidualsHistogram(Histogram<double> *histo, std::ostringstream &os)
{
    // Collect residuals for each observation
    vec_residuals.reserve(m_doc._sfm_data.structure.size());
    for (const auto &landmark_entry : m_doc._sfm_data.GetLandmarks())
    {
        const Observations &obs = landmark_entry.second.obs;
        for (const auto &observation : obs)
        {
            const View *view = m_doc._sfm_data.GetViews().find(observation.first)->second.get();
            const Pose3 pose = m_doc._sfm_data.GetPoseOrDie(view);
            const auto intrinsic = m_doc._sfm_data.GetIntrinsics().find(view->id_intrinsic)->second;
            const Vec2 residual = intrinsic->residual(pose(landmark_entry.second.X), observation.second.x);
            vec_residuals.emplace_back(residual);
        }
    }
    // Display statistics
    if (vec_residuals.size() > 1)
    {
        double dMin, dMax, dMean, dMedian, dRMSE;
        std::vector<double> vec_val;
        minMaxMeanMedianRMSE(vec_residuals.cbegin(), vec_residuals.cend(),
                             dMin, dMax, dMean, dMedian, dRMSE, vec_val);
        if (histo)
        {
            *histo = Histogram<double>(dMin, dMax, 10);
            histo->Add(vec_val.cbegin(), vec_val.cend());
        }

        std::cout << std::endl;
        //std::cout
        os << "\n"
           << "ComputeResidualsHistogram."
           << "\n"
           << "\t-- #Tracks:\t" << m_doc._sfm_data.GetLandmarks().size() << "\n"
           << "\t-- Residual min:\t" << dMin << "\n"
           << "\t-- Residual median:\t" << dMedian << "\n"
           << "\t-- Residual max:\t " << dMax << "\n"
           << "\t-- Residual mean:\t " << dMean << "\n"
           << "\t-- Residual rmse:\t " << dRMSE << "\n";
        std::cout << os.str();
        std::istringstream iss(os.str());
        std::string token;
        //clear os
        os.str("");
        os.clear();
        while (getline(iss, token))
        {
            os << token << "<br>";
        }
        return;
    }
    return;
}