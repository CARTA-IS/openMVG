#ifndef REPORTGENERATOR_HPP
#define REPORTGENERATOR_HPP

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "software/Geodesy/custom_gcp_registration/document.hpp"

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"

#include "third_party/histogram/histogram.hpp"
#include "third_party/htmlDoc/htmlDoc.hpp"
#include "third_party/progress/progress.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace htmlDocument;

class ReportGenerator
{
private:
    std::string prj;
    Document m_doc;
    std::string loggingFileName;
    std::string sfmFileName;
    std::vector<Vec2> vec_residuals;
    std::shared_ptr<htmlDocument::htmlDocumentStream> html_doc_stream_;

public:
    ReportGenerator();
    ~ReportGenerator();
    void openProject(std::string projectPath);
    void reportProject();
    bool minMaxMeanMedianRMSE(std::vector<Vec2>::const_iterator begin, std::vector<Vec2>::const_iterator end,
                              double &min, double &max, double &mean, double &median, double &rmse, std::vector<double> &vec);
    void generateHistogram(std::string outputPath);
    void ComputeResidualsHistogram(Histogram<double> *histo, std::ostringstream &os);
};
#endif