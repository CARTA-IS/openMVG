#include "ReportGenerator.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"

using namespace openMVG;
using namespace openMVG::sfm;

int main(int argc, char **argv)
{
    ReportGenerator *reportGenerator = new ReportGenerator();
    reportGenerator->openProject(std::string(argv[1]));
    reportGenerator->generateHistogram(std::string(argv[2]));
    return 0;
}