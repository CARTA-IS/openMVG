#include "GCPRegister.hpp"

#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"

int main(int argc, char **argv)
{
    // argv: 1=gcp_list 2=recon_dir 3=in.bin 4=out.bin [5=weight] [6=refine]
    //
    // refine uses the same grammar as main_GlobalSfM's -f/--refineIntrinsics,
    // so '|' combinations work. Parse and validate before touching the project:
    // the helper returns Intrinsic_Parameter_Type(0) on an unknown key, and a
    // caller that narrowed the intrinsics must not silently get ADJUST_ALL.
    auto intrinsic_refinement_options =
        openMVG::cameras::Intrinsic_Parameter_Type::ADJUST_ALL;
    if (argc >= 7)
    {
        intrinsic_refinement_options =
            openMVG::cameras::StringTo_Intrinsic_Parameter_Type(argv[6]);
        if (intrinsic_refinement_options ==
            static_cast<openMVG::cameras::Intrinsic_Parameter_Type>(0))
        {
            std::cerr << "Invalid input for Bundle Adjusment Intrinsic parameter "
                         "refinement option" << std::endl;
            return EXIT_FAILURE;
        }
    }

    GCPRegister *gcpRegister = new GCPRegister();
    gcpRegister->openProject(std::string(argv[2]) + "/" + std::string(argv[3]));
    gcpRegister->loadGCPFile(std::string(argv[1]));
    //gcpRegister->saveProject(path + "test.json");
    // The weight branch used to test argc < 5 while reading argv[5]; that reads
    // out of range when called with exactly 5 arguments.
    if (argc < 6)
    {
        gcpRegister->registerProject();
    }
    //For disabling, use negative value.
    else
    {
        gcpRegister->registerProject(atof(argv[5]), intrinsic_refinement_options);
    }
    std::ofstream fs(std::string(argv[2]) + "/../" + "GCP_RMS.txt");
    std::cout << "test :" << gcpRegister->log << std::endl;
    fs << gcpRegister->log;
    fs.close();
    gcpRegister->saveProject(std::string(argv[2]) + "/" + std::string(argv[4]));
}