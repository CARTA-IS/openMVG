#include "GCPRegister.hpp"

#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"

#include <cstdlib>
#include <stdexcept>

int main(int argc, char **argv)
{
    // argv: 1=gcp_list 2=recon_dir 3=in.bin 4=out.bin [5=weight] [6=refine]
    //
    // argv[1..4] were read unconditionally, so calling this with fewer than
    // four arguments dereferenced past the end of argv.
    if (argc < 5)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <gcp_list.txt> <reconstruction_dir> <input.bin>"
                     " <output.bin> [weight] [refine]\n"
                     "  weight  GCP weight for the registration BA. Default 20."
                     " Zero or negative drops the\n"
                     "          GCP term (useBundle = weight > 0), leaving the"
                     " similarity transform as the\n"
                     "          only registration.\n"
                     "  refine  Which intrinsics the BA may move, same grammar"
                     " as main_GlobalSfM -f. Default ADJUST_ALL.\n"
                     "          NONE | ADJUST_FOCAL_LENGTH |"
                     " ADJUST_PRINCIPAL_POINT | ADJUST_DISTORTION |"
                     " ADJUST_ALL, combinable with '|'."
                  << std::endl;
        return EXIT_FAILURE;
    }

    // atof reports a malformed number as 0.0 without setting errno, and
    // GCPRegister turns weight <= 0 into "skip the GCP term"
    // (useBundle = weight > 0). A typo would therefore disable the GCP-weighted
    // BA as quietly as passing 0 or a negative value deliberately does. stod
    // separates the two: a bad number fails the process, 0 and negatives are
    // kept as the documented way to ask for similarity-only registration, and
    // registerProject says so on stdout when it takes that path.
    double weight = 20.0;
    if (argc >= 6)
    {
        try
        {
            std::size_t consumed = 0;
            weight = std::stod(argv[5], &consumed);
            if (consumed != std::string(argv[5]).size())
                throw std::invalid_argument("trailing characters");
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid input for the GCP registration weight: '"
                      << argv[5] << "'. Pass a number; zero or negative drops"
                         " the GCP term." << std::endl;
            return EXIT_FAILURE;
        }
    }

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

    // None of the steps below used to reach the exit code: main saved
    // unconditionally and returned 0, so a run that registered nothing wrote
    // the untouched sfm_data to the output path and looked like a success.
    // Bail out instead, leaving the output absent rather than present and wrong.
    // Deliberately left on the heap and never deleted, as before: the process
    // exits right after, and running ~GCPRegister here would be a teardown path
    // this program has never taken.
    GCPRegister *gcpRegister = new GCPRegister();
    if (!gcpRegister->openProject(std::string(argv[2]) + "/" + std::string(argv[3])))
        return EXIT_FAILURE;
    if (!gcpRegister->loadGCPFile(std::string(argv[1])))
        return EXIT_FAILURE;

    // The weight branch used to test argc < 5 while reading argv[5]; that reads
    // out of range when called with exactly 5 arguments.
    //For disabling, use negative value.
    const bool registered = (argc < 6)
        ? gcpRegister->registerProject()
        : gcpRegister->registerProject(weight, intrinsic_refinement_options);

    // Leave both outputs alone on failure, for the same reason out.bin is left
    // alone: GCPRegister only fills `log` in the success path, so writing it
    // here would replace the previous run's report with an empty file while
    // out.bin still held that run's result -- a mismatched pair on disk. The
    // failure itself is on stderr and in whatever log the caller tees.
    if (!registered)
    {
        std::cerr << "GCP registration failed; leaving "
                  << std::string(argv[2]) + "/" + std::string(argv[4])
                  << " and GCP_RMS.txt untouched." << std::endl;
        return EXIT_FAILURE;
    }

    std::ofstream fs(std::string(argv[2]) + "/../" + "GCP_RMS.txt");
    std::cout << "test :" << gcpRegister->log << std::endl;
    fs << gcpRegister->log;
    fs.close();

    if (!gcpRegister->saveProject(std::string(argv[2]) + "/" + std::string(argv[4])))
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}