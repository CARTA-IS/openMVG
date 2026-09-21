#include "GCPRegister.hpp"

int main(int argc, char **argv)
{
    GCPRegister *gcpRegister = new GCPRegister();
    gcpRegister->openProject(std::string(argv[2]) + "/" + std::string(argv[3]));
    gcpRegister->loadGCPFile(std::string(argv[1]));
    //gcpRegister->saveProject(path + "test.json");
    // argv: 1=gcp_list 2=recon_dir 3=in.bin 4=out.bin [5=weight] [6=refine]
    // The weight branch used to test argc < 5 while reading argv[5]; that reads
    // out of range when called with exactly 5 arguments.
    if (argc < 6)
    {
        gcpRegister->registerProject();
    }
    //For disabling, use negative value.
    else
    {
        const std::string refine = (argc >= 7) ? std::string(argv[6]) : std::string("ADJUST_ALL");
        gcpRegister->registerProject(atof(argv[5]), refine);
    }
    std::ofstream fs(std::string(argv[2]) + "/../" + "GCP_RMS.txt");
    std::cout << "test :" << gcpRegister->log << std::endl;
    fs << gcpRegister->log;
    fs.close();
    gcpRegister->saveProject(std::string(argv[2]) + "/" + std::string(argv[4]));
}