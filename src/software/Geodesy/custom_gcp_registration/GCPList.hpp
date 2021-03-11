#ifndef GCPLIST_HPP
#define GCPLIST_HPP

#include <string>
#include "GCP.hpp"

class GCPList
{
private:
    std::string prj;

public:
    GCPList();
    ~GCPList();
    void GCPPush(GCP newGcp);
    void SetProjection(std::string prjStr);
    std::string GetProjection();
    int GetSize();
    GCP GCPPop(); // Get last GCP correspondence.
}
#endif