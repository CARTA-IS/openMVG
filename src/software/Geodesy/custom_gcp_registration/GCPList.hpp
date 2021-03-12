#ifndef GCPLIST_HPP
#define GCPLIST_HPP

#include <string>
#include "GCP.hpp"

class GCPList
{
private:
    std::vector<GCP> gcplist;

public:
    GCPList();
    ~GCPList();
    void GCPPush(GCP newGcp); // push and merge same GCP in to one GCP.
    int GetSize();
    GCP GCPPop(); // Get last GCP correspondence.
};
#endif