#include "GCPList.hpp"

GCPList::GCPList() {}
GCPList::~GCPList() {}
void GCPList::GCPPush(GCP newGcp) // push and merge same GCP in to one GCP.
{
    for (int i = 0; i < gcplist.size(); i++)
    {
        if (gcplist.at(i) == newGcp) // find same GCP.
        {
            gcplist.at(i) = gcplist.at(i) + newGcp;
            return;
        }
    }
    gcplist.push_back(newGCP);
}
int GCPList::GetSize()
{
    return gcplist.size();
}
GCP GCPList::GCPPop() // Get last GCP correspondence.
{
    return gcplist.pop_up();
}