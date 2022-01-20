#include "GCP.hpp"

GCP::GCP(double x, double y, double z, double px, double py, std::string imageName)
{
    easting = x;
    northing = y;
    altitude = z;
    Marker tmpMark;
    tmpMark.imageName = imageName;
    tmpMark.pixels = Vec2(px, py);
    markers.push_back(tmpMark);
}
GCP::~GCP()
{
}
Vec3 GCP::GetX(void)
{
    alignas(32) Vec3 tmp;
    tmp << easting, northing, altitude;
    return tmp;
    //return Vec3(easting, northing, altitude);
}
int GCP::GetObsSize()
{
    return markers.size();
}