#ifndef GCP_HPP
#define GCP_HPP

#include "openMVG/geometry/Similarity3.hpp"
using namespace openMVG;

struct Marker
{
    Vec2 pixels;
    std::string imageName;
};

class GCP
{
private:
    double easting;
    double northing;
    double altitude;
    std::string image;

public:
    std::vector<Marker> markers;
    GCP(double x, double y, double z, double px, double py, std::string imageName);
    ~GCP();
    bool operator==(GCP &ref)
    {
        return (easting == ref.easting && northing == ref.northing && altitude == ref.altitude);
    };
    GCP operator+(GCP &ref)
    {
        markers.insert(markers.end(), ref.markers.begin(), ref.markers.end());
        return *this;
    };
    Vec3 GetX(void);
    int GetObsSize();
};

#endif