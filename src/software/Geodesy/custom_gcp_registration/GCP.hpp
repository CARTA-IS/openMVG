#ifndef GCP_HPP
#define GCP_HPP

#include <string>

class GCP
{
private:
    double easting;
    double northing;
    double altitude;
    float pixelY;
    std::string image;

public:
    std::vector<Marker> markers;
    GCP(double x, double y, double z, float px, float py, std::string imageName);
    ~GCP();
    bool operator==(GCP &ref){
        return (easting == ref.GetEasting() && northing == ref.GetNorthing() && altitude == ref.GetAltitude())};
    GCP operator+(GCP &ref)
    {
        markers.insert(markers.end(), ref.markers.begin(), ref.markers.end());
        return *this;
    }
    Vec3 GetX();
    int GetObsSize();
};

struct Marker
{
    Vec2 pixels;
    std::string imageName;
};
#endif