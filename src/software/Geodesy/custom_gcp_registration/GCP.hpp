#ifndef GCP_HPP
#define GCP_HPP

#include <string>

class GCP
{
private:
    double easting;
    double northing;
    double altitude;
    float pixelX;
    float pixelY;
    std::string image;

public:
    GCP(double x, double y, double z, float px, float py, std::string imageName);
    ~GCP();
    double GetEasting();
    double GetNorthing();
    double GetAltitude();
    float GetPixelX();
    float GetPixelY();
    std::string GetImageName();
}
#endif