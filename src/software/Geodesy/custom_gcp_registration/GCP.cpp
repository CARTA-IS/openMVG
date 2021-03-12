#include "GCP.hpp"

GCP::GCP(double x, double y, double z, float px, float py, std::string imageName);
GCP::~GCP();
double GCP::GetEasting();
double GCP::GetNorthing();
double GCP::GetAltitude();
float GCP::GetPixelX();
float GCP::GetPixelY();
std::string GCP::GetImageName();