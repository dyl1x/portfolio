#include "sonar.h"

#include <random>
#include <chrono>
#include <cmath>
//#include "processing.cpp"

Sonar::Sonar():
    Sonar(10.0,0.2,20,{0.0,0.0,0},sonar::SN_001)
{}

Sonar::Sonar(ranger::SensorPose pose):
    Sonar(10.0,0.2,20,pose,sonar::SN_001)
{}

Sonar::Sonar(double maxr, double minr, unsigned int fov, ranger::SensorPose pose, sonar::description model):
    Ranger (maxr,minr, 0,fov,pose,ranger::CONE),
    model_(model)
{
}

std::vector<double> Sonar::generateData()
{
    std::vector<double> data;

    //Let's use a random number generator for the data point
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(getMinRange(),getMaxRange());
    data.push_back(distribution(generator));
    sequenceNum_++;
    return data;
}
//function overrides
bool Sonar::setFieldOfView(unsigned int fov)
{
    switch (model_) {
    case sonar::SN_001:
        if (fov == 20)
        {
            FOV_ = fov;
            return true;
        }
        else {
            return false;
        }
    case sonar::OTHER:
        return  false;

    }
}

//extras
void Sonar::setScanArea()
{
    scanArea_ = (getMaxRange()*getMaxRange()*M_PI) - (getMinRange()*getMinRange()*M_PI);
    scanArea_ = scanArea_ * (getFieldOfView()/2);
}

double Sonar::getScanArea()
{
    setScanArea();
    return scanArea_;
}
