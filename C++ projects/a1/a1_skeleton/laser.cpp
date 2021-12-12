#include "laser.h"

#include <random>
#include <chrono>
#include <cmath>
//#include "processing.cpp"

Laser::Laser():
    Laser(8.0,0.2,10.0,180,{0.0,0.0,0},laser::SICK_XL)
{}

Laser::Laser(ranger::SensorPose pose):
    Laser(8.0,0.2,10.0,180,pose,laser::SICK_XL)
{}

Laser::Laser(double maxr, double minr, unsigned int aRes, unsigned int fov,
             ranger::SensorPose pose, laser::description model):
    Ranger(maxr,minr, aRes, fov,pose,ranger::POINT),
    model_(model)
{
}



std::vector<double> Laser::generateData()
{
    std::vector<double> data;
    numofLines_ = (getFieldOfView()/getAngularResolution())+1;

    //Let's use a random number generator for the data points
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(getMinRange(),getMaxRange());

    for (unsigned int i=0; i<numofLines_; i++) {
        data.push_back(distribution(generator));
    }
    //genGeometry(data);
    sequenceNum_ += numofLines_;
    return data;
}



//function overrides
bool Laser::setAngularResolution(unsigned int resolution)
{
    switch (model_) {
    case laser::SICK_XL:
        if (resolution == 10 || resolution == 30)
        {
            angResolution_ = resolution;
            return true;
        }
        else {
            return false;
        }
    case laser::OTHER:
        return  false;

    }
}

bool Laser::setFieldOfView(unsigned int fov)
{
    switch (model_) {
    case laser::SICK_XL:
        if (fov == 180)
        {
            FOV_ = fov;
            return true;
        }
        else {
            return false;
        }
    case laser::OTHER:
        return  false;

    }
}
