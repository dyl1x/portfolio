#include "ranger.h"
#include <cmath>

Ranger::Ranger(double maxr, double minr,unsigned int aRes, unsigned int fov,
               ranger::SensorPose pose, ranger::SensingMethod method):

    maxRange_(maxr),
    minRange_(minr),   
    FOV_(fov),
    angResolution_(aRes),
    sequenceNum_(0),
    pose_(pose),
    sMethod_(method)

{
}

Ranger::~Ranger()
{
}

unsigned int Ranger::getAngularResolution(void)
{
    return angResolution_;
}

ranger::SensorPose Ranger::getSensorPose(void)
{
    return pose_;
}

unsigned int Ranger::getFieldOfView(void)
{
    return FOV_;
}

double Ranger::getMaxRange(void)
{
    return maxRange_;
}

double Ranger::getMinRange(void)
{
    return minRange_;
}

ranger::SensingMethod Ranger::getSensingMethod(void)
{
    return sMethod_;
}

bool Ranger::setAngularResolution(unsigned int resolution)
{
    switch (sMethod_) {
    case ranger::CONE:
        return false;
    case ranger::POINT:
        if (resolution < FOV_ && resolution > 0)
        {
            angResolution_ = resolution;
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool Ranger::setSensorPose(ranger::SensorPose pose)
{
    double TWO_PI = 2*M_PI;
    if (pose.theta > TWO_PI)
    {
        double theta =pose.theta;
        unsigned int num  = static_cast<unsigned int>(theta/(2*M_PI));
        double thetaNew = theta - (num*2*M_PI);
        pose.theta = thetaNew;
    }

    if (pose.theta < TWO_PI)
    {
        double theta = pose.theta * (-1);
        unsigned int num  = static_cast<unsigned int>(theta/(2*M_PI));
        double thetaNew = theta - (num*2*M_PI);
        pose.theta = thetaNew * (-1);
    }

    if (pose.theta < TWO_PI && pose.theta > (-TWO_PI))
    {
        pose_.theta += pose.theta;

        if (pose_.theta < 0)
        {
            pose_.theta = TWO_PI + pose_.theta;
        }
        if (pose_.theta > (TWO_PI))
        {
            pose_.theta = TWO_PI - pose_.theta;
        }

        pose_.x += pose.x;
        pose_.y += pose.y;
        return true;
    }
    else {
        return false;
    }
}

bool Ranger::setFieldOfView(unsigned int fov)
{

    if (fov < 360 && fov > 0)
    {
        FOV_ = fov;
        return true;
    }
    else
    {
        return false;
    }
}

double Ranger::getSequenceNum()
{
    return sequenceNum_;
}
