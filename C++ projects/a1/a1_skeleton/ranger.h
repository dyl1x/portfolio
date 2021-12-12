#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include <string>

/*!
 *  \brief     Ranger base Class
 *  \details
 * This base class is used to set all the methods that need to be embodied within any subsequent derived sensor classes.
 * Contains all the setters and getters for sensor variables.
 *  \author    Chamath Edirisinhege
 *  \version   1.0
 *  \date      2021-08-31
 *  \bug       none reported as of 2021-09-07
 */

/**
 * @class Ranger
 * @brief The Ranger base class for sensors. This class is derived from the interface class ranger interface
 * @details Used for sensor creation
 */

class Ranger: public RangerInterface
{
public:
    /**
     * @brief Costructor which would set all the variables of the sensor
     * @param maximum range
     * @param minimum range
     * @param angular resolution
     * @param FOV - angle [deg]
     * @param position: x [m], y [m], angle [radians]
     * @param Sensing method: cone or point?
     */
    Ranger(double maxr, double minr,unsigned int aRes, unsigned int fov,
           ranger::SensorPose pose, ranger::SensingMethod method);
    //See rangerinterface.h for more information

    virtual ~Ranger(); //!Ranger is an abstract class and requires a virtual destructor

    /**
     * @brief generate raw Data
     * @return  vector of data points
     */
    virtual std::vector<double> generateData() = 0;

    /**
     * @brief Gets the angular resolution
     * @return angular resolution of the sensor
     */
    unsigned int getAngularResolution(void);

    /**
     * @brief Gets the sensor position information
     * @return x [m], y [m] position coordinate and angle [radians]
     */
    ranger::SensorPose getSensorPose(void);

    /**
     * @brief Gets sensor field of view (FOV). FOV of point based sensors, is zero
     * @return the FOV angle [deg]
     */
    unsigned int getFieldOfView(void);

    /**
     * @brief Gets the maximum range of the sensor
     * @return maximum range [m]
     */
    double getMaxRange(void);

    /**
     * @brief Gets the minimum range of the sensor
     * @return minimum range [m]
     */
    double getMinRange(void);

    /**
     * @brief Gets the sensing method
     * @return sensing method
     */
    ranger::SensingMethod getSensingMethod(void);

    /**
     * @brief Sets the agular resolution of the sensor
     * @param resolution [deg]
     * @return True if supported and actioned, false otherwise.
     * @details As the default it will be actioned and always true. Defines the angular 'gap' between
     * two consecutive readings. Must be between 0 and FOV.
     */
    virtual bool setAngularResolution(unsigned int resolution);

    /**
     * @brief Sets an offset to the original position and angle of the sensor
     * @param pose: x position [m], y position [m], angle [radians]
     * @return True if actioned, false otherwise
     * @details Theta value has to be between 2PI and 0, therefore when it has been off set by 2PI it has returned to
     * it initial position. The if statement makes sure that theta would like between 0 and 2PI, by adding or
     * substracting 2PI from theta when it goes past the respective boundries.
     */
    bool setSensorPose(ranger::SensorPose pose);

    /**
     * @brief Sets the field of view (FOV) of the sensor
     * @param FOV value [deg]
     * @return True if actioned, false otherwise
     * @details FOV would be a value between 0 and 360 degrees for the command to be actioned
     */
    virtual bool setFieldOfView(unsigned int fov);

    /**
     * @brief Gets the number of readings taken by the sensor
     * @return number of readings taken by the sensor
     */
    double getSequenceNum();


protected:
    double maxRange_; //!maximum range of the sensor
    double minRange_; //!minimum range of the sensor
    unsigned int FOV_; //!field of view of the sensor
    unsigned int angResolution_ = 0; //!angular resolution of the sensor
    unsigned int sequenceNum_ = 0;//!Number of times the sensr has generated data

private:
    ranger::SensorPose pose_; //!sensor position (x,y) and sensor angle in radians
    ranger::SensingMethod sMethod_; //!sensing method
};

#endif // RANGER_H
