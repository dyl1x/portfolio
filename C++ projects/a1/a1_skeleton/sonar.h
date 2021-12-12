#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"
//#include "line.h"
//#include "circle.h"

/*!
 *  \brief     Sonar type sensor Class
 *  \details
 * This class creates a sonar type sensor object.
 *  \author    Chamath Edirisinhege
 *  \version   1.0
 *  \date      2021-08-31
 *  \bug       none reported as of 2021-09-07
 */


namespace sonar {

    typedef enum {
      SN_001, /*!< SN-001 model sonar */
      OTHER /*!< for adding additional hardware in future*/
    } description; /*!< Available sonar based sensor models*/
}

/**
 * @class Sonar
 * @brief The Sonar type sensor class. This class is derived from the base class Ranger.
 * @details Used for Sonar sensor creation
 */


class Sonar: public Ranger
{
public: 
    /**
     * @brief Sonar default
     * @details
     * max range: 10
     * min range: 0.2
     * fov: 20
     * position: 0,0,0
     * model:SN-001
     */
    Sonar();

    /**
     * @brief Costructor which would set defaults variables except angular resolution
     * @param pose: position {x,y,theta}
     */
    Sonar(ranger::SensorPose pose);

    /**
     * @brief Costructor which would set all the variables of the sensor
     * @param maximum range
     * @param minimum range
     * @param angular resolution
     * @param FOV - angle [deg]
     * @param position: x [m], y [m], angle [radians]
     */
    Sonar(double maxr, double minr, unsigned int fov, ranger::SensorPose pose, sonar::description model);

    /**
     * @brief Generates raw data for the sensor
     * @return vector containing senor data
     * @details Data from the sensor is just 1 distance measurement.
     */
    std::vector<double> generateData();


//function overrides

    /**
     * @brief Sets the field of view (FOV) of the sensor based on the model
     * @param FOV value [deg]
     * @return True if actioned, false otherwise
     * @details FOV would be a value between 0 and 360 degrees for the command to be actioned if supported (only 20 is supported by SN-001)
     */
    bool setFieldOfView(unsigned int fov);

//extras
    /**
     * @brief Calculates the area that is scaned by the sensor
     */
    void setScanArea();

    /**
     * @brief Gets the area that is scaned by the sensor
     * @details Calculates the area scanned by the sonar as a circle sector (pizza slice).
     */
    double getScanArea();

private:
    double scanArea_; /*!< scan area */
    sonar::description model_; /*!<sonar model*/
};

#endif // SONAR_H
