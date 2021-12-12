#ifndef LASER_H
#define LASER_H

#include "ranger.h"
//#include "line.h"

/*!
 *  \brief     Ranger base Class
 *  \details
 * This calss creates a laser type sensor object.
 *  \author    Chamath Edirisinhege
 *  \version   1.0
 *  \date      2021-08-31
 *  \bug       none reported as of 2021-09-07
 */

namespace laser {

    typedef enum {
      SICK_XL, /*!< Sick-XL model laser */
      OTHER /*!< for adding additional hardware in future*/
    } description; /*!< Available laser based sensor models*/

}



/**
 * @class Laser
 * @brief The Laser type sensor class This class is derived from the base class Ranger.
 * @details Used for Laser sensor creation.
 * Laser returns Nl, number of measurements, distances, which are related to the specified angular resolution.
 * Each measurements in a single point in space. The laser scans anticlockwise.
 */

class Laser: public Ranger
{
public:
    /**
     * @brief Laser default
     * @details
     * max range: 8
     * min range: 0.2
     * angular resolution: 10
     * fov: 180
     * position: 0,0,0
     * model:SICK-XL
     */
    Laser();

    /**
     * @brief Costructor which would set defaults variables except position
     * @param pose: position {x,y,theta}
     */
    Laser(ranger::SensorPose pose);


    /**
     * @brief Costructor which would set all the variables of the sensor
     * @param maximum range
     * @param minimum range
     * @param angular resolution
     * @param FOV - angle [deg]
     * @param position: x [m], y [m], angle [radians]
     */
    Laser(double maxr, double minr, unsigned int aRes, unsigned int fov,
          ranger::SensorPose pose, laser::description model);



    /**
     * @brief Generates raw data for the sensor
     * @return vector containing senor data
     * @details returning vector of doubles contain 'Nl' number of reading.
     * 'Nl' is an integer number of readings (FOV/angular resolution)+1.
     * The values should be distances to points within the FOV range between min and max range.
     *
     * The function calculates Nl number of distances, between min and max.
     */
    std::vector<double> generateData();

//function overrides
    /**
     * @brief Sets the agular resolution of the sensor based on the model
     * @param resolution [deg]
     * @return True if supported and actioned, false otherwise.
     * @details Actioned if the value is acceptable based on the model of the laser. (10 or 30 for SICK-XL)
     */
    bool setAngularResolution(unsigned int resolution);


    /**
     * @brief Sets the field of view (FOV) of the sensor based on the model
     * @param FOV value [deg]
     * @return True if actioned, false otherwise
     * @details FOV would be a value between 0 and 360 degrees for the command to be actioned (only 180 for SICK-XL, connot be changed for this model)
     */
    bool setFieldOfView(unsigned int fov);

private:
    unsigned int numofLines_; //!<number of data points per scan
    laser::description model_; //!< radius of circle

};

#endif // LASER_H
