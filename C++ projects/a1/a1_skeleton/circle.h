#ifndef CIRCLE_H
#define CIRCLE_H

#include "line.h"

/*!
 *  \brief     Circle class
 *  \details
 * Creates a circle at a given point in space, and stores its geometric properties
 *  \author    Chamath Edirisinhege
 *  \version   1.0
 *  \date      2021-08-31
 *  \bug       none reported as of 2021-09-07
 */

/**
 * @class Circle
 * @details creates a circle, and stores its geometric properties. Taken from quiz2 and then modified.
 */
class Circle
{
public:
    /**
     * @brief Default constructor.
     * @details centre at (0,0) and raius 0
     */
    Circle();

    /**
     * @brief Default constructor.
     * @param radius
     * @details centre at (0,0)
     */
    Circle(double radius);

    /**
     * @brief Default constructor.
     * @param radius
     * @param centre point and radius
     */
    Circle(geometry_msgs::Point cent, double radius);

    /**
     * @brief destructor
     */
    ~Circle();

    /**
     * @brief sets the radius
     * @param radius
     */
    void setRadius(double radius);

    /**
     * @brief gets the radius
     * @return radius
     */
    double getRadius();

    /**
     * @brief gets the area
     * @return area
     */
    double getArea ();

private:
    double radius_; //!< radius of circle
    double area_; //!< area of circle
    geometry_msgs::Point centre_; //!< centre point of circle
};

#endif // CIRCLE_H
