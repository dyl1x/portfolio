#ifndef LINE_H
#define LINE_H

namespace geometry_msgs {

    struct Point{
        double x;//!< x position
        double y;//!< y position
    };/*!< Structure for 2d Positions*/
}

/*!
 *  \brief     Line class
 *  \details
 * Creates a Line between given points in space, and stores its geometric properties
 *  \author    Chamath Edirisinhege
 *  \version   1.0
 *  \date      2021-08-31
 *  \bug       none reported as of 2021-09-07
 */

/**
 * @class Line
 * @details Lreates a line, and stores its geometric properties. Taken from quiz2 and then modified.
 */

class Line
{
public:
    /**
     * @brief Default constructor.
     * @param starting point, ending point
     * @details generates a line segment between the two points.
     */
    Line(geometry_msgs::Point pt1, geometry_msgs::Point pt2 );

    /**
     * @brief checks if a point is above the Line
     * @param pt - test point
     * @return true if test point is above the line
     */
    bool pointAboveLine(geometry_msgs::Point pt);

    /**
     * @brief gets the Gradient
     * @return gradient
     */
    double getGradient();

    /**
     * @brief gets the Y-Intercept
     * @return y intercept
     */
    double getYIntercept();

    /**
     * @brief get the two Points of the line segment
     * @param pt1 holding variable for first point
     * @param pt2 holding variable for first point
     */
    void getPoints(geometry_msgs::Point& pt1, geometry_msgs::Point& pt2);

private:
    /**
     * @brief generate a line from 2 Points
     * @param pt1 - first point
     * @param pt2 - second point
     */
    void fromPoints(geometry_msgs::Point pt1, geometry_msgs::Point pt2);

    /**
     * @brief set the Gradient
     * @param gradient
     */
    void setGradient(double gradient);

    /**
     * @brief set the Y-Intercept
     * @param y_intercept
     */
    void setYIntercept(double y_intercept);

private:
    double gradient_; //!< gradient
    double y_intercept_; //!< y intercept
    geometry_msgs::Point pt1_; //!< first point of the line segment
    geometry_msgs::Point pt2_; //!< second point of the line segment
};

#endif // LINE_H
