/*! @file
 *  @brief Math helper class.
 *  @details performs calculations related to geometry
 *  @author Chamath Edirisinhege
 *  @date 03-11-2021
 *  @version 1.0
 *  @bug none reported as of 11-11-2021
*/
#ifndef HELPER_H
#define HELPER_H

#include <cmath>

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"

namespace helper {

/*!
 * \brief normaliseAngle
 * \param theta
 * \return theta if measured from the x-axis, anticlockwise
 */
double normaliseAngle(double theta);

/*!
 * \brief poseEquality
 * \note poses are \ref geometry_msg::Pose type
 * \param p1 - pose 1
 * \param p2 - pose 2
 * \param offset - allowable tolerance for comparison
 * \return true if the poses are within 0.1 difference
 */
bool poseEquality(geometry_msgs::Pose p1, geometry_msgs::Pose p2, double offset);

/*!
 * \brief distance
 * \param p1 - point 1
 * \param p2 - point 2
 * \return distance between the points
 */
double distance(geometry_msgs::Point p1, geometry_msgs::Point p2);

/*!
 * \brief getAngle
 * \param p1 - point 1
 * \param p2 - point 2
 * \return bearing of p2 from p1
 */
double getAngle(geometry_msgs::Point p1, geometry_msgs::Point p2);

/*!
 * \brief destination
 * \param distance - how far is the destination
 * \param orientation - relative orientation from goal (r1), anticlock wise is positive
 * ie: -pi/2 is on the right hand side of the origin point
 * \param origin - origin point
 * \return A point which is at the range and bearing from the origin provided in the inputs
 */
geometry_msgs::Point destination(double distance, double orientation, geometry_msgs::Pose origin);

/*!
 * \brief polar2cartecian
 * \param range
 * \param bearing
 * \return cartecian coordinate of the range and bearing input, in the local frame.
 */
geometry_msgs::Point polar2cartecian(double range, double bearing);

}

#endif // HELPER_H
