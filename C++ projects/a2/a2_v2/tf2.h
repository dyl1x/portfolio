/*! @file
 *  @brief Collection of transforms between structures defined under types namespace
 *  @author Chamath Edirisinhege
 *  @date 02-10-2021
 *  @version 1.0
 *  @bug none reported as of 07-10-2021
*/
#ifndef TF2_H
#define TF2_H

#include <cmath>
#include "types.h"
#include "tf.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;

namespace tf2 {

/*!
 * \brief local2Global
 * \param rangeBearing - spherical coordinates w.r.t. local frame
 * \param localFrame - pose of the local frame
 * \return Point (x,y) in the global frame, of the spherical coordinates (range, beaering)
 */
Point local2Global(RangeBearingStamped rangeBearing, Pose localFrame);

/*!
 * \brief global2local
 * \param bogie - Rectangular coordinates of a point in the global frame
 * \param localFrame - pose of the local frame
 * \return spherical coordinates of the point in the local frame.
 */
RangeBearingStamped global2local(Point bogie, Pose localFrame);

/*!
 * \brief sperical2rectangular
 * \param range
 * \param bearing
 * \return Point (x,y) rectangular coordinates
 */
Point sperical2rectangular(double range, double bearing);

/*!
 * \brief rectangular2sperical
 * \param x
 * \param y
 * \return Point (range,bearing, 0) spherical coordinates.
 */
RangeBearingStamped rectangular2sperical(double x, double y);

/*!
 * \brief getRange
 * \param a - point 1
 * \param b - point 2
 * \return distance from a to b.
 */
double getRange(geometry_msgs::Point a, geometry_msgs::Point b);

/*!
 * \brief getBearing
 * \param a - point 1
 * \param b - point 2
 * \return bearing of point b from point a, starting from x axis of point a.
 * returns both positive and negative angles. CCW turn is positive.
 */
double getBearing(geometry_msgs::Point a, geometry_msgs::Point b);

/*!
 * \brief addVector - add the velocity correction
 * \param currentGoal - known bogie location
 * \param old - previous known position
 * \param bogieV - velocity of the bogie
 * \param time - weight/ time taken to visit bogies current position
 * \return predicted visiting point.
 */
Point addVector(Point currentGoal, Point old, double bogieV, double time);

/*!
 * \brief arrangeVelocity
 * \param rbs - vector of range bearing stamped local frame
 * \param rvs - vector of range bearing stamped global frame
 * \param loc - pose local frame
 * \return RangeVelocityStamped in order matched to range bearing stamped input
 */
std::vector<RangeVelocityStamped> arrangeVelocity (std::vector<RangeBearingStamped> rbs,
                                                   std::vector<RangeVelocityStamped> rvs, Pose loc);

/*!
 * \brief arrangePoints
 * \param nPoints - new points
 * \param oPoints - old points
 * \param loc - pose friendly
 * \details arrange the points in oPoints to closest matching order to nPoints based on the Range from
 * the point to position of loc. Using the bearing to compare causes many issues when friendly is turning.
 * \return points arranged in order
 */
std::vector<Point> arrangePoints (std::vector<Point> nPoints, std::vector<Point> oPoints, Pose loc);

/*!
 * \brief normaliseAngle
 * \param theta
 * \return theta if measured from the x-axis, anticlockwise
 */
double normaliseAngle(double theta);

/*!
 * \brief normalise180Angle
 * \param theta
 * \return
 */
double normalise180Angle(double theta);

/*!
 * \brief reversalize
 * \param angle
 * \return undo what \ref normalizeAngle does
 */
double reversalize(double angle);

}
#endif // TF2_H
