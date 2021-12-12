#ifndef PROCESSING_H
#define PROCESSING_H

#include "line.h"
#include <cmath>
#include <memory>
#include <complex>

/*!
 *  \brief     Processing Helper functions
 *  \details
 * A set of geometric manipulation and testing functions
 * Contains fucntions from analysis lybrary in quiz 2 and other useful functions
 *  \author    Chamath Edirisinhege
 *  \version   1.0
 *  \date      2021-08-31
 *  \bug       none reported as of 2021-09-07
 */



/**
 * @brief  checks of a point is on the Segment
 * @param p - point 1
 * @param q - point 2
 * @param r - point 3
 * @return true if q lies on pr
 */
bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);

/**
 * @brief To find orientation of ordered triplet (p, q, r).
 * @param p - point 1
 * @param q - point 2
 * @param r - point 3
 * @return 0 --> p, q and r are colinear
 * @return 1 --> Clockwise
 * @return 2 --> Counterclockwise
 */
int orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);

/**
 * @brief The main function that returns true if line segment 'p1q1' and 'p2q2' intersect
 * @param p1
 * @param q1
 * @param p2
 * @param q2
 * @return true it p1q1 intersect p2q2
 */
bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1,
                             geometry_msgs::Point p2, geometry_msgs::Point q2);

/**
 * @brief rotate a point in the x-y plane by theeta degrees Z
 * @param givenPoint
 * @param theeta [radians]
 * @return coordinates of the point after rotation
 * @details ref https://www.geeksforgeeks.org/rotation-of-a-point-about-another-point-in-cpp/
 * also allows reotating a point about any arbitary point. modified to only rotate about the origin.
 */
geometry_msgs::Point rotateAboutZ(geometry_msgs::Point givenPoint, double theeta);

/**
 * @brief Gives the square of the input
 * @param value
 * @return Returns the square
 * @details Taken from //https://rosettacode.org/wiki/Line_circle_intersection and modified
 * Only used in circleIntersects function.
 */
double sq(double x);

#endif // PROCESSING_H
