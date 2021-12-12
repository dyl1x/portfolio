/*! @file
 *  @brief Child class of PathFinder.
 *  @details Find where to go next in basic mode.
 *  @author Chamath Edirisinhege
 *  @date 02-10-2021
 *  @version 1.0
 *  @bug none reported as of 07-10-2021
*/
#ifndef BSCPATHFINDER_H
#define BSCPATHFINDER_H

#include <algorithm>
#include <limits>
#include "pathfinderbase.h"

class BscPathFinder: public PathFinderBase
{
public:
    /*!
     * \brief BscPathFinder
     * \param maxV - max velocity
     * \param maxG - max G force
     * \param minV - min velocity
     * \details Creates an object that does path planning for basic mode
     */
    BscPathFinder(double maxV, double maxG, double minV);

    /*!
      *\brief BscPathfinder Destructor
      */
    ~BscPathFinder();

    /*!
     * \brief getpath
     * \return
     * \details  Uses TSP algorithm to calculate the "best" order to navigate the points, visiting each
     * point at least once. Brute force mehtod from geeks for geeks. Modified to use vectors and doesnt add
     * the return distance.
     * \link <https://www.geeksforgeeks.org/traveling-salesman-problem-tsp-implementation/> {geeksforgeeks}
     */
    std::vector<geometry_msgs::Point> getPath();

protected:
    /*!
     * \brief addNodes
     * \details adds nodes to the matrix, doesnt update the path.
     */
    void addNodes();

    /*!
     * \brief getWeight
     * \param range from point a to b
     * \param bearing of point b from a
     * \return edge weight
     * \details uses the turn angle and the estimated travelling distance to calculate a
     * weight, which is the total time taken to reach the given coordinates from current position
     */
    double getWeight(double range, double bearing);
};

#endif // BSCPATHFINDER_H
