/*! @file
 *  @brief Child class of PathFinder.
 *  @details Find where to go next in advanced mode.
 *  @author Chamath Edirisinhege
 *  @date 02-10-2021
 *  @version 1.0
 *  @bug none reported as of 07-10-2021
 *  @note Functional but not completed
*/
#ifndef ADVPATHFINDER_H
#define ADVPATHFINDER_H

#include "pathfinderbase.h"
#include <algorithm>
#include <limits>

class AdvPathFinder: public PathFinderBase
{
public:
    /*!
     * \brief AdvPathFinder
     * \param maxV - max velocity
     * \param maxG - max G force
     * \param minV - min velocity
     * \details Creates an object that does path planning for basic mode
     */
    AdvPathFinder(double maxV, double maxG, double minV);

    /*!
      *\brief AdvPathfinder Destructor
      */
    ~AdvPathFinder();

    /*!
     * \brief update
     * \param rbsBogie - bogie data from friendly radar
     * \param loc - friendly location
     * \param rvsBogie - range velocity of bogie from base
     * \note intended overload
     * \details updates the input variables of the class, this is to prevent from having to make a new
     * object for calculation. Also resets calculation variable in preperation for a new calculation
     * cycle
     */
    void update(std::vector<geometry_msgs::RangeBearingStamped> rbsBogie, geometry_msgs::Pose loc,
                std::vector<geometry_msgs::RangeVelocityStamped> rvsBogie);

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

private:
    std::vector<geometry_msgs::RangeVelocityStamped> rvsBogie_; /*!<bogie range and velocity in the global frame*/
    std::vector<geometry_msgs::Point> oldNodes_; /*!<saved points from last iteration*/
};

#endif // ADVPATHFINDER_H
