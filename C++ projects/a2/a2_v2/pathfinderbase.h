/*! @file
 *  @brief Base class for PathFinder classes.
 *  @details Finds where to go next.
 *  @author Chamath Edirisinhege
 *  @date 02-10-2021
 *  @version 1.0
 *  @bug none reported as of 07-10-2021
*/
#ifndef PATHFINDERBASE_H
#define PATHFINDERBASE_H

#include "pathfinderinterface.h"
#include "tf2.h"

typedef std::vector<std::vector<double> > matrix;

class PathFinderBase: public PathFinderInterface
{
public:
    PathFinderBase(double maxV, double maxG, double minV);

    virtual ~PathFinderBase();

    /*!
     * \brief update
     * \param rbsBogie - bogie data from friendly radar
     * \param loc - friendly location
     * \details updates the input variables of the class, this is to prevent from having to make a new
     * object for calculation. Also resets calculation variable in preperation for a new calculation
     * cycle
     */
    virtual void update(std::vector<geometry_msgs::RangeBearingStamped> rbsBogie, geometry_msgs::Pose loc);

    /*!
     * \brief getPath
     * \details Do something and find the order of interception and produce the waypoint for each interception.
     * \return vector of points in the global frame arranged in ascending order with the closes one is first.
     */
    virtual std::vector<geometry_msgs::Point> getPath() = 0;

protected:
    /*!
     * \brief set2Global
     * \details converts the bogies data to find their position in the global frame.
     * Then store it in \ref nodes_ to be used for path planning.
     * Position 0 of \ref nodes_ will always have the current known location of friendly.
     * Uses \ref tf2 functions.
     */
    void set2Global();

protected:
    //constants from the simulator
    double maxV_; /*!<Max linear velocity */
    double maxG_; /*!<Max G force */
    double minV_; /*!<Min linear velocity */

    std::vector<geometry_msgs::Point> nodes_; /*!<bogie positions in the global frame*/
    std::vector<geometry_msgs::RangeBearingStamped> rbsBogie_; /*!<bogie positions in the local frame*/
    geometry_msgs::Pose loc_; /*!<Friendly position */
    matrix nodeDist_; /*!<matrix of the weights to bogie positions */
};

#endif // PATHFINDERBASE_H
