/*! @file
 *  @brief Interface class for PathFinder classes.
 *  @details Find where to go next.
 *  @author Chamath Edirisinhege
 *  @date 02-10-2021
 *  @version 1.0
 *  @bug none reported as of 07-10-2021
*/
#ifndef PATHFINDERINTERFACE_H
#define PATHFINDERINTERFACE_H

#include <vector>
#include "types.h"

class PathFinderInterface
{
public:
    PathFinderInterface(){}

    /*!
     * \brief PathFinderInterface destructor
     * \details Refer \link <https://stackoverflow.com/questions/3628529/should-c-interfaces-have-a-virtual-destructor> {stckoverflow}
     * for more info.
     */
    virtual ~PathFinderInterface(){}

    /*!
     * \brief update
     * \param rbsBogie - bogie data from friendly radar
     * \param loc - friendly location
     * \details updates the variables with the latest information recieved from the friendly
     */
    virtual void update(std::vector<geometry_msgs::RangeBearingStamped> rbsBogie, geometry_msgs::Pose loc) = 0;

    /*!
     * \brief getPath
     * \return vector of points in the global frame arranged in ascending order with the closes one is first.
     */
    virtual std::vector<geometry_msgs::Point> getPath() = 0;

};

#endif // PATHFINDERINTERFACE_H
