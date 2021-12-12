/*! @file
 *  @brief Child class of Pilot for advanced mode.
 *  @details Controls the plane and tell it how to get to the waypoint.
 * Uses \ref AdvPathFinder to find the waypoints.
 *  @author Chamath Edirisinhege
 *  @date 02-10-2021
 *  @version 1.0
 *  @bug none reported as of 07-10-2021
*/
#ifndef ADVPILOT_H
#define ADVPILOT_H

#include "pilotbase.h"
#include "tf2.h"
#include "advpathfinder.h"
#include "timer.h"

class AdvPilot: public PilotBase
{
public:
    /*!
     * \brief AdvPilot
     * \param sim - - simulation shared pointer
     */
    AdvPilot(std::shared_ptr<simulator::Simulator> sim);

    /*!
      *\brief AdvPolit Destructor
      */
    ~AdvPilot();

    /*!
     * \brief start
     * \details Starts the threads and assign work tot he threads
     */
    void start();

protected:
    /*!
     * \brief control
     * \details Manage the inputs to the simulator, the linear velocity and the angular velocity
     */
    void control();

    /*!
     * \brief getInformation
     * \details Querries the simulator and update the infromation about the bogies everytime
     * an update is available. This function will be in it own thread.
     */
    void getInformation();

    /*!
     * \brief drive
     * \details This function uses the information to find out where to go and then
     * calculate how to get there.
     */
    void drive();

    /*!
     * \brief setSpeeds
     * \param goal - waypoint
     * \details sets the speeds to reach the waypoint
     */
    void setSpeeds(geometry_msgs::Point goal);

    /*!
     * \brief checkBounds
     * \details checks if the current location is within 100m of the end of the air space.
     * \return true if close to outside.
     */
    bool checkBounds();
};

#endif // ADVPILOT_H
