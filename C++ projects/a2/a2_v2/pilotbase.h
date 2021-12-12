#ifndef PILOTBASE_H
#define PILOTBASE_H

#include "pilotinterface.h"
/*! @file
 *  @brief Base class of Pilot.
 *  @details Controls the plane, travels at minimum speed in a straight line.
 *  @author Chamath Edirisinhege
 *  @date 02-10-2021
 *  @version 1.0
 *  @bug none reported as of 07-10-2021
*/
#include <thread>
#include <condition_variable>
#include <atomic>
#include <mutex>
#include <vector>
#include "types.h"
#include "tf.h"
#include "tf2.h"
#include "simulator.h"

class PilotBase: public PilotInterface
{
public:
    /*!
     * \brief PilotBase
     * \param sim - simulation shared pointer
     */
    PilotBase(std::shared_ptr<simulator::Simulator> sim);

    /*!
     * \brief PilotBase destructor
     */
    virtual ~PilotBase();

    /*!
     * \brief start
     * \details Starts the threads and assign work tot he threads
     */
    virtual void start();

protected:
    /*!
     * \brief control
     * \details Manage the inputs to the simulator, the linear velocity and the angular velocity
     */
    virtual void control();

    /*!
     * \brief getInformation
     * \details Querries the simulator and update the infromation about the bogies everytime
     * an update is available. This function will be in it own thread.
     */
    virtual void getInformation() = 0;

    /*!
     * \brief drive
     * \details This function uses the information to find out where to go and then
     * calculate how to get there.
     */
    virtual void drive() = 0;

    /*!
     * \brief testPoses
     * \param goals - vector of points
     * \details shows the points given on the display if its enabled within the simulation.
     */
    void testPoses (std::vector<geometry_msgs::Point> goals);

protected:
    //controls
    double controlLV_; /*!<Holds the most recent linear velocity value calculated */
    double controlAV_; /*!<Holds the most recent angular velocity value calculated */
    //checks
    std::atomic<bool> running_; /*!<indicates that the control loop is still running. */
    std::atomic<bool> updates_; /*!<indicates that the new bogie data is available */
    std::atomic<bool> directions_; /*!<indicates that the new control data is available */
    bool bounds_; /*!<out of bounds situation */

    //data
    std::vector<std::thread> threads_; /*!<contains all the threads created by the object */
    std::shared_ptr<simulator::Simulator> sim_; /*!<pointer to the simulator */
    std::vector<RangeVelocityStamped> rvsFromBase_; /*!<range and velocity of bogies from base */
    std::vector<RangeBearingStamped> rbsFromFriendly_; /*!<range and bearing of bogies from friendly */

    //mutexes and convars
    std::mutex mu_; /*!<controls access to the control variables*/
    std::condition_variable cond_; /*!<controls access to the control variables*/
    std::mutex mu2_; /*!<controls access to the data variables*/
    std::condition_variable cond2_; /*!<controls access to the control variables*/
};

#endif // PILOTBASE_H
