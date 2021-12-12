/*! @file
 *  @brief Interface class for Pilot classes.
 *  @details Find How to go to where ever it needs to go.
 *  @author Chamath Edirisinhege
 *  @date 02-10-2021
 *  @version 1.0
 *  @bug none reported as of 07-10-2021
*/
#ifndef PILOTINTERFACE_H
#define PILOTINTERFACE_H


class PilotInterface
{
public:
    PilotInterface(){}

    /*!
     * \brief ~PilotInterface
     * * \details Refer \link <https://stackoverflow.com/questions/3628529/should-c-interfaces-have-a-virtual-destructor> {stckoverflow}
     * for more info.
     */
    virtual ~PilotInterface(){}

    /*!
     * \brief start
     * \details Starts the threads and assign work tot he threads
     */
    virtual void start() = 0;

    /*!
     * \brief control
     * \details Manage the inputs to the simulator, the linear velocity and the angular velocity
     * at the specified regular intervals. This function will be in its own thread.
     */
    virtual void control() = 0;

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

};

#endif // PILOTINTERFACE_H
