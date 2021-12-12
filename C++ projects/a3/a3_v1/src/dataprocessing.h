/*! @file
 *  @brief Laser data processing class.
 *  @details exploits the incoming laser data to obtain information
 *  @author Chamath Edirisinhege
 *  @date 03-11-2021
 *  @version 1.0
 *  @bug none reported as of 11-11-2021, one function (checkgap) not fully implemented
*/
#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include "nav_msgs/Odometry.h"
#include <cmath>
#include "helper.h"

class DataProcessing
{
public:
  /*!
   * \brief DataProcessing default constructor
   * \details creates empty object
   */
  DataProcessing();

  /*!
   * \brief Destructor
   * \details Will tear down the obeject
   */
  ~DataProcessing();

  /*!
   * \brief DataProcessing
   * \details creates the object and initializes the internal variables
   * \param laserScan
   */
  DataProcessing(sensor_msgs::LaserScan laserScan, geometry_msgs::Pose r0);

  /*!
   * \brief update the internal variables with the latest
   * \param laserscan
   */
  void update(sensor_msgs::LaserScan laserscan, geometry_msgs::Pose r0);

  /*!
   * \brief findGoal
   * \param goal
   * \return true if an high intensity object is detected at the said location.
   * otherwise will return false
   * \details By default the function prioratises the right side assuming the leader is right handed.
   */
  bool findGoal(geometry_msgs::Point goal);

  /*!
   * \brief getSides
   * \param r1 - position of r1
   * \return The position 0.5m to the left or right of r1, orientation will be the same as that of r1
   * If there is no space the return will be the pose of r0
   * \details By default the function prioratises the right side assuming the leader is right handed.
   */
  geometry_msgs::Pose getSide(geometry_msgs::Pose r1);

  /*!
   * \brief checkFront
   * \param r1 - position of leader
   * \return True if there is an obstacle infront and its not the leader
   * \details We are checking a 10 degree cone front and conparing the position of leaeder
   * with the object found to determine if we should continue or not
   */
  bool checkPoint(geometry_msgs::Point goal);

  /*!
   * \brief point2index
   * \param p - a point
   * \param index - valiable to pass value of index
   * \return true if the value of index was changed
   * \details checks if the given point lies within the laser scan and returns the closest index
   * of the laser scan which would be reading that point in space.
   * checks against the last updated scan data available within the object
   */
  bool point2index(geometry_msgs::Point p, unsigned int & index);

private:

  /*!
   * \brief global2local
   * \return convert the coordinates into the frame of the robot0
   */
  geometry_msgs::Point global2local(geometry_msgs::Point goal);

  /*!
   * \brief checkGap
   * \note not fully implemented, thus private
   * \param gap - gap width
   * \return true if there is sace infornt which is larger and the input gap
   * \details The idea was to make sure that there is gap large enough to move through when
   * the goal is detected. would be handy for obstacle avoidance
   */
  bool checkGap(double gap);



private:
  sensor_msgs::LaserScan scan_; /*!< Laser scan */
  geometry_msgs::Pose r0global_; /*!< global position of the robot (owner of laser scan) */
  int goalIndex_; /*!< index of r1 in the Laser scan */
};

#endif // DATAPROCESSING_H
