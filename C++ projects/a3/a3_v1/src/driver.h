/*! @file
 *  @brief Ros interfacing class.
 *  @details Controls the publishing and subscribing, manages the communication between
 * all the components that tneed to talk to each other to keep the robot working.
 *  @author Chamath Edirisinhege
 *  @date 02-11-2021
 *  @version 1.0
 *  @bug none reported as of 02-11-2021
 * Known issues: robot runs into other robot when moving to the others side, this is when turning
 * 0.5 seems to be very close to each other, also the positioning of pathfollower is not a 100% accurate.
 * there seems to be a 3 degree error in yaw.
*/
#ifndef DRIVER_H
#define DRIVER_H

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"

#include <atomic>
#include <deque>
#include "grid_processing.h"
#include "dataprocessing.h"
#include "helper.h"

/*!
 * \brief contains structures and enumerations for neat data storage withing the class
 */
namespace DriverData {

typedef enum {
  BASIC=0,
  ADVANCED=1,
  DEFAULT=-1
} opMode; /*!< Available operation modes*/

typedef enum {
  MOVING=0,
  STOP=1,
  LONGSTOP=2,
  UNKONOWN=-1
} status; /*!< Available operation modes*/

struct PoseDataBuffer /*!< stores a pose and a mutex */
{
  geometry_msgs::Pose pose;
  std::mutex mtx;
};

struct OgMapBuffer /*!< stores a Ogmap and a mutex */
{
  nav_msgs::OccupancyGrid grid;
  std::mutex mtx;
};

struct LaserDataBuffer /*!< stores a laser scan and a mutex */
{
  sensor_msgs::LaserScan scan;
  std::mutex mtx;
};

struct CounterBuffer /*!< stores a cunter and a mutex, in case it needs to be shared */
{
  unsigned int count = 0;
  std::mutex mtx;
};

}



class Driver
{
public:

  /*!
   * \brief Driver
   * \param nh: node handle
   * \details Will take the node handle and initialize the callback and internal variables
   */
  Driver(ros::NodeHandle nh);

  /*!
   * \brief Driver destructor
   * \details Will tear down the object
   */
  ~Driver();

  //callbacks
  /*!
   * \brief Odometry Callback
   * \param nav_msgs::OdometryConstPtr - The odometry message
   * \note This function and the declaration are ROS specific
   */
  void r0OdomCallback(const nav_msgs::OdometryConstPtr& msg);

  /*!
   * \brief Odometry Callback
   * \param nav_msgs::OdometryConstPtr - The odometry message
   * \note This function and the declaration are ROS specific
   */
  void r1OdomCallback(const nav_msgs::OdometryConstPtr& msg);

  /*!
   * \brief LaserScan Callback
   * \param sensor_msgs::LaserScanConstPtr - The laserscan message
   * \note This function and the declaration are ROS specific
   */
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /*! \brief OccupancyGrid Callback
    * \param sensor_msgs::ImageConstPtr - The imageconst message
    * \note This function and the declaration are ROS specific
    */
  void occupancyGridCallback(const nav_msgs::OccupancyGridPtr & msg);

  /*!
   * \brief timerCallback
   * \param event - rostimer callback details, can be used to debug the callback.
   * ref roswiki for more info and usecases
   * \details increments the timer counter by 1 every tick (every second in this case).
   */
  void timerCallback(const ros::TimerEvent& event);

  //operations

  /*!
   * \brief runDriver
   * \details The main processing function that will run continously and utilise the data.
   * \note the function does not open a thread internally, however it is recomended that
   * this function is run in a seperate thread to ensure the callback are also processed.
   */
  void runDriver();

private:

  /*!
   * \brief stopPursuit
   * \details Sends an empty pose array to the path follower so it would abandon previous path
   * and stop. Also clears the path markers.
   */
  void stopPursuit();

  /*!
   * \brief adjustGoal
   * \param goal
   * \return add an offset to the end point of the path so that the robot stops before colliding
   * with the leader if the leader stops.
   */
  geometry_msgs::Pose adjustGoal(geometry_msgs::Pose goal);

  /*!
   * \brief simpleFollow
   * \param r0 - position of robot 0
   * \details Calculates the poses and publishes the pose array for movement
   */
  void simpleFollow(geometry_msgs::Pose r0, geometry_msgs::Pose r1, sensor_msgs::LaserScan lscan);

  /*!
   * \brief addGoal
   * \param goal - new waypoint
   * \details adds a new waypoint to the path
   */
  void addGoal(geometry_msgs::Pose goal);

  /*!
   * \brief advancedFollow
   * \details Manages the maneuvers for advanced mode
   */
  void advancedFollow(geometry_msgs::Pose r0, geometry_msgs::Pose r1, sensor_msgs::LaserScan lscan);

  /*!
   * \brief pubWayPoint
   * \param waypoint
   * \param r0 - pose of r0
   * \param lscan - laser scan of r0
   * \details sends the next waypoint
   */
  bool pubWayPoint(geometry_msgs::Pose waypoint, geometry_msgs::Pose r0, sensor_msgs::LaserScan lscan);

  /*!
   * \brief move2side
   * \param waypoint
   * \param r0 - pose of r0
   * \param lscan - laser scan of r0
   * \details adds posses in front the deque to try and move around a point.
   * if the way point is blocked and the obstacle is right in front, this function is called
   * hoping to move around
   */
  void move2side(geometry_msgs::Pose waypoint, geometry_msgs::Pose r0, sensor_msgs::LaserScan lscan);

  /*!
   * \brief markersFromPoses
   * \param pa - deque of poses
   * \return array of markers from the poses
   */
  visualization_msgs::MarkerArray markersFromPoses(std::deque<geometry_msgs::Pose> pa);

  /*!
   * \brief makeArrowMarker makes a marker of arrow shape with designated colour
   * \param pose - pose of the marker
   * \note the orientation is used to calculate an end point of the arrow, ie: where it is pointing,
   * and the length is assumed as 1.
   * \param [in|out] id - in an array all markers should have a unique id, so we pass
   * this by ref to be incremented.
   * \param arrowType - just changes the colour in this function.
   * 1 - green
   * 2 - blue
   * \return - a marker object as specified
   */
  visualization_msgs::Marker makeArrowMarker (geometry_msgs::Pose pose, int& id, unsigned int arrowType);

private:
  ros::NodeHandle nh_; /*!< Node handle */
  ros::Publisher viz_pub_; /*!< Visualisation marker publisher */
  ros::Publisher pathViz_pub_; /*!< Visualisation marker publisher for r0 path */
  ros::Publisher path_pub_; /*!< path publisher for r0 */

  ros::Subscriber sub1_, sub2_, sub3_, sub4_; /*!<Subscribers */

  ros::Timer timer_; /*!<ros timer */

  //data
  DriverData::opMode mode_; /*!< mode of operation */
  DriverData::OgMapBuffer ogMapBuffer_; /*!< Container for occupancygrid data and mutex */
  DriverData::PoseDataBuffer r0PoseDataBuffer_; /*!< Container for pose data and mutex */
  DriverData::PoseDataBuffer r1PoseDataBuffer_; /*!< Container for pose data and mutex */
  DriverData::LaserDataBuffer r0LaserData_; /*!< Container for laser data and mutex */

  std::deque<geometry_msgs::Pose> r0path_; /*!< Robot 0's path */
  geometry_msgs::Pose lastGoal_; /*!< Last known position of r1 */
  DriverData::status r1Status_; /*!< motion state of r1 */
  DriverData::CounterBuffer countSec_; /*!< counts seconds elapsed */
  bool timerTrigger_; /*!< True if we are waiting for the 10s timer */
  bool advManeuver_; /*!< True if we have already added advanced moves */
};

#endif // DRIVER_H
