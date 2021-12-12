#include <gtest/gtest.h>
#include <climits>

//This tool allows to identify the path of the package on your system
#include <ros/package.h>

//These allow us to inspect bags of data
#include <rosbag/bag.h>
#include <rosbag/view.h>

//We include our header file
#include "../src/dataprocessing.h"
#include "../src/helper.h"

//==================== BRIEF ====================//
/*
 * tests the destination function from helper.cpp
 * which is used to get a point x distance away from the point of interest
 * in any given direction.
 * This function is used find the poses to move to for the advanced mode and
 * the simple obstacle avoidance stuff
 *
 * Unit test the computation of the poses to go through to be behind OR to either side or robot
 *
 * Because of the way this function is used, ros bags are not really necesary for the testing
 * However, the wrapping function has been tested with a ros bag.
 *
 * the functionality of this function is tested here
 */
//==================== BRIEF ====================//

TEST(DataProcessing,point2index){

  //! The data has been saved in a bag, that is opened and used.
  //! Below command allows to find the folder belonging to a package (a3_skeleton is the package name not the folder name)
  std::string path = ros::package::getPath("a3_12977866");
  //! Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  //! The file is called smiley.bag
  std::string file = path + "scan_0_all.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default

  //We need to have a pointer to extract the data
  sensor_msgs::LaserScanConstPtr r0scan;
  nav_msgs::OdometryConstPtr r0;

  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/robot_0/base_scan"){
      r0scan = m.instantiate<sensor_msgs::LaserScan>();
    }
    if (r0scan != nullptr){

      break;
    }
  }

  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/robot_0/odom"){
      r0 = m.instantiate<nav_msgs::Odometry>();
    }
    if (r0 != nullptr){

      break;
    }
  }
  // Now we have the first grid in bag so we can proceed
  bag.close();

  ASSERT_NE(r0scan, nullptr);//Check that we have a grid from the bag
  ASSERT_NE(r0, nullptr);//Check that we have a grid from the bag


  DataProcessing process1;
  process1.update( *r0scan, r0->pose.pose);

  geometry_msgs::Pose testpoint;
  testpoint.position.x=0; testpoint.position.y=0; testpoint.position.z=0;
  testpoint.orientation = r0->pose.pose.orientation; // which is 0

  geometry_msgs::Pose p = process1.getSide(testpoint);
  EXPECT_NEAR(p.position.y,-0.5,0.1);

}

TEST(HelperTests,destination_1)
{
  geometry_msgs::Pose testpoint;
  testpoint.position.x=0; testpoint.position.y=0; testpoint.position.z=0;
  testpoint.orientation.w=1; testpoint.orientation.x=0;
  testpoint.orientation.y=0; testpoint.orientation.z=0;

  geometry_msgs::Point p = helper::destination(2,0,testpoint);
  EXPECT_NEAR(p.x,2,0.1);
}

TEST(HelperTests,destination_2)
{
  geometry_msgs::Pose testpoint;
  testpoint.position.x=0; testpoint.position.y=0; testpoint.position.z=0;
  testpoint.orientation.w=1; testpoint.orientation.x=0;
  testpoint.orientation.y=0; testpoint.orientation.z=0;

  geometry_msgs::Point p = helper::destination(2,M_PI,testpoint);
  EXPECT_NEAR(p.x,-2,0.1);
}

TEST(HelperTests,destination_3)
{
  geometry_msgs::Pose testpoint;
  testpoint.position.x=0; testpoint.position.y=0; testpoint.position.z=0;
  testpoint.orientation.w=1; testpoint.orientation.x=0;
  testpoint.orientation.y=0; testpoint.orientation.z=0;

  geometry_msgs::Point p = helper::destination(2,M_PI/2,testpoint);
  EXPECT_NEAR(p.y,2,0.1);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
