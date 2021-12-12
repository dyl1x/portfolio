#include <gtest/gtest.h>
#include <climits>

//This tool allows to identify the path of the package on your system
#include <ros/package.h>

//These allow us to inspect bags of data
#include <rosbag/bag.h>
#include <rosbag/view.h>

//We include our header file
#include "../src/dataprocessing.h"

//==================== BRIEF ====================//
/*
 * point2index function takes a point and find the closes index of the laser readings.
 * that function is tested using this script.
 * the function is used to find free space in the directions that we want to move in
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

  geometry_msgs::Point testpoint;
  testpoint.x=0; testpoint.y=-3; testpoint.z=0;
  unsigned int index = 999;
  EXPECT_TRUE(process1.point2index(testpoint,index));
  EXPECT_EQ(index,0);
}

TEST(DataProcessing,point2index_2){

  //! The data has been saved in a bag, that is opened and used.
  //! Below command allows to find the folder belonging to a package (a3_skeleton is the package name not the folder name)
  std::string path = ros::package::getPath("a3_12977866");
  //! Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  //! The file is called smiley.bag
  std::string file = path + "scan_1.bag";

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

  geometry_msgs::Point testpoint;
  testpoint.x=0; testpoint.y=-3; testpoint.z=0;
  unsigned int index = 999;
  EXPECT_TRUE(process1.point2index(testpoint,index));
  EXPECT_EQ(index,88);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
