#include <gtest/gtest.h>
#include <climits>

//This tool allows to identify the path of the package on your system
#include <ros/package.h>

//These allow us to inspect bags of data
#include <rosbag/bag.h>
#include <rosbag/view.h>

//We include our header file
#include "../src/dataprocessing.h"

//==================== HELPER FUNCTIONS ====================//


//==================== HELPER FUNCTIONS ====================//

//==================== BRIEF ====================//
/*
 * These tests test if robot 1 is detected properly using the laser scans.
 * images of the setup can be found in /sample_images
 * the names correspond the rosbag used.
 */
//==================== BRIEF ====================//


TEST(DataProcessing,LineOfSightTest){

  //! The data has been saved in a bag, that is opened and used.
  //! Below command allows to find the folder belonging to a package (a3_skeleton is the package name not the folder name)
  std::string path = ros::package::getPath("a3_12977866");
  //! Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  //! The file is called smiley.bag
  std::string file = path + "los_all.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default

  //We need to have a pointer to extract the data
  sensor_msgs::LaserScanConstPtr r0scan;
  nav_msgs::OdometryConstPtr r0;
  nav_msgs::OdometryConstPtr r1;

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

  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/robot_1/odom"){
      r1 = m.instantiate<nav_msgs::Odometry>();
    }
    if (r1 != nullptr){

      break;
    }
  }
  // Now we have the first grid in bag so we can proceed
  bag.close();

  ASSERT_NE(r0scan, nullptr);//Check that we have a grid from the bag
  ASSERT_NE(r0, nullptr);//Check that we have a grid from the bag
  ASSERT_NE(r1, nullptr);//Check that we have a grid from the bag

  DataProcessing process1;
  process1.update( *r0scan, r0->pose.pose);

  EXPECT_TRUE(process1.findGoal(r1->pose.pose.position));

}

TEST(DataProcessing,LineOfSightTest_2){

  //! The data has been saved in a bag, that is opened and used.
  //! Below command allows to find the folder belonging to a package (a3_skeleton is the package name not the folder name)
  std::string path = ros::package::getPath("a3_12977866");
  //! Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  //! The file is called smiley.bag
  std::string file = path + "los_2_all.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default

  //We need to have a pointer to extract the data
  sensor_msgs::LaserScanConstPtr r0scan;
  nav_msgs::OdometryConstPtr r0;
  nav_msgs::OdometryConstPtr r1;

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

  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/robot_1/odom"){
      r1 = m.instantiate<nav_msgs::Odometry>();
    }
    if (r1 != nullptr){

      break;
    }
  }
  // Now we have the first grid in bag so we can proceed
  bag.close();

  ASSERT_NE(r0scan, nullptr);//Check that we have a grid from the bag
  ASSERT_NE(r0, nullptr);//Check that we have a grid from the bag
  ASSERT_NE(r1, nullptr);//Check that we have a grid from the bag

  DataProcessing process1;
  process1.update( *r0scan, r0->pose.pose);

  EXPECT_TRUE(process1.findGoal(r1->pose.pose.position));

}

TEST(DataProcessing,LineOfSightTest_3){

  //! The data has been saved in a bag, that is opened and used.
  //! Below command allows to find the folder belonging to a package (a3_skeleton is the package name not the folder name)
  std::string path = ros::package::getPath("a3_12977866");
  //! Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  //! The file is called smiley.bag
  std::string file = path + "los_3_all.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default

  //We need to have a pointer to extract the data
  sensor_msgs::LaserScanConstPtr r0scan;
  nav_msgs::OdometryConstPtr r0;
  nav_msgs::OdometryConstPtr r1;

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

  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/robot_1/odom"){
      r1 = m.instantiate<nav_msgs::Odometry>();
    }
    if (r1 != nullptr){

      break;
    }
  }
  // Now we have the first grid in bag so we can proceed
  bag.close();

  ASSERT_NE(r0scan, nullptr);//Check that we have a grid from the bag
  ASSERT_NE(r0, nullptr);//Check that we have a grid from the bag
  ASSERT_NE(r1, nullptr);//Check that we have a grid from the bag

  DataProcessing process1;
  process1.update( *r0scan, r0->pose.pose);

  EXPECT_FALSE(process1.findGoal(r1->pose.pose.position));

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
