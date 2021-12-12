#include "driver.h"

Driver::Driver(ros::NodeHandle nh):
  nh_(nh),
  r1Status_(DriverData::status::UNKONOWN),
  timerTrigger_(false),
  advManeuver_(false)
{
  sub1_ = nh_.subscribe("robot_0/odom", 1000, &Driver::r0OdomCallback,this);
  sub2_ = nh_.subscribe("robot_0/base_scan", 10, &Driver::laserCallback,this);
  sub3_ = nh_.subscribe("local_map/local_map", 1, &Driver::occupancyGridCallback,this);
  sub4_ = nh_.subscribe("robot_1/odom", 1000, &Driver::r1OdomCallback,this);

  timer_ = nh.createTimer(ros::Duration(1.0), &Driver::timerCallback, this);

  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",1,false);
  pathViz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("robot_0/following",1,false);
  path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("robot_0/path", 1);

  ros::NodeHandle pn("~");
  std::string in;
  //arg '_param:=adv' for advanced
  pn.getParam("param", in);
  if (in.compare("adv")==0)
  {
    mode_ = DriverData::ADVANCED;
    ROS_INFO_STREAM("Running Advanced Mode");
  }
  else {
    mode_ = DriverData::BASIC;
    ROS_INFO_STREAM("Running Basic Mode");
  }
}

Driver::~Driver(){}

void Driver::r0OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // We store a copy of the pose and lock a mutex when updating
  std::unique_lock<std::mutex> lck (r0PoseDataBuffer_.mtx);
  r0PoseDataBuffer_.pose = msg->pose.pose;

}

void Driver::r1OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // We store a copy of the pose and lock a mutex when updating
  std::unique_lock<std::mutex> lck (r1PoseDataBuffer_.mtx);
  r1PoseDataBuffer_.pose = msg->pose.pose;

}

void Driver::occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg)
{

  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  ogMapBuffer_.grid = *msg;

}

void Driver::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  std::unique_lock<std::mutex> lck (r0LaserData_.mtx);
  r0LaserData_.scan = *msg;
}

void Driver::timerCallback(const ros::TimerEvent& event)
{
  std::unique_lock<std::mutex> lck (countSec_.mtx);
  countSec_.count++;
  ROS_DEBUG_STREAM("timer: " << event.last_real); //when last callback actually happened
}

void Driver::runDriver()
{
  ros::Duration(2.0).sleep();
  ros::Rate rate_limiter(10); //rate limit thread
  DataProcessing process1;

  while (ros::ok()) {
    std::unique_lock<std::mutex> lck1 (r0PoseDataBuffer_.mtx);
    geometry_msgs::Pose r0 = r0PoseDataBuffer_.pose;
    lck1.unlock();
    std::unique_lock<std::mutex> lck2 (r1PoseDataBuffer_.mtx);
    geometry_msgs::Pose r1 = r1PoseDataBuffer_.pose;
    lck2.unlock();
    std::unique_lock<std::mutex> lck3 (r0LaserData_.mtx);
    sensor_msgs::LaserScan scanTemp = r0LaserData_.scan;
    lck3.unlock();

    process1.update(scanTemp,r0);
    if(process1.findGoal(r1.position))
    {
      if (!helper::poseEquality(lastGoal_,r1,0.1))
      {
        addGoal(r1);
        //r1 is moving, if timer is on, turn off
        r1Status_ = DriverData::status::MOVING;
        advManeuver_ = false;
        if (timerTrigger_ == true)
        {
          timerTrigger_ = false;
        }
      }
      else
      {
        if (r1Status_ == DriverData::status::MOVING)
        {
          r1Status_ = DriverData::status::STOP;
          //start timer
          {
            std::unique_lock<std::mutex> lck (countSec_.mtx);
            countSec_.count = 0;
          }
          timerTrigger_ = true;
        }
        else if (r1Status_ == DriverData::status::STOP && mode_ == DriverData::opMode::ADVANCED)
        {
          unsigned int tempcount;
          {
            std::unique_lock<std::mutex> lck (countSec_.mtx);
            tempcount = countSec_.count;
          }
          //ROS_INFO_STREAM("tc: " << tempcount);
          if (tempcount > 10) //do advanced maneuver
          {
            timerTrigger_ = false;
            if (advManeuver_ == false)
            {
              advancedFollow(r0, r1, scanTemp);
              advManeuver_ = true;
            }

          }
        }
      }
    }
    else {
      ROS_INFO_THROTTLE(10,"Leader cant be seen :("); //every 10 seconds to prevent cluttering
      //ROS_INFO_STREAM("Leader cant be seen :(");
    }
    simpleFollow(r0,r1,scanTemp);
    rate_limiter.sleep();
  }
}

void Driver::stopPursuit()
{
  geometry_msgs::PoseArray nullarray;
  nullarray.poses.clear();
  path_pub_.publish(nullarray);

  visualization_msgs::MarkerArray nullmarker;
  nullmarker.markers.clear();
  pathViz_pub_.publish(nullmarker);
}

void Driver::simpleFollow(geometry_msgs::Pose r0, geometry_msgs::Pose r1, sensor_msgs::LaserScan lscan)
{
  if (!r0path_.empty())
  {
    if (helper::poseEquality(r0,r0path_.front(),0.1))
    {
      r0path_.pop_front();
      //ROS_INFO_STREAM("pose reached");

      if (!r0path_.empty())
      {
        bool c = pubWayPoint(r0path_.front(),r0,lscan);
        if(!c)
        {
          move2side(r0path_.front(),r0,lscan);
        }

        //ROS_INFO_STREAM("set new pose");
      }
    }
    else {
       bool c = pubWayPoint(r0path_.front(),r0,lscan);
       if(!c)
       {
         move2side(r0path_.front(),r0,lscan);
       }
      //ROS_INFO_STREAM("setting a new pose");
    }
  }
}

void Driver::advancedFollow(geometry_msgs::Pose r0, geometry_msgs::Pose r1, sensor_msgs::LaserScan lscan)
{
  //check if theres space
  DataProcessing advP(lscan, r0);
  geometry_msgs::Pose side = advP.getSide(r1);
  //check if the goal is the same
  if (!helper::poseEquality(side,r0path_.back(),0.1))
  {
    //movement
    geometry_msgs::Pose mid;
    mid.orientation = side.orientation;
    //ROS_INFO_STREAM("r1: " << tf::getYaw(r1.orientation));
    //ROS_INFO_STREAM("side: " << tf::getYaw(side.orientation));
    mid.position = helper::destination(0.5,M_PI,side);
    r0path_.push_back(mid);
    r0path_.push_back(side);
    ROS_INFO_STREAM("added advanced movements");
  }
}

bool Driver::pubWayPoint(geometry_msgs::Pose waypoint, geometry_msgs::Pose r0, sensor_msgs::LaserScan lscan)
{

  DataProcessing tempProcess(lscan,r0);
  if(tempProcess.checkPoint(waypoint.position)) //is it clear?
  {
    geometry_msgs::PoseArray path;
    path.poses.push_back(waypoint);
    path_pub_.publish(path);
    pathViz_pub_.publish(markersFromPoses(r0path_));
    return true;
  }
  else //no
  {
    if (tempProcess.checkPoint(helper::destination(0.5,0,r0))) //is it clear o.5 infront?
    {
      geometry_msgs::PoseArray path;
      path.poses.push_back(waypoint);
      path_pub_.publish(path);
      pathViz_pub_.publish(markersFromPoses(r0path_));
      return true; //keep moving
    }
    else //stop and move to side
    {
      ROS_INFO_STREAM("waypoint cannot be reached");
      stopPursuit();
      return false;
    }
  }

}

void Driver::move2side(geometry_msgs::Pose waypoint, geometry_msgs::Pose r0, sensor_msgs::LaserScan lscan)
{
  DataProcessing tprocess(lscan,r0);
  geometry_msgs::Pose side = tprocess.getSide(r0);
  side.orientation = waypoint.orientation;
  if (!helper::poseEquality(r0,side,0.1))
  {
    ROS_INFO_STREAM("moving to side");

    if (r0path_.size()>1)
    {
      r0path_.pop_front();
    }
    r0path_.push_front(side);
  }
}



//geometry_msgs::Pose side = tempProcess.getSide(waypoint);


void Driver::addGoal(geometry_msgs::Pose goal)
{
  if(r0path_.empty())
  {
    r0path_.push_back(adjustGoal(goal));
  }
  else
  {
    r0path_.pop_back();
    r0path_.push_back(lastGoal_);
    r0path_.push_back(adjustGoal(goal));
  }
}


geometry_msgs::Pose Driver::adjustGoal(geometry_msgs::Pose goal)
{
  lastGoal_ = goal;
  geometry_msgs::Pose newgoal;
  newgoal.position = helper::destination(0.5,M_PI,goal);
  newgoal.orientation = goal.orientation;

  return newgoal;
}

visualization_msgs::MarkerArray Driver::markersFromPoses(std::deque<geometry_msgs::Pose> pa)
{
  visualization_msgs::MarkerArray ma;
  int id = 0;
  visualization_msgs::Marker goal = makeArrowMarker(pa.back(),id,1);
  pa.pop_back();

  while (!pa.empty()) {
    ma.markers.push_back(makeArrowMarker(pa.front(),id,2));
    pa.pop_front();
  }
  ma.markers.push_back(goal);
  return ma;
}

visualization_msgs::Marker Driver::makeArrowMarker (geometry_msgs::Pose pose, int& id, unsigned int arrowType)
{
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color;
  switch (arrowType) {
  case 1:
  {
    color.a=0.5;//a is alpha - transparency 0.5 is 50%;
    color.r=0;
    color.g=1.0;
    color.b=0;
    break;
  }
  case 2:
  {
    color.a=0.5;
    color.r=0;
    color.g=0;
    color.b=1.0;
    break;
  }
  }

  // Set the frame ID and time stamp.
  marker.header.frame_id = "world";
  //single_marker_person.header.stamp = ros::Time();
  marker.header.stamp = ros::Time::now();

  marker.lifetime = ros::Duration(1.0); //seconds

  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "path";
  marker.id = id++;
  marker.type = visualization_msgs::Marker::ARROW;

  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point start = pose.position;
  geometry_msgs::Point end;
  double theta = tf::getYaw(pose.orientation);
  end.x = (0.2 * cos(theta)) + start.x;
  end.y = (0.2 * sin(theta)) + start.y;

  marker.points.push_back(start);
  marker.points.push_back(end);

  //scale.x is the shaft diameter, and scale.y is the head diameter.
  //If scale.z is not zero, it specifies the head length.
  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.scale.z = 0.0;

  marker.color = color;

  return marker;
}
