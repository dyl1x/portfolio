#include "dataprocessing.h"

DataProcessing::DataProcessing()
{
}

DataProcessing::~DataProcessing()
{}

DataProcessing::DataProcessing(sensor_msgs::LaserScan laserScan, geometry_msgs::Pose r0):
  scan_(laserScan),
  r0global_(r0)
{
}

void DataProcessing::update(sensor_msgs::LaserScan laserscan, geometry_msgs::Pose r0)
{
  scan_ = laserscan;
  r0global_ = r0;
}

geometry_msgs::Point DataProcessing::global2local(geometry_msgs::Point goal)
{
  geometry_msgs::Point p;
  p.x = goal.x - r0global_.position.x;
  p.y = goal.y - r0global_.position.y;
  p.z = 0;

  return p;
}

bool DataProcessing::findGoal(geometry_msgs::Point goal)
{
  float dist = static_cast<float>(sqrt((goal.x-r0global_.position.x)*(goal.x-r0global_.position.x) +
                                       (goal.y-r0global_.position.y)*(goal.y-r0global_.position.y)));
  goal = global2local(goal);
  //ROS_INFO_STREAM("g "<<goal.x <<" " << goal.y);
  double ro = tf::getYaw(r0global_.orientation);

  if (dist < scan_.range_max && dist > scan_.range_min) //is the point in range?
  {
    bool check = false;
    for (unsigned int i = 0; i < scan_.ranges.size(); i++) { //can we see it?
      if (scan_.intensities.at(i) > static_cast<float>(0.5))
      {
        double theta = static_cast<double>((scan_.angle_increment * i) + scan_.angle_min) + ro;
        geometry_msgs::Point p = helper::polar2cartecian(static_cast<double>(scan_.ranges.at(i)),theta);
        //ROS_INFO_STREAM("p "<<p.x <<" " << p.y<<" " << theta);
        double xdiff = goal.x - p.x;
        double ydiff = goal.y - p.y; //offset since the laser is not at the centre.
        //ROS_INFO_STREAM("d "<< xdiff << " " << ydiff);
        if ( (fabs(xdiff)<0.2) && (fabs(ydiff)<0.2) )
        {
          check = true;
          goalIndex_ = static_cast<int>(i);
        }
      }
    }
    return check;
  }
  else {
    goalIndex_ = -1;
    return false;
  }
}

geometry_msgs::Pose DataProcessing::getSide(geometry_msgs::Pose r1)
{
  geometry_msgs::Pose right;
  right.position = helper::destination(0.5,(-M_PI/2),r1);
  geometry_msgs::Pose left;
  left.position = helper::destination(0.5,(M_PI/2),r1);

  if (checkPoint(right.position))
  {
    right.orientation = r1.orientation;
    return right;
  }
  else if (checkPoint(left.position))
  {
    left.orientation = r1.orientation;
    return left;
  }
  else
  {
    return r0global_;
  }
}

bool DataProcessing::checkPoint(geometry_msgs::Point goal)
{
  unsigned int index = 0;
  unsigned int maxIndex = static_cast<unsigned int>((scan_.angle_max - scan_.angle_min)/scan_.angle_increment);
  if (point2index(goal,index))
  {
    bool check = false;
    unsigned int start = index;
    unsigned int end = start+2;
    if (!(index < 1))
    {
      start--;
    }
    else if (index > (maxIndex-1))
    {
      start = maxIndex-2;
    }

    while (check == false && start < end) {

      double dist = helper::distance(goal,r0global_.position);
      double diff = static_cast<double>(scan_.ranges.at(start)) - dist;
      if(diff>0.1)
      {
        check = true; //clear
      }
      else
      {
        check = false; //occupied
      }
      start++;
    }
    return check;
  }
  else
  {
    return false; //unkown or not clear
  }
}

bool DataProcessing::point2index(geometry_msgs::Point p, unsigned int & index)//point in local
{
  geometry_msgs::Point o;
  p = global2local(p);
  o.x = 0; o.y = 0; o.z = 0;
  double range = helper::distance(o,p);
  if (((range - static_cast<double>(scan_.range_max)) < 0) &&
      range > static_cast<double>(scan_.range_min))
  {
    double bearing = helper::getAngle(o,p) - helper::normaliseAngle(tf::getYaw(r0global_.orientation));
    bearing = helper::normaliseAngle(bearing);
    bearing = bearing + (static_cast<double>(scan_.angle_max - scan_.angle_min)/2);
    bearing = helper::normaliseAngle(bearing);

    if (bearing > static_cast<double>(scan_.angle_max - scan_.angle_min) ||
        bearing < static_cast<double>(scan_.angle_max + scan_.angle_min))
    {
      return false;
    }
    else
    {
      double ind = bearing/static_cast<double>(scan_.angle_increment);
      index = static_cast<unsigned int>(ind);
      return true;
    }
  }
  else {
    return false;
  }
}

bool DataProcessing::checkGap(double gap) //not fully implemented
{
  if (goalIndex_>-1)
  {
    //int offset = scan_.ranges.at(static_cast<unsigned int>(goalIndex_))
    int left = -1;
    int right = -1;

    unsigned int i = static_cast<unsigned int>(goalIndex_);
    while ((i < scan_.intensities.size()) && left == -1) {
      if (scan_.intensities.at(i) > static_cast<float>(0.1))
      {
        left = static_cast<int>(i);
      }
      i++;
    }

    i = static_cast<unsigned int>(goalIndex_);
    while ((i > 0) && right == -1) {
      if (scan_.intensities.at(i) > static_cast<float>(0.1))
      {
        right = static_cast<int>(i-1);
      }
      i--;
    }

    if (left > -1 && right > -1)
    {
      //double a = atan2(scan_.ranges.at(static_cast<unsigned int>(goalIndex_)),0.22);
      //ROS_INFO_STREAM("a "<<a);
      //ROS_INFO_STREAM("t "<<left<<" "<<right);
      if( (scan_.ranges.at(static_cast<unsigned int>(goalIndex_)) > scan_.ranges.at(static_cast<unsigned int>(left))) &&
          (scan_.ranges.at(static_cast<unsigned int>(goalIndex_)) > scan_.ranges.at(static_cast<unsigned int>(right)))   )
      {
        geometry_msgs::Point p1 = helper::polar2cartecian(static_cast<double>(scan_.ranges.at(static_cast<unsigned int>(left))),
                                                  ((static_cast<double>(scan_.angle_increment)*left)+1));
        geometry_msgs::Point p2 = helper::polar2cartecian(static_cast<double>(scan_.ranges.at(static_cast<unsigned int>(right))),
                                                  ((static_cast<double>(scan_.angle_increment)*right)+1));

        double detectedGap = helper::distance(p1,p2);
        //ROS_INFO_STREAM("gapd "<<detectedGap);


        if (detectedGap > gap)
        {
          return true;
        }
      }
    }
  }
  else
  {
    return false;
  }
  return false;
}
