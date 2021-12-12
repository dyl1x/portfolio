#include "helper.h"

namespace helper {

double normaliseAngle(double theta)
{
  if (theta > (2 * M_PI))
    theta = theta - (2 * M_PI);
  else if (theta < 0)
    theta = theta + (2 * M_PI);
  return theta;
}

bool poseEquality(geometry_msgs::Pose p1, geometry_msgs::Pose p2, double offset)
{
  double xdiff = p1.position.x - p2.position.x;
  double ydiff = p1.position.y - p2.position.y;
  double odiff = normaliseAngle(tf::getYaw(p1.orientation)) - normaliseAngle(tf::getYaw(p2.orientation));

  if (fabs(xdiff) < offset && fabs(ydiff )< offset && fabs(odiff)<0.1)
  {
    return true;
  }

  return false;
}

double distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  double r = sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
  return  r;
}

double getAngle(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  double ang = atan2((p2.y-p1.y),(p2.x-p1.x));
  return normaliseAngle(ang);
}

geometry_msgs::Point destination(double distance, double orientation, geometry_msgs::Pose origin)
{
  double toffset = M_PI/2; //when orientation = 0 robot is looking in +x, offset makes it +y;
  geometry_msgs::Point d;
  double angle = normaliseAngle(orientation) + normaliseAngle(tf::getYaw(origin.orientation));
  d.x = origin.position.x + (distance*cos(normaliseAngle(angle)));
  d.y = origin.position.y + (distance*sin(normaliseAngle(angle)));
  d.z = 0.0;

  return d;
}

geometry_msgs::Point polar2cartecian(double range, double bearing)
{
  geometry_msgs::Point p;
  p.x = range*cos(bearing);
  p.y = range*sin(bearing);
  p.z = 0;
  return p;
}

}
