#include "tf2.h"
#include <iostream>
namespace tf2 {
std::vector<RangeVelocityStamped> arrangeVelocity (std::vector<RangeBearingStamped> rbs,
                                                   std::vector<RangeVelocityStamped> rvs, Pose loc)
{
    std::vector<RangeVelocityStamped> arranged(rvs);
    std::vector<Point> rbglobal;

    for (auto a: rbs) {
        rbglobal.push_back(local2Global(a,loc));
    }

    for (unsigned int b=0; b < rbs.size(); b++) {
        double min = 99999999;
        for (auto c : rvs ) {
            double diff = fabs(c.range - getRange(loc.position,rbglobal.at(b)));
            if (diff < min){
                min = diff;
                arranged.at(b) = c;
            }
        }
    }

    return arranged;
}

std::vector<Point> arrangePoints (std::vector<Point> nPoints, std::vector<Point> oPoints, Pose loc)
{
    std::vector<Point> arranged(oPoints);

    for (unsigned int n=1; n < nPoints.size(); n++) {
        double min = 9999999;
        double rn = getRange(loc.position,nPoints.at(n));

        for (unsigned int i = 1;i<oPoints.size();i++ ) {
            double ro = getRange(loc.position,oPoints.at(i));

            double diff = fabs(ro-rn);

            if (diff < min){
                min = diff;
                arranged.at(n) = oPoints.at(i);
            }
        }
    }
    return arranged;
}

Point local2Global(RangeBearingStamped rangeBearing, Pose localFrame)
{
    Point p;
    double localTilt = tf::quaternionToYaw(localFrame.orientation);
    Point bogie = sperical2rectangular(rangeBearing.range, rangeBearing.bearing);

    p.x = bogie.x*cos(localTilt) - bogie.y*sin(localTilt) + localFrame.position.x;
    p.y = bogie.x*sin(localTilt) + bogie.y*cos(localTilt) + localFrame.position.y;
    p.z = 0;

    return p;
}

RangeBearingStamped global2local(Point bogie, Pose localFrame)
{
    double localTilt = tf::quaternionToYaw(localFrame.orientation);
    Point blocal;

    blocal.x = (bogie.x-localFrame.position.x)*cos(localTilt) + (bogie.y-localFrame.position.y)*sin(localTilt);
    blocal.y = (localFrame.position.x-bogie.x)*sin(localTilt) + (bogie.y-localFrame.position.y)*cos(localTilt);
    blocal.z = 0;

    return rectangular2sperical(blocal.x,blocal.y);
}

Point sperical2rectangular(double range, double bearing)
{
    Point p;
    bearing = normaliseAngle(bearing);

    p.x = range*cos(bearing);
    p.y = range*sin(bearing);
    p.z = 0;

    return p;
}

RangeBearingStamped rectangular2sperical(double x, double y)
{
    RangeBearingStamped p = {0,0,0};

    p.range = sqrt((x*x)+(y*y));
    p.bearing = normaliseAngle(atan2(y,x));

    return p;
}

double getRange(geometry_msgs::Point a, geometry_msgs::Point b)
{
    double range = sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
    return  range;
}

double getBearing(geometry_msgs::Point a, geometry_msgs::Point b)
{
    double bearing = atan2((b.y-a.y),(b.x-a.x));

    return bearing;
}

Point addVector(Point currentGoal, Point old, double bogieV, double time)
{
    double correction = bogieV * time;
    Point p {0,0,0};
    double bearing = normaliseAngle(getBearing(old,currentGoal));

    p.x = correction*cos(bearing) + currentGoal.x;
    p.y = correction*sin(bearing) + currentGoal.y;

    return p;
}

double normaliseAngle(double theta)
{
  if (theta > (2 * M_PI))
    theta = theta - (2 * M_PI);
  else if (theta < 0)
    theta = theta + (2 * M_PI);

  return theta;
}

double normalise180Angle(double theta)
{
  if (theta > (M_PI))
    theta -= 2*M_PI;
  else if (theta < (-M_PI))
    theta += (2 * M_PI);

  return theta;
}

double reversalize(double angle) //lmao dont asky why
{
    if  (angle > M_PI)
    {
        angle = (2*M_PI) - angle;
    }
    return  angle;
}
}
