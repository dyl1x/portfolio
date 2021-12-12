
#include "processing.h"
#include <limits>

using std::min;
using std::max;


//// Given three colinear points p, q, r, the function checks if
//// point q lies on line segment 'pr'
bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
       return true;

    return false;
}

//// To find orientation of ordered triplet (p, q, r).
//// The function returns following values
//// 0 --> p, q and r are colinear
//// 1 --> Clockwise
//// 2 --> Counterclockwise
int orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    double val = (
                (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y)
                );

    if (val < -1e-7) return 2;//counter clockwise
    else if (val > 1e-7) {
        return 1; //clock wise
    }
    else {
        return 0; //colinear
    }

//    if (val < 1e-14 && ) return 0;  // colinear

//    return (val > 0)? 1: 2; // clock or counterclock wise
}

//// The main function that returns true if line segment 'p1q1'
//// and 'p2q2' intersect.
bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1,
                             geometry_msgs::Point p2, geometry_msgs::Point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false;
}


//#define PI 3.1415926535897932384626

//rotates the coordinates x,y in point around the z axis by theeta radians (positive anticlockwise)
//ref https://www.geeksforgeeks.org/rotation-of-a-point-about-another-point-in-cpp/
//also allows reotating a point about any arbitary point. modified to only rotate about the origin.
geometry_msgs::Point rotateAboutZ(geometry_msgs::Point givenPoint, double theeta)
{

    std::complex <double> point (givenPoint.x,givenPoint.y);
    point = point * std::polar(1.0,theeta);

    givenPoint.x = point.real();
    givenPoint.y = point.imag();
    return givenPoint;
}
//fuction to square
//https://rosettacode.org/wiki/Line_circle_intersection
double sq(double x) {
    return x * x;
}



