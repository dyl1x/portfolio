#include "circle.h"
#include <cmath>
#include <iostream>

Circle::Circle():
    Circle({0,0},0)
{
}

Circle::Circle(double radius):
    Circle({0,0},radius)
{
}

Circle::Circle(geometry_msgs::Point cent ,double radius):
    radius_(radius),
    centre_(cent)
{
    area_ = 2 * M_PI * radius;
}

Circle::~Circle(){

}




void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::getRadius()
{
    return radius_;
}


double Circle::getArea()
{
    return area_;
}

