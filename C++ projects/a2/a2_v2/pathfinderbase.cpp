#include "pathfinderbase.h"

PathFinderBase::PathFinderBase(double maxV, double maxG, double minV):
    maxV_(maxV),
    maxG_(maxG),
    minV_(minV)
{

}

PathFinderBase::~PathFinderBase()
{

}

void PathFinderBase::update(std::vector<geometry_msgs::RangeBearingStamped> rbsBogie, geometry_msgs::Pose loc)
{
    rbsBogie_ = rbsBogie;
    loc_ = loc;
    nodeDist_.clear();
    nodes_.clear();
}

void PathFinderBase::set2Global()
{
    double fx = loc_.position.x;
    double fy = loc_.position.y;
    geometry_msgs::Point f {fx,fy,0};
    nodes_.push_back(f);

    for (unsigned int i=0; i<rbsBogie_.size(); i++) {
        geometry_msgs::Point point = tf2::local2Global(rbsBogie_.at(i),loc_);
        nodes_.push_back(point);
    }
    //now all the nodes are in the vector, in global coordinate frame.
}
