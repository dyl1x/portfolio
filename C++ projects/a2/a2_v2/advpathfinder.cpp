#include "advpathfinder.h"
#include <iostream>
AdvPathFinder::AdvPathFinder(double maxV, double maxG, double minV):
    PathFinderBase (maxV, maxG, minV)
{

}

AdvPathFinder::~AdvPathFinder()
{

}

void AdvPathFinder::update(std::vector<geometry_msgs::RangeBearingStamped> rbsBogie, geometry_msgs::Pose loc,
                           std::vector<geometry_msgs::RangeVelocityStamped> rvsBogie)
{
    rbsBogie_ = rbsBogie;
    rvsBogie_ = rvsBogie;
    loc_ = loc;

    nodeDist_.clear();
    nodes_.clear();
}

std::vector<geometry_msgs::Point> AdvPathFinder::getPath()
{
    addNodes();
    std::vector<unsigned int> vertex;
    std::vector<unsigned int> out;
    for (unsigned int i = 1; i < nodes_.size(); i++ )
    {
        vertex.push_back(i);
    }

    double min_path_w = std::numeric_limits<double>::max();

    do {
        double cost = 0;

        unsigned int k=0;
        //unsigned int s=0; //used for return edge

        for (unsigned int i = 0; i < vertex.size(); i++) {
            cost += nodeDist_.at(k).at(vertex.at(i));
            k = vertex.at(i);
        }

        //cost += matrix.at(k).at(s); //we dont need to return to start

        min_path_w = std::min(min_path_w,cost);

        if((cost - min_path_w) < std::numeric_limits<double>::epsilon())
        {
            out = vertex;
        }

    } while (std::next_permutation(vertex.begin(),vertex.end()));

    //out.insert(out.begin(),0);

    std::vector<geometry_msgs::Point> goals;


    if (!((rvsBogie_.size() < rbsBogie_.size())) && oldNodes_.size() == nodes_.size() && out.size() > 0)
    {
        //rearrange to match
        rvsBogie_ = tf2::arrangeVelocity(rbsBogie_,rvsBogie_,loc_);
        oldNodes_ = tf2::arrangePoints(nodes_,oldNodes_,loc_);
        //continue
        for (unsigned int i=0;i<out.size();i++) {

            double time = tf2::getRange(nodes_.at(out.at(i)-1),nodes_.at(out.at(i)))/maxV_;

           time *= 1.0; //reduction gain to smooth the difference at large distances

            goals.push_back(tf2::addVector(nodes_.at(out.at(i)),oldNodes_.at(out.at(i)),
                                           rvsBogie_.at(out.at(i)-1).velocity, time));
        }
    }
    else {
        for (unsigned int i=0;i<out.size();i++) {
            goals.push_back(nodes_.at(out.at(i)));
        }
    }
    oldNodes_ = nodes_;
    return goals;
}

void AdvPathFinder::addNodes()
{
    set2Global();
    nodeDist_.clear();
    for (unsigned int i = 0; i < nodes_.size(); i++) {
        std::vector<double> row;
        for (unsigned int j = 0; j < nodes_.size(); j++) {
            row.push_back(0);
        }
        nodeDist_.push_back(row);
    } // matrix is now the correct, square

    //now we put the weights in

    for (unsigned int c = 0; c < nodeDist_.size(); c++) {

        for (unsigned int r = 0; r < nodeDist_.at(c).size(); r++) {
            double weight = getWeight(tf2::getRange(nodes_.at(c), nodes_.at(r)),
                                      tf2::getBearing(nodes_.at(c), nodes_.at(r))); //weight for stright line and turn from x axis
            double incomingCorrection = 0;
            if (c==0)
            {
                incomingCorrection = tf2::normalise180Angle(tf::quaternionToYaw(loc_.orientation));
                incomingCorrection = fabs(incomingCorrection)/((maxG_*9.81)/minV_);
            }

            //add it to matrix
            nodeDist_.at(c).at(r) = weight - incomingCorrection;
        }
    }
}

double AdvPathFinder::getWeight(double range, double bearing)
{
    double maxOmega = (maxG_*9.81)/minV_;
    bearing = std::fabs(bearing);
    double w = (bearing/maxOmega) + (range/maxV_);
    return w;
}
