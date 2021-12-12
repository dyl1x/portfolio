#include "bscpathfinder.h"

BscPathFinder::BscPathFinder(double maxV, double maxG, double minV):
    PathFinderBase (maxV, maxG, minV)
{

}

BscPathFinder::~BscPathFinder()
{

}

std::vector<geometry_msgs::Point> BscPathFinder::getPath()
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

    for (unsigned int i=0;i<out.size();i++) {
        goals.push_back(nodes_.at(out.at(i)));
    }

    return goals;
}

void BscPathFinder::addNodes()
{
    set2Global();
    for (unsigned int i = 0; i < nodes_.size(); i++) {
        std::vector<double> row;
        for (unsigned int j = 0; j < nodes_.size(); j++) {
            row.push_back(0);
        }
        nodeDist_.push_back(row);
    } // matrix is now the correct size nbogie+1, square

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

double BscPathFinder::getWeight(double range, double bearing)
{
    double maxOmega = (maxG_*9.81)/minV_;
    bearing = std::fabs(bearing);
    double w = (bearing/maxOmega) + (range/maxV_);
    return w;
}
