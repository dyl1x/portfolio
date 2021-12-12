#include "bscpilot.h"

BscPilot::BscPilot(std::shared_ptr<simulator::Simulator> sim):
    PilotBase (sim)
{

}

BscPilot::~BscPilot()
{
    //running_ = false;
    for (auto &t : threads_) {
        t.join();
    }
}

void BscPilot::start()
{
    running_ = true;
    threads_.push_back(sim_->spawn());
    threads_.push_back(std::thread(&BscPilot::getInformation, this));
    threads_.push_back(std::thread(&BscPilot::drive, this));
    threads_.push_back(std::thread(&BscPilot::control, this));

}

void BscPilot::control()
{
    double lv = 50;
    double av = 0.0;
    while (running_) {
        if (directions_)
        {
            std::unique_lock<std::mutex> locker2 (mu2_);
            cond2_.wait(locker2, [&](){return directions_ == true;});
            lv = controlLV_;
            av = controlAV_;
            directions_ = false;
            cond2_.notify_one();
            //std::cout << "used" << std::endl;
        }
        sim_->controlFriendly(lv,av);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

void BscPilot::getInformation()
{
    while (running_) {

        {
            std::unique_lock<std::mutex> locker (mu_);
            rbsFromFriendly_ = sim_->rangeBearingToBogiesFromFriendly();
            updates_ = true;
            cond_.notify_one();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void BscPilot::drive()
{
    std::vector<geometry_msgs::Point> goals;
    BscPathFinder path1(sim_->V_MAX,sim_->G_MAX,sim_->V_TERM);
    Timer t1;
    while (running_) {

        {
            std::unique_lock<std::mutex> locker (mu_);
            cond_.wait(locker, [&]() {return updates_ == true;});
            path1.update(rbsFromFriendly_,sim_->getFriendlyPose());
            updates_ = false;
            cond_.notify_one();
        }
        goals = path1.getPath();

        unsigned int count = 0;
        while (count<goals.size()) {

            if (checkBounds())
            {
                setSpeeds(geometry_msgs::Point {0,0,0});
            }
            else {
                setSpeeds(goals.at(count));
            }
            geometry_msgs::Pose p= sim_->getFriendlyPose();
            double xdiff = fabs(goals.at(count).x) - fabs(p.position.x);
            double ydiff = fabs(goals.at(count).y) - fabs(p.position.y);
            if ( fabs(xdiff) < 100 && fabs(ydiff) < 100) count++;
            if (t1.elapsed() > 2000)
            {
                t1.reset();
                testPoses(goals);
            }
        }
    }
}

void BscPilot::setSpeeds(geometry_msgs::Point goal)
{
    geometry_msgs::Point loc = sim_->getFriendlyPose().position;
    double fOrient = tf::quaternionToYaw(sim_->getFriendlyPose().orientation);
    fOrient = tf2::normalise180Angle(fOrient);

    double thetaError = (atan2((goal.y-loc.y),(goal.x-loc.x))) - fOrient;
    thetaError = tf2::normalise180Angle(thetaError);
    thetaError *= 0.6; //gain

    {
        std::unique_lock<std::mutex> locker2 (mu2_);
        if (fabs(thetaError) < 0.05886)
        {
            controlAV_ = thetaError;
            controlLV_ = sim_->V_MAX;
            //std::cout << "1" << std::endl;
        }
        else if (fabs(thetaError) > 1.1772) {
            controlAV_ = std::copysign(1.177,thetaError);
            controlLV_ = ((sim_->G_MAX) *9.81)/fabs(controlAV_);
            //std::cout << "2" << std::endl;
        }
        else {
            controlAV_ = thetaError;
            controlLV_ = ((sim_->G_MAX)*9.81)/controlAV_;
            //std::cout << "3" << std::endl;
        }
        directions_ = true;
        cond2_.notify_one();
        //std::cout << "given" << std::endl;
    }

}

bool BscPilot::checkBounds()
{

    double edge = (sim_->AIRSPACE_SIZE/2)-200;
    geometry_msgs::Point loc = sim_->getFriendlyPose().position;
    if ( edge < fabs(loc.x) || edge < fabs(loc.y))
    {
        return true;
    }
    return false;
}
