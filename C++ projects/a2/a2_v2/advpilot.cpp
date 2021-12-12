#include "advpilot.h"

AdvPilot::AdvPilot(std::shared_ptr<simulator::Simulator> sim):
    PilotBase (sim)
{

}

AdvPilot::~AdvPilot()
{
    //running_ = false;
    for (auto &t : threads_) {
        t.join();
    }
}

void AdvPilot::start()
{
    running_ = true;
    threads_.push_back(sim_->spawn());
    threads_.push_back(std::thread(&AdvPilot::getInformation, this));
    threads_.push_back(std::thread(&AdvPilot::control, this));
    threads_.push_back(std::thread(&AdvPilot::drive, this));
}

void AdvPilot::control()
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
            locker2.unlock();
            cond2_.notify_one();
        }

        sim_->controlFriendly(lv,av);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

void AdvPilot::getInformation()
{
    Timer t1;
    while (running_) {
        {
            std::unique_lock<std::mutex> locker (mu_);
            rbsFromFriendly_ = sim_->rangeBearingToBogiesFromFriendly();
            if (t1.elapsed() > 100)
            {
                rvsFromBase_ = sim_->rangeVelocityToBogiesFromBase();
                t1.reset();
            }
            updates_ = true;
            cond_.notify_all();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

void AdvPilot::drive()
{
    std::vector<geometry_msgs::Point> goals;
    AdvPathFinder path1(sim_->V_MAX, sim_->G_MAX, sim_->V_TERM);
    Timer t1;
    while (running_) {
        {
            std::unique_lock<std::mutex> locker (mu_);
            cond_.wait(locker, [&]() {return updates_.load();});
            path1.update(rbsFromFriendly_,sim_->getFriendlyPose(),rvsFromBase_);
            updates_ = false;
            locker.unlock();
            cond_.notify_one();
        }

        goals = path1.getPath();
        if (checkBounds())
        {
            setSpeeds(geometry_msgs::Point {0,0,0});
        }
        else {
            setSpeeds(goals.at(0));
        }

        if (t1.elapsed() > 2000)
        {
            t1.reset();
            testPoses(goals);
        }
    }
}

void AdvPilot::setSpeeds(geometry_msgs::Point goal)
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
        }
        else if (fabs(thetaError) > 1.1772) {
            controlAV_ = std::copysign(1.177,thetaError);
            controlLV_ = ((sim_->G_MAX) *9.81)/fabs(controlAV_);
        }
        else {
            controlAV_ = thetaError;
            controlLV_ = ((sim_->G_MAX)*9.81)/controlAV_;
        }

        directions_ = true;
        cond2_.notify_one();
        //std::cout << "given" << std::endl;
    }
}

bool AdvPilot::checkBounds()
{

    double edge = (sim_->AIRSPACE_SIZE/2)-200;
    geometry_msgs::Point loc = sim_->getFriendlyPose().position;
    if ( edge < fabs(loc.x) || edge < fabs(loc.y))
    {
        return true;
    }
    return false;
}
