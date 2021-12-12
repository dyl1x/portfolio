#include "pilotbase.h"

PilotBase::PilotBase(std::shared_ptr<simulator::Simulator> sim):
    sim_(sim)
{
    controlLV_ = sim_->V_TERM + 0.01;
    controlAV_ = 0.0;

    running_ = false;
    updates_ = false;
    directions_ = false;
    bounds_ = false;
}

PilotBase::~PilotBase()
{
    //running_ = false;
    for (auto &t : threads_) {
        t.join();
    }
}

void PilotBase::start()
{
    running_ = true;
    threads_.push_back(sim_->spawn());
    threads_.push_back(std::thread(&PilotBase::control, this));
}

void PilotBase::control()
{

    double lv = 50;
    double av = 0.0;
    while (running_) {
        sim_->controlFriendly(lv,av);
        std::this_thread::sleep_for(std::chrono::milliseconds(26));
    }
}

void PilotBase::testPoses(std::vector<geometry_msgs::Point> goals)
{
    std::vector<geometry_msgs::Pose> poses;
    for (auto g : goals) {
        geometry_msgs::Pose p;
        p.orientation = tf::yawToQuaternion(0);
        p.position = g;
        poses.push_back(p);
    }
    sim_->testPose(poses);
}
