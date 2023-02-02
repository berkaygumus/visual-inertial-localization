#include <Task.h>
#include <Planner.h>
#include <thread>
#include <chrono>
#include <iostream>

Task::Task(arp::Autopilot& autopilot, arp::ViEkf& viEkf, Eigen::Vector3d goal, const OccupancyMap& occupancyMap) : 
        autopilot_{autopilot}, viEkf_{viEkf}, goal_{goal}, occupancyMap_{occupancyMap} 
{
    autopilot_.onDestinationReached(std::bind(&Task::goalReachedCallback, std::ref(*this)));
}

void Task::execute()
{
    if (running_) return;
    running_ = true;

    if (!initialized_) {
        // the current position is the start position, point A
        start_ = viEkf_.getPositionEstimate();
        if (!autopilot_.isFlying()) {
            // set Z to 1 if the start point is set while landed.. 
            // otherwise the drone kamikazes
            start_(2) = 1;
        }
        std::cout << "Setting start point to: [" << start_(0) << "," << start_(1) << "," << start_(2) << "]" << std::endl;
        initialized_ = true;
    }
    startJourney();
}

void Task::pause()
{
    if (!running_) return;
    running_ = false;
    autopilot_.setManual();
}

void Task::startJourney()
{
    if (!autopilot_.isFlying()) {
        autopilot_.takeoff();
        std::this_thread::sleep_for(std::chrono::seconds(3)); // wait for the takeoff to finish
    }
    auto dest = currentJourney_ == Journey::ToGoal ? goal_ : start_;
    autopilot_.flyPath(Planner::planFlight(viEkf_.getPositionEstimate(), dest, occupancyMap_));
    autopilot_.setAutomatic();
}

void Task::goalReachedCallback()
{
    std::cout << "Goal reached." << std::endl;
    autopilot_.land();
    std::this_thread::sleep_for(std::chrono::seconds(5)); // wait for the landing to finish
    currentJourney_ = Journey::ToStart;
    autopilot_.onDestinationReached(std::bind(&Task::startReachedCallback, std::ref(*this)));
    startJourney();
}

void Task::startReachedCallback()
{
    autopilot_.clearDestinationReachedCallback();
    std::cout << "Start reached." << std::endl;
}