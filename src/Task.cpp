#include <Task.h>
#include <Planner.h>
#include <thread>
#include <chrono>
#include <iostream>

Task::Task(arp::Autopilot& autopilot, arp::ViEkf& viEkf, Eigen::Vector3d goal, const OccupancyMap& occupancyMap) : 
        autopilot_{autopilot}, viEkf_{viEkf}, goal_{goal}, occupancyMap_{occupancyMap} 
{
    autopilot.onDestinationReached(std::bind(&Task::destinationReachedCallback, std::ref(*this)));
}

void Task::execute()
{
    if (waypoints_.empty()) {
        // the current position is the start position, point A
        start_ = viEkf_.getPositionEstimate();
        std::cout << "Setting start point to: " << start_ << std::endl;
    }
    startJourney();
}

void Task::pause()
{
    autopilot_.setManual();
}

void Task::startJourney()
{
    if (!autopilot_.isFlying()) {
        autopilot_.takeoff();
        // std::this_thread::sleep_for(std::chrono::seconds(3)); // wait for the takeoff to finish
    }
    auto dest = currentJourney_ == Journey::ToGoal ? goal_ : start_;
    waypoints_ = Planner::planFlight(viEkf_.getPositionEstimate(), dest, occupancyMap_);
    autopilot_.flyPath(waypoints_);
    autopilot_.setAutomatic();
    std::cout << "CONTINUING" << std::endl;
}

void Task::destinationReachedCallback()
{
    // We are now heading back home (point A). Do not call [this] function
    // again when reaching home.
    autopilot_.clearDestinationReachedCallback();
    std::cout << "DESTINATION REACHED" << std::endl;

    autopilot_.setManual();
    autopilot_.land();
    std::this_thread::sleep_for(std::chrono::seconds(5)); // wait for the landing to finish
    currentJourney_ = Journey::ToStart;
    startJourney();
}